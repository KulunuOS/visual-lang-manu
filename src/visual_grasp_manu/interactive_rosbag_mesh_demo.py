from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path

import yaml


def run_command(command: list[str], *, env: dict[str, str] | None = None) -> None:
    print("+ " + " ".join(command), flush=True)
    completed = subprocess.run(command, env=env, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {completed.returncode}: {' '.join(command)}")


def start_process(command: list[str], *, env: dict[str, str] | None = None) -> subprocess.Popen:
    print("+ " + " ".join(command), flush=True)
    return subprocess.Popen(command, env=env, preexec_fn=os.setsid)


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=8)


def ask_yes_no(question: str) -> bool:
    answer = input(f"{question} [y/N]: ").strip().lower()
    return answer in {"y", "yes"}


def demo_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/visual_grasp_manu/log/interactive_rosbag_mesh_demo")
    env.setdefault("MPLCONFIGDIR", "/tmp/visual_grasp_manu/matplotlib")
    return env


def source_note() -> None:
    if "AMENT_PREFIX_PATH" not in os.environ:
        print(
            "This command expects the ROS overlay to be sourced, for example:\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source /tmp/visual_grasp_manu/install/setup.bash\n",
            file=sys.stderr,
        )


def remove_dataset(path: Path, *, overwrite: bool) -> None:
    if path.exists() and overwrite:
        shutil.rmtree(path)
    if path.exists():
        raise RuntimeError(f"Output path already exists. Use --overwrite or choose another path: {path}")


def mark_review_dataset(path: Path) -> None:
    metadata_path = path / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise RuntimeError(f"metadata.yaml must contain a mapping: {metadata_path}")
    metadata["mode"] = "partial_geometry_smoke_test"
    metadata["multi_view"] = False
    metadata["notes"] = "Temporary first-frame mask review dataset for interactive demo."
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")


def export_scan(
    *,
    bag_path: Path,
    output_path: Path,
    prompt: str,
    frame_stride: int,
    max_frames: int,
    bag_rate: float,
    loop_bag: bool,
    database_path: Path,
    env: dict[str, str],
) -> None:
    run_command(
        [
            "ros2",
            "launch",
            "visual_grasp_manu",
            "object_scan_dataset_export.launch.py",
            f"bag_path:={bag_path}",
            f"output_path:={output_path}",
            f"object_prompt:={prompt}",
            f"frame_stride:={frame_stride}",
            f"max_frames:={max_frames}",
            f"bag_rate:={bag_rate}",
            f"loop_bag:={'true' if loop_bag else 'false'}",
            f"database_path:={database_path}",
        ],
        env=env,
    )


def run_interactive_demo(args: argparse.Namespace) -> None:
    source_note()
    env = demo_env()
    output_path = args.output_path
    first_frame_path = Path(str(output_path) + "_first_frame_review")
    remove_dataset(output_path, overwrite=args.overwrite)
    remove_dataset(first_frame_path, overwrite=args.overwrite)

    rviz_process: subprocess.Popen | None = None
    prompt = args.prompt
    if not prompt:
        rviz_process = start_process(
            [
                "ros2",
                "launch",
                "visual_grasp_manu",
                "object_scan_camera_pose.launch.py",
                f"bag_path:={args.bag_path}",
                "rviz:=true",
                "play_bag:=true",
                f"bag_rate:={args.preview_bag_rate}",
                f"database_path:={args.preview_database_path}",
            ],
            env=env,
        )
        try:
            prompt = input("Enter object query after RViz point cloud is visible: ").strip()
        finally:
            print("Pausing preview by stopping the initial RViz/bag playback process.", flush=True)
            stop_process(rviz_process)

    if not prompt:
        raise RuntimeError("Object query cannot be empty.")

    print("Exporting first frame for mask proposal review.", flush=True)
    export_scan(
        bag_path=args.bag_path,
        output_path=first_frame_path,
        prompt=prompt,
        frame_stride=args.frame_stride,
        max_frames=1,
        bag_rate=args.export_bag_rate,
        loop_bag=False,
        database_path=args.first_frame_database_path,
        env=env,
    )
    mark_review_dataset(first_frame_path)

    mask_command = [
        "ros2",
        "run",
        "visual_grasp_manu",
        "interactive_track_masks",
        str(first_frame_path),
        "--prompt",
        prompt,
        "--review-mode",
        args.review_mode,
        "--limit",
        "1",
        "--max-center-jump-px",
        str(args.max_center_jump_px),
        "--min-area-ratio",
        str(args.min_area_ratio),
        "--max-area-ratio",
        str(args.max_area_ratio),
        "--max-depth-jump-m",
        str(args.max_depth_jump_m),
    ]
    add_backend_args(mask_command, args)
    run_command(mask_command, env=env)

    initial_overlay = first_frame_path / "interactive_review" / "000001_initial_overlay.png"
    print(f"Initial mask overlay saved at: {initial_overlay}", flush=True)
    if args.review_mode == "auto" and not args.assume_yes and not ask_yes_no(
        "Auto review was used. Continue with full tracking anyway?"
    ):
        raise RuntimeError("Tracking rejected by user.")

    print("Mask accepted. Exporting full scan while rosbag loops until max_frames.", flush=True)
    export_scan(
        bag_path=args.bag_path,
        output_path=output_path,
        prompt=prompt,
        frame_stride=args.frame_stride,
        max_frames=args.max_frames,
        bag_rate=args.export_bag_rate,
        loop_bag=True,
        database_path=args.export_database_path,
        env=env,
    )

    full_track_command = [
        "ros2",
        "run",
        "visual_grasp_manu",
        "interactive_track_masks",
        str(output_path),
        "--prompt",
        prompt,
        "--review-mode",
        "auto",
        "--max-center-jump-px",
        str(args.max_center_jump_px),
        "--min-area-ratio",
        str(args.min_area_ratio),
        "--max-area-ratio",
        str(args.max_area_ratio),
        "--max-depth-jump-m",
        str(args.max_depth_jump_m),
    ]
    add_backend_args(full_track_command, args)
    run_command(full_track_command, env=env)

    accepted_frames = output_path / "accepted_frames.txt"
    run_command(
        [
            "ros2",
            "run",
            "visual_grasp_manu",
            "preview_scan_masks",
            str(output_path),
            "--columns",
            "2",
            "--tile-width",
            "220",
        ],
        env=env,
    )
    run_command(
        [
            "ros2",
            "run",
            "visual_grasp_manu",
            "generate_tsdf_mesh",
            str(output_path),
            "--frame-list",
            str(accepted_frames),
            "--voxel-length",
            str(args.voxel_length),
            "--sdf-trunc",
            str(args.sdf_trunc),
            "--depth-trunc",
            str(args.depth_trunc),
            "--min-mask-pixels",
            str(args.min_mask_pixels),
        ],
        env=env,
    )
    if args.open_mesh_viewer:
        run_command(["ros2", "run", "visual_grasp_manu", "visualize_mesh", str(output_path)], env=env)
    else:
        run_command(
            ["ros2", "run", "visual_grasp_manu", "visualize_mesh", str(output_path), "--no-view"],
            env=env,
        )


def add_backend_args(command: list[str], args: argparse.Namespace) -> None:
    command.extend(["--backend", args.backend])
    if args.backend == "grounding_dino_sam2":
        command.extend(
            [
                "--grounding-config",
                str(args.grounding_config),
                "--grounding-checkpoint",
                str(args.grounding_checkpoint),
                "--sam2-config",
                args.sam2_config,
                "--sam2-checkpoint",
                str(args.sam2_checkpoint),
                "--device",
                args.device,
                "--grounding-device",
                args.grounding_device,
                "--box-threshold",
                str(args.box_threshold),
                "--text-threshold",
                str(args.text_threshold),
            ]
        )
    elif args.backend == "hsv_color":
        command.extend(
            [
                "--hsv-lower",
                args.hsv_lower,
                "--hsv-upper",
                args.hsv_upper,
                "--hsv-min-area",
                str(args.hsv_min_area),
            ]
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a realtime-interactive rosbag-to-mask-to-mesh demo with RViz preview."
    )
    parser.add_argument("--bag-path", type=Path, default=Path("outputs/datasets/object_scan_001"))
    parser.add_argument("--output-path", type=Path, default=Path("outputs/datasets/object_scan_interactive_realtime_demo"))
    parser.add_argument("--prompt", default="", help="Optional object query. Empty asks interactively after RViz starts.")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--review-mode", choices=["terminal", "gui", "auto"], default="terminal")
    parser.add_argument(
        "--assume-yes",
        action="store_true",
        help="Continue after auto review without asking for confirmation. Intended for smoke tests.",
    )
    parser.add_argument("--preview-bag-rate", type=float, default=0.5)
    parser.add_argument("--export-bag-rate", type=float, default=0.5)
    parser.add_argument("--frame-stride", type=int, default=5)
    parser.add_argument("--max-frames", type=int, default=60)
    parser.add_argument("--preview-database-path", type=Path, default=Path("/tmp/visual_grasp_manu/rtabmap_interactive_preview.db"))
    parser.add_argument("--first-frame-database-path", type=Path, default=Path("/tmp/visual_grasp_manu/rtabmap_interactive_first_frame.db"))
    parser.add_argument("--export-database-path", type=Path, default=Path("/tmp/visual_grasp_manu/rtabmap_interactive_export.db"))

    parser.add_argument("--backend", choices=["grounding_dino_sam2", "box_stub", "hsv_color"], default="grounding_dino_sam2")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--grounding-device", default="cpu")
    parser.add_argument("--grounding-config", type=Path, default=Path("/tmp/visual_grasp_manu/model_repos/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"))
    parser.add_argument("--grounding-checkpoint", type=Path, default=Path("/tmp/visual_grasp_manu/checkpoints/groundingdino_swint_ogc.pth"))
    parser.add_argument("--sam2-config", default="configs/sam2.1/sam2.1_hiera_t.yaml")
    parser.add_argument("--sam2-checkpoint", type=Path, default=Path("/tmp/visual_grasp_manu/checkpoints/sam2.1_hiera_tiny.pt"))
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--hsv-lower", default="90,90,25")
    parser.add_argument("--hsv-upper", default="135,255,255")
    parser.add_argument("--hsv-min-area", type=int, default=50)

    parser.add_argument("--max-center-jump-px", type=float, default=80.0)
    parser.add_argument("--min-area-ratio", type=float, default=0.35)
    parser.add_argument("--max-area-ratio", type=float, default=2.8)
    parser.add_argument("--max-depth-jump-m", type=float, default=0.12)
    parser.add_argument("--voxel-length", type=float, default=0.003)
    parser.add_argument("--sdf-trunc", type=float, default=0.015)
    parser.add_argument("--depth-trunc", type=float, default=1.5)
    parser.add_argument("--min-mask-pixels", type=int, default=50)
    parser.add_argument("--open-mesh-viewer", action="store_true")
    args = parser.parse_args(argv)

    run_interactive_demo(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
