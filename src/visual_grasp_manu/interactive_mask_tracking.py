from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import yaml

from visual_grasp_manu.mask_generation import (
    MaskBackend,
    build_backend,
    create_mask_overlay,
    ensure_binary_mask,
    mask_box_xyxy,
    read_rgb_image,
    write_json,
)
from visual_grasp_manu.scan_dataset import format_report, read_yaml_file, validate_scan_dataset
from visual_grasp_manu.tsdf_mesh import read_depth


@dataclass(frozen=True)
class TrackStats:
    frame: str
    area_pixels: int
    center_x: float
    center_y: float
    median_depth_m: float | None


@dataclass(frozen=True)
class TrackDecision:
    frame: str
    accepted: bool
    reason: str
    stats: TrackStats


@dataclass(frozen=True)
class InteractiveTrackingResult:
    dataset_path: Path
    frames_processed: int
    frames_accepted: int
    accepted_frames_path: Path
    summary_path: Path
    initial_overlay_path: Path


def generate_interactive_tracked_masks(
    dataset_path: Path | str,
    *,
    backend: MaskBackend,
    prompt: str | None = None,
    accept_initial: Callable[[Path], bool] | None = None,
    auto_accept_initial: bool = False,
    max_center_jump_px: float = 80.0,
    min_area_ratio: float = 0.35,
    max_area_ratio: float = 2.8,
    max_depth_jump_m: float = 0.12,
    limit: int = 0,
    save_overlays: bool = True,
) -> InteractiveTrackingResult:
    path = Path(dataset_path)
    report = validate_scan_dataset(path, stage="capture")
    if not report.ok:
        raise ValueError(format_report(report))

    errors: list[str] = []
    metadata = read_yaml_file(path / "metadata.yaml", errors, required=True)
    intrinsics = read_yaml_file(path / "camera_intrinsics.yaml", errors, required=True)
    if errors:
        raise ValueError("\n".join(errors))

    object_prompt = prompt or str(metadata.get("object_prompt", "")).strip()
    if not object_prompt:
        raise ValueError("A text prompt is required. Set metadata.yaml object_prompt or pass --prompt.")
    depth_scale = float(intrinsics["depth_scale"])

    rgb_frames = sorted((path / "rgb").glob("*.png"))
    if limit > 0:
        rgb_frames = rgb_frames[:limit]
    if not rgb_frames:
        raise ValueError(f"No RGB frames found in {path / 'rgb'}")

    masks_dir = path / "masks"
    annotations_dir = path / "mask_annotations"
    overlays_dir = path / "mask_overlays"
    review_dir = path / "interactive_review"
    masks_dir.mkdir(parents=True, exist_ok=True)
    annotations_dir.mkdir(parents=True, exist_ok=True)
    overlays_dir.mkdir(parents=True, exist_ok=True)
    review_dir.mkdir(parents=True, exist_ok=True)

    initial_rgb = rgb_frames[0]
    initial_prediction = backend.predict(initial_rgb, object_prompt)
    initial_mask = ensure_binary_mask(initial_prediction.mask)
    initial_overlay_path = review_dir / f"{initial_rgb.stem}_initial_overlay.png"
    write_overlay(initial_overlay_path, initial_rgb, initial_mask)

    accepted = auto_accept_initial
    if not accepted:
        if accept_initial is None:
            accept_initial = terminal_accept_initial
        accepted = accept_initial(initial_overlay_path)
    if not accepted:
        raise RuntimeError(f"Initial mask rejected: {initial_overlay_path}")

    decisions: list[TrackDecision] = []
    accepted_frames: list[str] = []
    previous_stats: TrackStats | None = None

    for rgb_path in rgb_frames:
        prediction = initial_prediction if rgb_path == initial_rgb else backend.predict(rgb_path, object_prompt)
        mask = ensure_binary_mask(prediction.mask)
        depth_path = path / "depth" / f"{rgb_path.stem}.npy"
        depth = read_depth(depth_path)
        stats = compute_track_stats(rgb_path.stem, mask, depth, depth_scale=depth_scale)
        accept, reason = accept_track_stats(
            stats,
            previous_stats,
            max_center_jump_px=max_center_jump_px,
            min_area_ratio=min_area_ratio,
            max_area_ratio=max_area_ratio,
            max_depth_jump_m=max_depth_jump_m,
        )
        output_mask = mask if accept else np.zeros_like(mask)
        cv2.imwrite(str(masks_dir / f"{rgb_path.stem}.png"), output_mask)
        if save_overlays:
            write_overlay(overlays_dir / f"{rgb_path.stem}.png", rgb_path, output_mask)

        annotation = {
            "frame": rgb_path.stem,
            "rgb_path": str(rgb_path.relative_to(path)),
            "mask_path": str((masks_dir / f"{rgb_path.stem}.png").relative_to(path)),
            "prompt": object_prompt,
            "backend": f"{backend.name}_interactive_tracking",
            "score": prediction.score,
            "phrase": prediction.phrase,
            "box_xyxy": list(mask_box_xyxy(output_mask)) if accept and mask_box_xyxy(output_mask) is not None else None,
            "accepted": accept,
            "tracking_reason": reason,
            "tracking_stats": asdict(stats),
        }
        write_json(annotations_dir / f"{rgb_path.stem}.json", annotation)
        decisions.append(TrackDecision(rgb_path.stem, accept, reason, stats))
        if accept:
            accepted_frames.append(rgb_path.stem)
            previous_stats = stats

    accepted_frames_path = path / "accepted_frames.txt"
    accepted_frames_path.write_text("\n".join(accepted_frames) + "\n", encoding="utf-8")
    summary_path = annotations_dir / "tracking_summary.json"
    write_json(
        summary_path,
        {
            "dataset_id": path.name,
            "prompt": object_prompt,
            "backend": f"{backend.name}_interactive_tracking",
            "frames_processed": len(rgb_frames),
            "frames_accepted": len(accepted_frames),
            "accepted_frames_path": str(accepted_frames_path.relative_to(path)),
            "initial_overlay_path": str(initial_overlay_path.relative_to(path)),
            "tracking_thresholds": {
                "max_center_jump_px": max_center_jump_px,
                "min_area_ratio": min_area_ratio,
                "max_area_ratio": max_area_ratio,
                "max_depth_jump_m": max_depth_jump_m,
            },
            "frames": [
                {
                    "frame": decision.frame,
                    "accepted": decision.accepted,
                    "reason": decision.reason,
                    "stats": asdict(decision.stats),
                }
                for decision in decisions
            ],
        },
    )
    write_mask_metadata(path, object_prompt, backend.name, len(rgb_frames), len(accepted_frames), accepted_frames_path)
    return InteractiveTrackingResult(
        dataset_path=path,
        frames_processed=len(rgb_frames),
        frames_accepted=len(accepted_frames),
        accepted_frames_path=accepted_frames_path,
        summary_path=summary_path,
        initial_overlay_path=initial_overlay_path,
    )


def compute_track_stats(
    frame: str,
    mask: np.ndarray,
    depth: np.ndarray,
    *,
    depth_scale: float,
) -> TrackStats:
    binary = ensure_binary_mask(mask)
    ys, xs = np.nonzero(binary)
    area = int(len(xs))
    if area == 0:
        return TrackStats(frame, 0, float("nan"), float("nan"), None)
    depth_values = depth[binary > 0]
    valid_depth = depth_values[depth_values > 0]
    median_depth = float(np.median(valid_depth) * depth_scale) if len(valid_depth) else None
    return TrackStats(
        frame=frame,
        area_pixels=area,
        center_x=float(xs.mean()),
        center_y=float(ys.mean()),
        median_depth_m=median_depth,
    )


def accept_track_stats(
    stats: TrackStats,
    previous: TrackStats | None,
    *,
    max_center_jump_px: float,
    min_area_ratio: float,
    max_area_ratio: float,
    max_depth_jump_m: float,
) -> tuple[bool, str]:
    if stats.area_pixels <= 0:
        return False, "empty_mask"
    if previous is None:
        return True, "initial_accepted"

    center_jump = float(
        np.hypot(stats.center_x - previous.center_x, stats.center_y - previous.center_y)
    )
    if center_jump > max_center_jump_px:
        return False, f"center_jump_px={center_jump:.3f}"

    area_ratio = stats.area_pixels / max(1, previous.area_pixels)
    if area_ratio < min_area_ratio or area_ratio > max_area_ratio:
        return False, f"area_ratio={area_ratio:.3f}"

    if stats.median_depth_m is not None and previous.median_depth_m is not None:
        depth_jump = abs(stats.median_depth_m - previous.median_depth_m)
        if depth_jump > max_depth_jump_m:
            return False, f"depth_jump_m={depth_jump:.3f}"
    return True, "tracked"


def terminal_accept_initial(overlay_path: Path) -> bool:
    print(f"Initial mask overlay: {overlay_path}")
    answer = input("Accept this initial mask and continue tracking? [y/N]: ").strip().lower()
    return answer in {"y", "yes"}


def gui_accept_initial(overlay_path: Path) -> bool:
    image = cv2.imread(str(overlay_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read initial overlay: {overlay_path}")
    cv2.imshow("Initial mask: press y to accept, n to reject", image)
    key = cv2.waitKey(0)
    cv2.destroyAllWindows()
    return chr(key & 0xFF).lower() == "y"


def write_overlay(path: Path, rgb_path: Path, mask: np.ndarray) -> None:
    overlay = create_mask_overlay(read_rgb_image(rgb_path), mask)
    cv2.imwrite(str(path), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))


def write_mask_metadata(
    dataset_path: Path,
    prompt: str,
    backend_name: str,
    frames_processed: int,
    frames_accepted: int,
    accepted_frames_path: Path,
) -> None:
    data = {
        "backend": f"{backend_name}_interactive_tracking",
        "prompt": prompt,
        "frames_processed": frames_processed,
        "frames_accepted": frames_accepted,
        "accepted_frames": str(accepted_frames_path.relative_to(dataset_path)),
        "annotation_summary": "mask_annotations/tracking_summary.json",
    }
    (dataset_path / "masks_metadata.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Interactively accept the initial object mask and continue identity-gated mask tracking."
    )
    parser.add_argument("dataset_path", type=Path, help="Scan dataset directory.")
    parser.add_argument("--prompt", default=None, help="Text prompt. Defaults to metadata.yaml object_prompt.")
    parser.add_argument(
        "--review-mode",
        choices=["terminal", "gui", "auto"],
        default="terminal",
        help="How to accept the initial mask.",
    )
    parser.add_argument("--max-center-jump-px", type=float, default=80.0)
    parser.add_argument("--min-area-ratio", type=float, default=0.35)
    parser.add_argument("--max-area-ratio", type=float, default=2.8)
    parser.add_argument("--max-depth-jump-m", type=float, default=0.12)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--no-overlays", action="store_true")

    # Reuse the backend options from the normal mask-generation CLI.
    parser.add_argument(
        "--backend",
        choices=["grounding_dino_sam2", "box_stub", "hsv_color"],
        default="grounding_dino_sam2",
    )
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--grounding-device", default=None)
    parser.add_argument("--grounding-config", type=Path, default=None)
    parser.add_argument("--grounding-checkpoint", type=Path, default=None)
    parser.add_argument("--sam2-config", default="configs/sam2.1/sam2.1_hiera_t.yaml")
    parser.add_argument("--sam2-checkpoint", type=Path, default=None)
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--multimask-output", action="store_true")
    parser.add_argument("--hsv-lower", default="85,45,25")
    parser.add_argument("--hsv-upper", default="135,255,255")
    parser.add_argument("--hsv-min-area", type=int, default=50)
    args = parser.parse_args(argv)

    accept_initial = None
    if args.review_mode == "gui":
        accept_initial = gui_accept_initial

    result = generate_interactive_tracked_masks(
        args.dataset_path,
        backend=build_backend(args),
        prompt=args.prompt,
        accept_initial=accept_initial,
        auto_accept_initial=args.review_mode == "auto",
        max_center_jump_px=args.max_center_jump_px,
        min_area_ratio=args.min_area_ratio,
        max_area_ratio=args.max_area_ratio,
        max_depth_jump_m=args.max_depth_jump_m,
        limit=args.limit,
        save_overlays=not args.no_overlays,
    )
    print(f"Scan dataset: {result.dataset_path}")
    print(f"Frames processed: {result.frames_processed}")
    print(f"Frames accepted: {result.frames_accepted}")
    print(f"Initial overlay: {result.initial_overlay_path}")
    print(f"Accepted frames: {result.accepted_frames_path}")
    print(f"Tracking summary: {result.summary_path}")
    print("Status: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
