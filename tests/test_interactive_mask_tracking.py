from pathlib import Path

import cv2
import numpy as np
import yaml

from visual_grasp_manu.interactive_mask_tracking import (
    accept_track_stats,
    compute_track_stats,
    generate_interactive_tracked_masks,
)
from visual_grasp_manu.mask_generation import BoxStubMaskBackend


def create_scan_dataset(root: Path, *, frame_count: int = 3) -> Path:
    scan = root / "scan_001"
    for dirname in ["rgb", "depth", "camera_poses"]:
        (scan / dirname).mkdir(parents=True, exist_ok=True)

    (scan / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "dataset_id": "scan_001",
                "mode": "mesh_generation",
                "multi_view": True,
                "object_prompt": "object",
                "mask_backend": "grounding_dino_sam2",
            }
        ),
        encoding="utf-8",
    )
    (scan / "camera_intrinsics.yaml").write_text(
        yaml.safe_dump(
            {
                "width": 8,
                "height": 6,
                "fx": 100.0,
                "fy": 100.0,
                "cx": 4.0,
                "cy": 3.0,
                "depth_scale": 0.001,
            }
        ),
        encoding="utf-8",
    )
    for index in range(1, frame_count + 1):
        stem = f"{index:06d}"
        image = np.zeros((6, 8, 3), dtype=np.uint8)
        image[:, :, 1] = 120
        cv2.imwrite(str(scan / "rgb" / f"{stem}.png"), image)
        np.save(scan / "depth" / f"{stem}.npy", np.ones((6, 8), dtype=np.uint16) * 500)
        (scan / "camera_poses" / f"{stem}.txt").write_text(
            "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n",
            encoding="utf-8",
        )
    return scan


def test_accept_track_stats_rejects_large_center_jump():
    previous = compute_track_stats(
        "000001",
        square_mask(20, 20, 10, 10),
        np.ones((80, 80), dtype=np.uint16) * 500,
        depth_scale=0.001,
    )
    current = compute_track_stats(
        "000002",
        square_mask(60, 60, 10, 10),
        np.ones((80, 80), dtype=np.uint16) * 500,
        depth_scale=0.001,
    )

    accepted, reason = accept_track_stats(
        current,
        previous,
        max_center_jump_px=15.0,
        min_area_ratio=0.5,
        max_area_ratio=2.0,
        max_depth_jump_m=0.1,
    )

    assert not accepted
    assert reason.startswith("center_jump_px=")


def test_generate_interactive_tracked_masks_writes_accepted_frame_list(tmp_path: Path):
    scan = create_scan_dataset(tmp_path, frame_count=3)

    result = generate_interactive_tracked_masks(
        scan,
        backend=BoxStubMaskBackend(),
        prompt="object",
        auto_accept_initial=True,
        max_center_jump_px=100.0,
        save_overlays=True,
    )

    assert result.frames_processed == 3
    assert result.frames_accepted == 3
    assert result.accepted_frames_path.read_text(encoding="utf-8").splitlines() == [
        "000001",
        "000002",
        "000003",
    ]
    assert (scan / "interactive_review" / "000001_initial_overlay.png").is_file()
    assert (scan / "mask_annotations" / "tracking_summary.json").is_file()


def square_mask(x: int, y: int, width: int, height: int) -> np.ndarray:
    mask = np.zeros((80, 80), dtype=np.uint8)
    mask[y : y + height, x : x + width] = 255
    return mask
