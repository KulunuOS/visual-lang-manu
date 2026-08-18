from pathlib import Path

import cv2
import numpy as np
import yaml

from visual_grasp_manu.mask_generation import BoxStubMaskBackend, generate_masks
from visual_grasp_manu.mask_preview import collect_preview_frames, create_mask_contact_sheet


def create_scan_dataset(root: Path, *, frame_count: int = 2) -> Path:
    scan = root / "scan_001"
    for dirname in ["rgb", "depth", "camera_poses"]:
        (scan / dirname).mkdir(parents=True, exist_ok=True)

    (scan / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "dataset_id": "scan_001",
                "mode": "mesh_generation",
                "multi_view": True,
                "object_prompt": "blue object",
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
        np.save(scan / "depth" / f"{stem}.npy", np.ones((6, 8), dtype=np.uint16))
        (scan / "camera_poses" / f"{stem}.txt").write_text(
            "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n",
            encoding="utf-8",
        )
    return scan


def test_create_mask_contact_sheet_writes_preview(tmp_path: Path):
    scan = create_scan_dataset(tmp_path, frame_count=2)
    generate_masks(scan, backend=BoxStubMaskBackend(), save_overlays=True, overwrite=True)

    result = create_mask_contact_sheet(scan, columns=2, tile_width=32)

    assert result.frames_rendered == 2
    assert result.output_path == scan / "mask_preview" / "contact_sheet.png"
    assert result.output_path.is_file()
    assert cv2.imread(str(result.output_path)) is not None


def test_collect_preview_frames_requires_matching_masks(tmp_path: Path):
    scan = create_scan_dataset(tmp_path, frame_count=1)

    assert collect_preview_frames(scan) == []
