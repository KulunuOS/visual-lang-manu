from pathlib import Path

import cv2
import numpy as np
import yaml

from visual_grasp_manu.mask_generation import (
    BoxStubMaskBackend,
    HsvColorMaskBackend,
    generate_masks,
    parse_hsv_triplet,
    normalized_cxcywh_to_xyxy,
)
from visual_grasp_manu.scan_dataset import validate_scan_dataset


def write_yaml(path: Path, data: dict):
    path.write_text(yaml.safe_dump(data), encoding="utf-8")


def create_scan_dataset(root: Path, *, frame_count: int = 2) -> Path:
    scan = root / "scan_001"
    for dirname in ["rgb", "depth", "camera_poses"]:
        (scan / dirname).mkdir(parents=True, exist_ok=True)

    write_yaml(
        scan / "metadata.yaml",
        {
            "dataset_id": "scan_001",
            "mode": "mesh_generation",
            "multi_view": True,
            "object_prompt": "blue cube",
            "mask_backend": "grounding_dino_sam2",
        },
    )
    write_yaml(
        scan / "camera_intrinsics.yaml",
        {
            "width": 8,
            "height": 6,
            "fx": 100.0,
            "fy": 100.0,
            "cx": 4.0,
            "cy": 3.0,
            "depth_scale": 0.001,
        },
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


def test_normalized_cxcywh_to_xyxy_scales_and_clamps():
    box = np.array([0.5, 0.5, 0.5, 0.25], dtype=np.float32)

    assert normalized_cxcywh_to_xyxy(box, 640, 480) == (160.0, 180.0, 480.0, 300.0)


def test_generate_masks_with_box_stub_writes_masks_and_annotations(tmp_path: Path):
    scan = create_scan_dataset(tmp_path)

    result = generate_masks(
        scan,
        backend=BoxStubMaskBackend(),
        save_overlays=True,
        overwrite=True,
    )

    assert result.frames_processed == 2
    assert result.masks_written == 2
    assert (scan / "masks" / "000001.png").is_file()
    assert (scan / "mask_annotations" / "000001.json").is_file()
    assert (scan / "mask_annotations" / "summary.json").is_file()
    assert (scan / "mask_overlays" / "000001.png").is_file()
    assert (scan / "masks_metadata.yaml").is_file()
    assert validate_scan_dataset(scan, stage="masks").ok


def test_generate_masks_uses_prompt_override(tmp_path: Path):
    scan = create_scan_dataset(tmp_path, frame_count=1)
    write_yaml(
        scan / "metadata.yaml",
        {
            "dataset_id": "scan_001",
            "mode": "partial_geometry_smoke_test",
            "multi_view": False,
            "object_prompt": "old prompt",
            "mask_backend": "grounding_dino_sam2",
        },
    )

    result = generate_masks(
        scan,
        backend=BoxStubMaskBackend(),
        prompt="new prompt",
        overwrite=True,
    )

    summary = (result.annotations_path).read_text(encoding="utf-8")
    assert '"prompt": "new prompt"' in summary


def test_hsv_color_backend_masks_blue_region(tmp_path: Path):
    image_path = tmp_path / "image.png"
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[:, :] = (20, 20, 20)
    image[5:15, 8:20] = (255, 0, 0)
    cv2.imwrite(str(image_path), image)

    prediction = HsvColorMaskBackend(min_area=10).predict(image_path, "blue object")

    assert prediction.box_xyxy == (8.0, 5.0, 19.0, 14.0)
    assert np.count_nonzero(prediction.mask) > 0


def test_parse_hsv_triplet():
    assert parse_hsv_triplet("85,45,25") == (85, 45, 25)
