from pathlib import Path

import cv2
import numpy as np
import yaml

from visual_grasp_manu.tsdf_mesh import (
    apply_mask_to_depth,
    apply_mask_to_rgb,
    collect_fusion_frames,
    keep_largest_triangle_component,
    read_pose_matrix,
    read_frame_list,
)


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
        cv2.imwrite(str(scan / "rgb" / f"{stem}.png"), np.zeros((6, 8, 3), dtype=np.uint8))
        np.save(scan / "depth" / f"{stem}.npy", np.ones((6, 8), dtype=np.uint16))
        (scan / "camera_poses" / f"{stem}.txt").write_text(
            "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n",
            encoding="utf-8",
        )
    return scan


def test_apply_mask_to_depth_zeroes_background():
    depth = np.array([[100, 200], [300, 400]], dtype=np.uint16)
    mask = np.array([[255, 0], [0, 255]], dtype=np.uint8)

    masked = apply_mask_to_depth(depth, mask)

    assert masked.tolist() == [[100, 0], [0, 400]]


def test_apply_mask_to_rgb_zeroes_background():
    rgb = np.full((2, 2, 3), 100, dtype=np.uint8)
    mask = np.array([[0, 255], [255, 0]], dtype=np.uint8)

    masked = apply_mask_to_rgb(rgb, mask)

    assert masked[0, 0].tolist() == [0, 0, 0]
    assert masked[0, 1].tolist() == [100, 100, 100]


def test_collect_fusion_frames_matches_rgb_depth_mask_and_pose(tmp_path: Path):
    scan = create_scan_dataset(tmp_path, frame_count=2)
    (scan / "masks").mkdir()
    for stem in ["000001", "000002"]:
        cv2.imwrite(str(scan / "masks" / f"{stem}.png"), np.ones((6, 8), dtype=np.uint8) * 255)

    frames = collect_fusion_frames(scan)

    assert [frame.stem for frame in frames] == ["000001", "000002"]


def test_read_pose_matrix_requires_4x4(tmp_path: Path):
    pose = tmp_path / "pose.txt"
    pose.write_text("1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n", encoding="utf-8")

    matrix = read_pose_matrix(pose)

    assert matrix.shape == (4, 4)


def test_keep_largest_triangle_component_removes_small_island():
    import open3d as o3d

    large = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
    small = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
    small.translate((2.0, 0.0, 0.0))
    combined = large + small

    cleaned = keep_largest_triangle_component(combined)

    assert len(cleaned.triangles) == len(large.triangles)


def test_read_frame_list_ignores_empty_lines_and_comments(tmp_path: Path):
    frame_list = tmp_path / "accepted_frames.txt"
    frame_list.write_text("000001\n\n# comment\n000003\n", encoding="utf-8")

    assert read_frame_list(frame_list) == {"000001", "000003"}
