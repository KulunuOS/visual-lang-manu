from pathlib import Path

import yaml

from visual_grasp_manu.scan_dataset import validate_scan_dataset


def write_yaml(path: Path, data: dict):
    path.write_text(yaml.safe_dump(data), encoding="utf-8")


def create_scan_dataset(
    root: Path,
    *,
    frame_count: int = 2,
    mode: str = "mesh_generation",
    multi_view: bool = True,
    include_masks: bool = False,
    include_mesh: bool = False,
) -> Path:
    scan = root / "scan_001"
    for dirname in ["rgb", "depth"]:
        (scan / dirname).mkdir(parents=True, exist_ok=True)

    if multi_view:
        (scan / "camera_poses").mkdir(parents=True, exist_ok=True)
    if include_masks:
        (scan / "masks").mkdir(parents=True, exist_ok=True)
    if include_mesh:
        (scan / "mesh").mkdir(parents=True, exist_ok=True)

    write_yaml(
        scan / "metadata.yaml",
        {
            "dataset_id": "scan_001",
            "mode": mode,
            "multi_view": multi_view,
            "object_prompt": "blue cube",
            "mask_backend": "grounding_dino_sam2",
        },
    )
    write_yaml(
        scan / "camera_intrinsics.yaml",
        {
            "width": 640,
            "height": 480,
            "fx": 615.0,
            "fy": 615.0,
            "cx": 320.0,
            "cy": 240.0,
            "depth_scale": 0.001,
        },
    )

    for index in range(1, frame_count + 1):
        stem = f"{index:06d}"
        (scan / "rgb" / f"{stem}.png").write_bytes(b"rgb")
        (scan / "depth" / f"{stem}.png").write_bytes(b"depth")
        if multi_view:
            (scan / "camera_poses" / f"{stem}.txt").write_text(
                "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n",
                encoding="utf-8",
            )
        if include_masks:
            (scan / "masks" / f"{stem}.png").write_bytes(b"mask")

    if include_mesh:
        (scan / "mesh" / "object.ply").write_text("ply\n", encoding="utf-8")

    return scan


def create_camera_pose_validation_stub(root: Path) -> Path:
    scan = create_scan_dataset(
        root,
        frame_count=2,
        mode="camera_pose_validation_stub",
        multi_view=False,
    )
    (scan / "gt_camera_poses").mkdir(parents=True)
    for stem in ["000001", "000002"]:
        (scan / "gt_camera_poses" / f"{stem}.txt").write_text(
            "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n",
            encoding="utf-8",
        )
    return scan


def test_validate_scan_dataset_accepts_multiview_mesh_generation_capture(tmp_path):
    scan = create_scan_dataset(tmp_path)

    report = validate_scan_dataset(scan)

    assert report.ok
    assert report.errors == ()


def test_validate_scan_dataset_requires_text_prompt(tmp_path):
    scan = create_scan_dataset(tmp_path)
    write_yaml(
        scan / "metadata.yaml",
        {
            "dataset_id": "scan_001",
            "mode": "mesh_generation",
            "multi_view": True,
            "object_prompt": "",
        },
    )

    report = validate_scan_dataset(scan)

    assert not report.ok
    assert "metadata.yaml must define non-empty object_prompt" in report.errors


def test_validate_scan_dataset_rejects_single_view_mesh_generation(tmp_path):
    scan = create_scan_dataset(tmp_path, frame_count=1, multi_view=False)

    report = validate_scan_dataset(scan)

    assert not report.ok
    assert "mesh_generation datasets must set metadata.yaml multi_view: true" in report.errors
    assert "mesh_generation datasets must contain at least two distinct RGB-D frames" in report.errors


def test_validate_scan_dataset_allows_single_view_partial_smoke_test(tmp_path):
    scan = create_scan_dataset(
        tmp_path,
        frame_count=1,
        mode="partial_geometry_smoke_test",
        multi_view=False,
    )

    report = validate_scan_dataset(scan)

    assert report.ok


def test_validate_scan_dataset_masks_stage_requires_matching_masks(tmp_path):
    scan = create_scan_dataset(tmp_path, include_masks=True)
    (scan / "masks" / "000002.png").unlink()

    report = validate_scan_dataset(scan, stage="masks")

    assert not report.ok
    assert "masks/ is missing files for rgb frame stems: 000002" in report.errors


def test_validate_scan_dataset_mesh_stage_requires_mesh_file(tmp_path):
    scan = create_scan_dataset(tmp_path, include_masks=True, include_mesh=True)
    (scan / "mesh" / "object.ply").unlink()

    report = validate_scan_dataset(scan, stage="mesh")

    assert not report.ok
    assert "mesh stage requires at least one .ply or .obj file in mesh/" in report.errors


def test_validate_scan_dataset_accepts_camera_pose_validation_stub(tmp_path):
    scan = create_camera_pose_validation_stub(tmp_path)

    report = validate_scan_dataset(scan)

    assert report.ok
