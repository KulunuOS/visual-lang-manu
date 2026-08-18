from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

RGB_EXTENSIONS = {".jpg", ".jpeg", ".png"}
DEPTH_EXTENSIONS = {".npy", ".png", ".tif", ".tiff"}
MASK_EXTENSIONS = {".npy", ".png"}
POSE_EXTENSIONS = {".json", ".txt", ".yaml", ".yml"}

MESH_GENERATION_MODE = "mesh_generation"
PARTIAL_GEOMETRY_SMOKE_TEST_MODE = "partial_geometry_smoke_test"
CAMERA_POSE_VALIDATION_STUB_MODE = "camera_pose_validation_stub"
VALID_MODES = {
    MESH_GENERATION_MODE,
    PARTIAL_GEOMETRY_SMOKE_TEST_MODE,
    CAMERA_POSE_VALIDATION_STUB_MODE,
}


@dataclass(frozen=True)
class ScanDatasetReport:
    dataset_path: Path
    stage: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors


def validate_scan_dataset(
    dataset_path: Path | str,
    *,
    stage: str = "capture",
) -> ScanDatasetReport:
    path = Path(dataset_path)
    errors: list[str] = []
    warnings: list[str] = []

    if stage not in {"capture", "masks", "mesh"}:
        errors.append(f"Unsupported stage '{stage}'. Expected one of: capture, masks, mesh")
        return ScanDatasetReport(path, stage, tuple(errors), tuple(warnings))

    if not path.is_dir():
        errors.append(f"Dataset path does not exist or is not a directory: {path}")
        return ScanDatasetReport(path, stage, tuple(errors), tuple(warnings))

    metadata = read_yaml_file(path / "metadata.yaml", errors, required=True)
    intrinsics = read_yaml_file(path / "camera_intrinsics.yaml", errors, required=True)

    mode = str(metadata.get("mode", "")).strip() if isinstance(metadata, dict) else ""
    if mode not in VALID_MODES:
        errors.append(
            "metadata.yaml must define mode as one of: "
            f"{', '.join(sorted(VALID_MODES))}"
        )

    object_prompt = str(metadata.get("object_prompt", "")).strip() if isinstance(metadata, dict) else ""
    if not object_prompt:
        errors.append("metadata.yaml must define non-empty object_prompt")

    validate_intrinsics(intrinsics, errors)
    rgb_frames = collect_stemmed_files(path / "rgb", RGB_EXTENSIONS, errors, "rgb")
    depth_frames = collect_stemmed_files(path / "depth", DEPTH_EXTENSIONS, errors, "depth")
    compare_stems(rgb_frames, depth_frames, errors, "rgb", "depth")

    frame_count = len(rgb_frames)
    multi_view = bool(metadata.get("multi_view", False)) if isinstance(metadata, dict) else False
    requires_mesh = mode == MESH_GENERATION_MODE or stage == "mesh"

    if requires_mesh:
        if not multi_view:
            errors.append("mesh_generation datasets must set metadata.yaml multi_view: true")
        if frame_count < 2:
            errors.append(
                "mesh_generation datasets must contain at least two distinct RGB-D frames"
            )

    if mode == PARTIAL_GEOMETRY_SMOKE_TEST_MODE and frame_count < 1:
        errors.append("partial_geometry_smoke_test datasets must contain at least one RGB-D frame")

    if mode == CAMERA_POSE_VALIDATION_STUB_MODE:
        gt_camera_poses = collect_stemmed_files(
            path / "gt_camera_poses", POSE_EXTENSIONS, errors, "gt_camera_poses"
        )
        compare_stems(rgb_frames, gt_camera_poses, errors, "rgb", "gt_camera_poses")
    elif multi_view:
        camera_poses = collect_stemmed_files(
            path / "camera_poses", POSE_EXTENSIONS, errors, "camera_poses"
        )
        compare_stems(rgb_frames, camera_poses, errors, "rgb", "camera_poses")
    elif requires_mesh:
        errors.append("mesh_generation datasets require camera_poses for TSDF fusion")

    if stage in {"masks", "mesh"}:
        masks = collect_stemmed_files(path / "masks", MASK_EXTENSIONS, errors, "masks")
        compare_stems(rgb_frames, masks, errors, "rgb", "masks")

    if stage == "mesh":
        mesh_dir = path / "mesh"
        if not mesh_dir.is_dir():
            errors.append("mesh stage requires mesh/ directory")
        elif not any(mesh_dir.glob("*.ply")) and not any(mesh_dir.glob("*.obj")):
            errors.append("mesh stage requires at least one .ply or .obj file in mesh/")

    if frame_count == 1 and mode != PARTIAL_GEOMETRY_SMOKE_TEST_MODE:
        warnings.append(
            "Dataset has one RGB-D frame. This is valid only for capture checks, "
            "masking, or partial-geometry smoke tests."
        )

    return ScanDatasetReport(path, stage, tuple(errors), tuple(warnings))


def read_yaml_file(path: Path, errors: list[str], *, required: bool) -> dict[str, Any]:
    if not path.is_file():
        if required:
            errors.append(f"Missing required file: {path.name}")
        return {}

    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        errors.append(f"Invalid YAML in {path.name}: {exc}")
        return {}

    if not isinstance(data, dict):
        errors.append(f"{path.name} must contain a YAML mapping")
        return {}
    return data


def validate_intrinsics(data: dict[str, Any], errors: list[str]) -> None:
    required_numeric_fields = ["width", "height", "fx", "fy", "cx", "cy", "depth_scale"]
    for field in required_numeric_fields:
        if field not in data:
            errors.append(f"camera_intrinsics.yaml missing required field: {field}")
            continue
        if not isinstance(data[field], (int, float)):
            errors.append(f"camera_intrinsics.yaml field '{field}' must be numeric")

    if isinstance(data.get("depth_scale"), (int, float)) and float(data["depth_scale"]) <= 0.0:
        errors.append("camera_intrinsics.yaml depth_scale must be positive")


def collect_stemmed_files(
    directory: Path,
    extensions: set[str],
    errors: list[str],
    label: str,
) -> dict[str, Path]:
    if not directory.is_dir():
        errors.append(f"Missing required directory: {label}/")
        return {}

    files = {
        file.stem: file
        for file in sorted(directory.iterdir())
        if file.is_file() and file.suffix.lower() in extensions
    }
    if not files:
        errors.append(f"No supported files found in {label}/")
    return files


def compare_stems(
    reference: dict[str, Path],
    candidate: dict[str, Path],
    errors: list[str],
    reference_label: str,
    candidate_label: str,
) -> None:
    if not reference or not candidate:
        return

    reference_stems = set(reference)
    candidate_stems = set(candidate)
    missing = sorted(reference_stems - candidate_stems)
    extra = sorted(candidate_stems - reference_stems)

    if missing:
        errors.append(
            f"{candidate_label}/ is missing files for {reference_label} frame stems: "
            f"{', '.join(missing)}"
        )
    if extra:
        errors.append(
            f"{candidate_label}/ has files without matching {reference_label} stems: "
            f"{', '.join(extra)}"
        )


def format_report(report: ScanDatasetReport) -> str:
    lines = [f"Scan dataset: {report.dataset_path}", f"Stage: {report.stage}"]
    if report.ok:
        lines.append("Status: OK")
    else:
        lines.append("Status: INVALID")
        lines.append("Errors:")
        lines.extend(f"- {error}" for error in report.errors)

    if report.warnings:
        lines.append("Warnings:")
        lines.extend(f"- {warning}" for warning in report.warnings)
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate a visual_grasp_manu scan dataset.")
    parser.add_argument("dataset_path", type=Path, help="Path to the scan dataset directory.")
    parser.add_argument(
        "--stage",
        choices=["capture", "masks", "mesh"],
        default="capture",
        help="Validation strictness for the current pipeline stage.",
    )
    args = parser.parse_args(argv)

    report = validate_scan_dataset(args.dataset_path, stage=args.stage)
    print(format_report(report))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
