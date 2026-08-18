from __future__ import annotations

import argparse
import zipfile
from dataclasses import dataclass
from pathlib import Path

from visual_grasp_manu.scan_dataset import format_report, validate_scan_dataset

BOP_MARKER_FILES = {
    "dataset_info.md",
    "scene_camera.json",
    "scene_gt.json",
    "test_targets_bop19.json",
}
ROSBAG_ARCHIVE_EXTENSIONS = {".zip"}
ROS1_BAG_EXTENSIONS = {".bag"}
ROS2_STORAGE_EXTENSIONS = {".db3", ".mcap"}


@dataclass(frozen=True)
class InputValidationReport:
    input_path: Path
    input_type: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors


def validate_input(
    input_path: Path | str,
    *,
    input_type: str,
    stage: str = "capture",
) -> InputValidationReport:
    path = Path(input_path)
    if input_type == "dataset":
        return validate_dataset_input(path, stage=stage)
    if input_type == "rosbag":
        return validate_rosbag_input(path)
    return InputValidationReport(
        path,
        input_type,
        (f"Unsupported input type '{input_type}'. Expected dataset or rosbag.",),
        (),
    )


def validate_dataset_input(path: Path, *, stage: str = "capture") -> InputValidationReport:
    if not path.exists():
        return InputValidationReport(path, "dataset", (f"Path does not exist: {path}",), ())

    if path.is_dir() and (path / "metadata.yaml").is_file():
        scan_report = validate_scan_dataset(path, stage=stage)
        return InputValidationReport(
            path,
            "dataset",
            scan_report.errors,
            scan_report.warnings,
        )

    if path.is_dir():
        return validate_bop_directory(path)

    if path.suffix.lower() == ".zip":
        return validate_bop_zip(path)

    return InputValidationReport(
        path,
        "dataset",
        (
            "Dataset input must be a scan dataset directory, a BOP-style "
            "directory, or a BOP .zip archive.",
        ),
        (),
    )


def validate_bop_directory(path: Path) -> InputValidationReport:
    marker_paths = [
        file
        for file in path.rglob("*")
        if file.is_file() and file.name in BOP_MARKER_FILES
    ]
    if marker_paths:
        return InputValidationReport(path, "dataset", (), ())
    return InputValidationReport(
        path,
        "dataset",
        (
            "BOP-style dataset directory did not contain expected marker files: "
            f"{', '.join(sorted(BOP_MARKER_FILES))}",
        ),
        (),
    )


def validate_bop_zip(path: Path) -> InputValidationReport:
    try:
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            markers = {
                Path(name).name
                for name in names
                if not name.endswith("/") and Path(name).name in BOP_MARKER_FILES
            }
    except zipfile.BadZipFile:
        return InputValidationReport(path, "dataset", ("Dataset archive is not a valid zip file.",), ())

    if markers:
        return InputValidationReport(path, "dataset", (), ())
    return InputValidationReport(
        path,
        "dataset",
        (
            "BOP-style dataset archive did not contain expected marker files: "
            f"{', '.join(sorted(BOP_MARKER_FILES))}",
        ),
        (),
    )


def validate_rosbag_input(path: Path) -> InputValidationReport:
    if not path.exists():
        return InputValidationReport(path, "rosbag", (f"Path does not exist: {path}",), ())

    if path.is_dir():
        if (path / "metadata.yaml").is_file() and any(
            child.suffix.lower() in ROS2_STORAGE_EXTENSIONS for child in path.iterdir()
        ):
            return InputValidationReport(path, "rosbag", (), ())
        return InputValidationReport(
            path,
            "rosbag",
            ("ROS 2 bag directory must contain metadata.yaml and a .db3 or .mcap storage file.",),
            (),
        )

    suffix = path.suffix.lower()
    if suffix in ROS1_BAG_EXTENSIONS or suffix in ROS2_STORAGE_EXTENSIONS:
        return InputValidationReport(path, "rosbag", (), ())
    if suffix in ROSBAG_ARCHIVE_EXTENSIONS:
        return validate_rosbag_zip(path)

    return InputValidationReport(
        path,
        "rosbag",
        ("Rosbag input must be a .bag, .db3, .mcap, ROS 2 bag directory, or .zip archive.",),
        (),
    )


def validate_rosbag_zip(path: Path) -> InputValidationReport:
    try:
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            bag_members = [
                name
                for name in names
                if not name.endswith("/")
                and Path(name).suffix.lower() in ROS1_BAG_EXTENSIONS | ROS2_STORAGE_EXTENSIONS
            ]
            has_ros2_metadata = any(Path(name).name == "metadata.yaml" for name in names)
    except zipfile.BadZipFile:
        return InputValidationReport(path, "rosbag", ("Rosbag archive is not a valid zip file.",), ())

    if bag_members:
        return InputValidationReport(path, "rosbag", (), ())
    if has_ros2_metadata:
        return InputValidationReport(
            path,
            "rosbag",
            ("Rosbag archive has metadata.yaml but no .db3 or .mcap storage file.",),
            (),
        )
    return InputValidationReport(
        path,
        "rosbag",
        ("Rosbag archive did not contain a .bag, .db3, or .mcap file.",),
        (),
    )


def format_input_report(report: InputValidationReport) -> str:
    lines = [f"Input: {report.input_path}", f"Type: {report.input_type}"]
    lines.append("Status: OK" if report.ok else "Status: INVALID")
    if report.errors:
        lines.append("Errors:")
        lines.extend(f"- {error}" for error in report.errors)
    if report.warnings:
        lines.append("Warnings:")
        lines.extend(f"- {warning}" for warning in report.warnings)
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate a dataset or rosbag input.")
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--dataset", action="store_true", help="Validate dataset input.")
    input_group.add_argument("--rosbag", action="store_true", help="Validate rosbag input.")
    parser.add_argument("input_path", type=Path, help="Input path to validate.")
    parser.add_argument(
        "--stage",
        choices=["capture", "masks", "mesh"],
        default="capture",
        help="Scan dataset validation strictness when --dataset points to a scan dataset.",
    )
    args = parser.parse_args(argv)

    input_type = "dataset" if args.dataset else "rosbag"
    report = validate_input(args.input_path, input_type=input_type, stage=args.stage)
    print(format_input_report(report))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
