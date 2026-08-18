from pathlib import Path
from zipfile import ZipFile

from visual_grasp_manu.input_validation import validate_input


def write_zip(path: Path, members: dict[str, bytes]) -> None:
    with ZipFile(path, "w") as archive:
        for name, data in members.items():
            archive.writestr(name, data)


def test_validate_dataset_accepts_bop_base_archive(tmp_path):
    archive_path = tmp_path / "ycbv_base.zip"
    write_zip(
        archive_path,
        {
            "ycbv/dataset_info.md": b"YCB-V",
            "ycbv/test_targets_bop19.json": b"[]",
        },
    )

    report = validate_input(archive_path, input_type="dataset")

    assert report.ok
    assert report.errors == ()


def test_validate_dataset_rejects_non_bop_archive(tmp_path):
    archive_path = tmp_path / "notes.zip"
    write_zip(archive_path, {"README.txt": b"not a BOP dataset"})

    report = validate_input(archive_path, input_type="dataset")

    assert not report.ok
    assert "BOP-style dataset archive did not contain expected marker files" in report.errors[0]


def test_validate_rosbag_accepts_zip_with_ros1_bag(tmp_path):
    archive_path = tmp_path / "objecttracking_box_bag.zip"
    write_zip(archive_path, {"objecttracking_box.bag": b"bag bytes"})

    report = validate_input(archive_path, input_type="rosbag")

    assert report.ok
    assert report.errors == ()


def test_validate_rosbag_accepts_ros2_bag_directory(tmp_path):
    bag_dir = tmp_path / "rosbag2_001"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    (bag_dir / "rosbag2_001_0.db3").write_bytes(b"sqlite")

    report = validate_input(bag_dir, input_type="rosbag")

    assert report.ok
    assert report.errors == ()


def test_validate_rosbag_rejects_zip_without_bag_storage(tmp_path):
    archive_path = tmp_path / "archive.zip"
    write_zip(archive_path, {"metadata.yaml": b"rosbag2_bagfile_information: {}\n"})

    report = validate_input(archive_path, input_type="rosbag")

    assert not report.ok
    assert "metadata.yaml but no .db3 or .mcap" in report.errors[0]
