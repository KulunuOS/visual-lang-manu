import sqlite3

import yaml

from visual_grasp_manu.rosbag2_dataset import (
    infer_realsense_topics,
    inspect_rosbag2_sqlite,
    write_rosbag2_metadata,
)


def create_bag_db(path):
    connection = sqlite3.connect(path)
    try:
        connection.execute(
            "create table topics ("
            "id integer primary key, "
            "name text not null, "
            "type text not null, "
            "serialization_format text not null, "
            "offered_qos_profiles text not null)"
        )
        connection.execute(
            "create table messages ("
            "id integer primary key, "
            "topic_id integer not null, "
            "timestamp integer not null, "
            "data blob not null)"
        )
        topics = [
            (1, "/camera/ee_cam/color/image_raw", "sensor_msgs/msg/Image", "cdr", ""),
            (
                2,
                "/camera/ee_cam/aligned_depth_to_color/image_raw",
                "sensor_msgs/msg/Image",
                "cdr",
                "",
            ),
            (3, "/camera/ee_cam/color/camera_info", "sensor_msgs/msg/CameraInfo", "cdr", ""),
        ]
        connection.executemany("insert into topics values (?, ?, ?, ?, ?)", topics)
        messages = [
            (1, 1, 100, b"rgb"),
            (2, 2, 120, b"depth"),
            (3, 3, 130, b"info"),
            (4, 1, 200, b"rgb"),
        ]
        connection.executemany("insert into messages values (?, ?, ?, ?)", messages)
        connection.commit()
    finally:
        connection.close()


def test_inspect_rosbag2_sqlite_reports_topics_and_counts(tmp_path):
    db3_path = tmp_path / "scan_0.db3"
    create_bag_db(db3_path)

    info = inspect_rosbag2_sqlite(db3_path)

    assert info.message_count == 4
    assert info.duration_ns == 100
    assert info.topics[0].message_count == 2


def test_infer_realsense_topics_uses_recorded_topic_layout(tmp_path):
    db3_path = tmp_path / "scan_0.db3"
    create_bag_db(db3_path)
    info = inspect_rosbag2_sqlite(db3_path)

    inferred = infer_realsense_topics(info.topics)

    assert inferred["rgb_topic"] == "/camera/ee_cam/color/image_raw"
    assert inferred["depth_topic"] == "/camera/ee_cam/aligned_depth_to_color/image_raw"
    assert inferred["camera_info_topic"] == "/camera/ee_cam/color/camera_info"


def test_write_rosbag2_metadata_creates_playable_bag_directory(tmp_path):
    db3_path = tmp_path / "scan_0.db3"
    output_dir = tmp_path / "scan"
    create_bag_db(db3_path)
    info = inspect_rosbag2_sqlite(db3_path)

    metadata_path = write_rosbag2_metadata(info, output_dir)

    assert (output_dir / "scan_0.db3").is_symlink()
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    bag_info = metadata["rosbag2_bagfile_information"]
    assert bag_info["storage_identifier"] == "sqlite3"
    assert bag_info["relative_file_paths"] == ["scan_0.db3"]
    assert bag_info["message_count"] == 4
