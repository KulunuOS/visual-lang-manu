from __future__ import annotations

import argparse
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class Rosbag2Topic:
    topic_id: int
    name: str
    type: str
    serialization_format: str
    message_count: int
    offered_qos_profiles: str


@dataclass(frozen=True)
class Rosbag2SqliteInfo:
    db3_path: Path
    topics: tuple[Rosbag2Topic, ...]
    message_count: int
    starting_time_ns: int
    duration_ns: int


def inspect_rosbag2_sqlite(db3_path: Path | str) -> Rosbag2SqliteInfo:
    path = Path(db3_path)
    if not path.is_file():
        raise FileNotFoundError(f"ROS 2 SQLite bag does not exist: {path}")

    connection = sqlite3.connect(path)
    try:
        topic_columns = table_columns(connection, "topics")
        qos_column = "offered_qos_profiles" if "offered_qos_profiles" in topic_columns else "''"
        rows = connection.execute(
            f"""
            select topics.id,
                   topics.name,
                   topics.type,
                   topics.serialization_format,
                   {qos_column},
                   count(messages.id)
            from topics
            left join messages on topics.id = messages.topic_id
            group by topics.id
            order by topics.id
            """
        ).fetchall()
        topics = tuple(
            Rosbag2Topic(
                topic_id=int(row[0]),
                name=str(row[1]),
                type=str(row[2]),
                serialization_format=str(row[3]),
                offered_qos_profiles=str(row[4] or ""),
                message_count=int(row[5]),
            )
            for row in rows
        )

        bounds = connection.execute(
            "select count(id), min(timestamp), max(timestamp) from messages"
        ).fetchone()
    finally:
        connection.close()

    message_count = int(bounds[0] or 0)
    starting_time_ns = int(bounds[1] or 0)
    ending_time_ns = int(bounds[2] or starting_time_ns)
    return Rosbag2SqliteInfo(
        db3_path=path,
        topics=topics,
        message_count=message_count,
        starting_time_ns=starting_time_ns,
        duration_ns=max(0, ending_time_ns - starting_time_ns),
    )


def table_columns(connection: sqlite3.Connection, table_name: str) -> set[str]:
    return {str(row[1]) for row in connection.execute(f"pragma table_info({table_name})")}


def infer_realsense_topics(topics: tuple[Rosbag2Topic, ...]) -> dict[str, str]:
    names_by_type: dict[str, list[str]] = {}
    for topic in topics:
        names_by_type.setdefault(topic.type, []).append(topic.name)

    image_topics = names_by_type.get("sensor_msgs/msg/Image", [])
    camera_info_topics = names_by_type.get("sensor_msgs/msg/CameraInfo", [])
    pointcloud_topics = names_by_type.get("sensor_msgs/msg/PointCloud2", [])

    rgb_topic = first_matching(image_topics, ["/color/image_raw", "color/image"])
    depth_topic = first_matching(
        image_topics,
        ["/aligned_depth_to_color/image_raw", "aligned_depth", "/depth/image_raw"],
    )
    camera_info_topic = first_matching(
        camera_info_topics,
        ["/color/camera_info", "camera_info"],
    )
    aligned_camera_info_topic = first_matching(
        camera_info_topics,
        ["/aligned_depth_to_color/camera_info"],
    )
    pointcloud_topic = first_matching(pointcloud_topics, ["/depth/color/points", "points"])

    inferred = {
        "rgb_topic": rgb_topic,
        "depth_topic": depth_topic,
        "camera_info_topic": camera_info_topic,
        "aligned_camera_info_topic": aligned_camera_info_topic,
        "pointcloud_topic": pointcloud_topic,
    }
    return {key: value for key, value in inferred.items() if value}


def first_matching(candidates: list[str], preferred_patterns: list[str]) -> str:
    for pattern in preferred_patterns:
        for candidate in candidates:
            if pattern in candidate:
                return candidate
    return candidates[0] if candidates else ""


def write_rosbag2_metadata(
    info: Rosbag2SqliteInfo,
    output_dir: Path | str,
    *,
    symlink_db3: bool = True,
    force: bool = False,
) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    target_db3 = output_path / info.db3_path.name
    if info.db3_path.resolve() != target_db3.resolve():
        if target_db3.exists() or target_db3.is_symlink():
            if not force:
                raise FileExistsError(f"Refusing to replace existing bag file link: {target_db3}")
            target_db3.unlink()
        if symlink_db3:
            target_db3.symlink_to(info.db3_path.resolve())
        else:
            raise ValueError("Copying large ROS bag databases is intentionally not supported")

    metadata = rosbag2_metadata_mapping(info)
    metadata_path = output_path / "metadata.yaml"
    if metadata_path.exists() and not force:
        raise FileExistsError(f"Refusing to replace existing metadata: {metadata_path}")
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    return metadata_path


def rosbag2_metadata_mapping(info: Rosbag2SqliteInfo) -> dict[str, Any]:
    relative_file = info.db3_path.name
    return {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": info.duration_ns},
            "starting_time": {"nanoseconds_since_epoch": info.starting_time_ns},
            "message_count": info.message_count,
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": topic.name,
                        "type": topic.type,
                        "serialization_format": topic.serialization_format,
                        "offered_qos_profiles": topic.offered_qos_profiles,
                    },
                    "message_count": topic.message_count,
                }
                for topic in info.topics
            ],
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [relative_file],
            "files": [
                {
                    "path": relative_file,
                    "starting_time": {
                        "nanoseconds_since_epoch": info.starting_time_ns,
                    },
                    "duration": {"nanoseconds": info.duration_ns},
                    "message_count": info.message_count,
                }
            ],
        }
    }


def format_bag_info(info: Rosbag2SqliteInfo) -> str:
    lines = [
        f"ROS 2 SQLite bag: {info.db3_path}",
        f"Messages: {info.message_count}",
        f"Duration: {info.duration_ns / 1_000_000_000:.3f} s",
        "Topics:",
    ]
    for topic in info.topics:
        lines.append(f"- {topic.name} [{topic.type}] messages={topic.message_count}")

    inferred = infer_realsense_topics(info.topics)
    if inferred:
        lines.append("Suggested RealSense topic mapping:")
        lines.extend(f"- {key}: {value}" for key, value in inferred.items())
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Inspect a ROS 2 SQLite bag and optionally create a playable bag directory."
    )
    parser.add_argument("db3_path", type=Path, help="Path to a loose ROS 2 .db3 bag file.")
    parser.add_argument(
        "output_dir",
        nargs="?",
        type=Path,
        help="Optional ROS 2 bag directory to create with metadata.yaml and a symlink to the .db3.",
    )
    parser.add_argument("--force", action="store_true", help="Replace existing generated metadata/link.")
    args = parser.parse_args(argv)

    info = inspect_rosbag2_sqlite(args.db3_path)
    print(format_bag_info(info))
    if args.output_dir:
        metadata_path = write_rosbag2_metadata(info, args.output_dir, force=args.force)
        print(f"Wrote ROS 2 bag metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
