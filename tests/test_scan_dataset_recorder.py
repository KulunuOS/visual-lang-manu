from collections import deque
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from visual_grasp_manu.scan_dataset_recorder_node import (
    TimedMessage,
    image_to_depth_array,
    image_to_rgb_array,
    metadata_notes,
    nearest_message,
    prepare_output_dirs,
    pose_matrix_from_transform,
    write_pose_matrix_from_odometry,
)


def image_msg(*, encoding: str, width: int, height: int, data: bytes) -> Image:
    msg = Image()
    msg.encoding = encoding
    msg.width = width
    msg.height = height
    msg.data = data
    return msg


def test_nearest_message_returns_closest_stamp():
    queue = deque(
        [
            TimedMessage(100, "a"),
            TimedMessage(250, "b"),
            TimedMessage(400, "c"),
        ]
    )

    assert nearest_message(queue, 260).msg == "b"


def test_image_to_rgb_array_accepts_rgb8():
    data = bytes([1, 2, 3, 4, 5, 6])
    msg = image_msg(encoding="rgb8", width=2, height=1, data=data)

    array = image_to_rgb_array(msg)

    assert array.shape == (1, 2, 3)
    assert array[0, 1].tolist() == [4, 5, 6]


def test_image_to_rgb_array_accepts_padded_rows():
    msg = image_msg(
        encoding="rgb8",
        width=2,
        height=2,
        data=bytes(
            [
                1,
                2,
                3,
                4,
                5,
                6,
                99,
                99,
                7,
                8,
                9,
                10,
                11,
                12,
                88,
                88,
            ]
        ),
    )
    msg.step = 8

    array = image_to_rgb_array(msg)

    assert array.shape == (2, 2, 3)
    assert array[1, 0].tolist() == [7, 8, 9]
    assert array[1, 1].tolist() == [10, 11, 12]


def test_image_to_depth_array_accepts_16uc1():
    depth = np.array([[100, 200]], dtype=np.uint16)
    msg = image_msg(encoding="16UC1", width=2, height=1, data=depth.tobytes())

    array = image_to_depth_array(msg)

    assert array.dtype == np.uint16
    assert array.tolist() == [[100, 200]]


def test_image_to_depth_array_accepts_padded_rows():
    raw = np.array([100, 200, 999, 300, 400, 888], dtype=np.uint16)
    msg = image_msg(encoding="16UC1", width=2, height=2, data=raw.tobytes())
    msg.step = 6

    array = image_to_depth_array(msg)

    assert array.dtype == np.uint16
    assert array.tolist() == [[100, 200], [300, 400]]


def test_prepare_output_dirs_rejects_nonempty_directory_when_not_overwriting(tmp_path: Path):
    rgb_dir = tmp_path / "rgb"
    rgb_dir.mkdir()
    (rgb_dir / "000001.png").write_bytes(b"existing")

    try:
        prepare_output_dirs(tmp_path, overwrite=False)
    except RuntimeError as exc:
        assert "Output directory is not empty" in str(exc)
    else:
        raise AssertionError("Expected RuntimeError for non-empty output directory")


def test_write_pose_matrix_from_odometry_writes_4x4_matrix(tmp_path: Path):
    odom = Odometry()
    odom.pose.pose.position = Point(x=1.0, y=2.0, z=3.0)
    odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    path = tmp_path / "pose.txt"

    write_pose_matrix_from_odometry(path, odom)

    assert path.read_text(encoding="utf-8") == (
        "1 0 0 1\n"
        "0 1 0 2\n"
        "0 0 1 3\n"
        "0 0 0 1\n"
    )


def test_pose_matrix_from_transform_uses_tf_translation_and_rotation():
    transform = TransformStamped()
    transform.transform.translation = Vector3(x=1.0, y=2.0, z=3.0)
    transform.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    matrix = pose_matrix_from_transform(transform)

    assert matrix.tolist() == [
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def test_metadata_notes_describes_tf_pose_source():
    assert "robot TF" in metadata_notes("tf")
