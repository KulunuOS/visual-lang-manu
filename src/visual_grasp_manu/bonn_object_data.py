from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class HeaderData:
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str


@dataclass(frozen=True)
class ImageData:
    header: HeaderData
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: bytes


@dataclass(frozen=True)
class PoseStampedData:
    header: HeaderData
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class BonnObjectData:
    header: HeaderData
    rgb: ImageData
    depth: ImageData
    constant: float
    reference_board_pose: PoseStampedData
    reference_camera_pose: PoseStampedData


def parse_object_data(payload: bytes) -> BonnObjectData:
    offset = 0
    header, offset = parse_header(payload, offset)
    rgb, offset = parse_image(payload, offset)
    depth, offset = parse_image(payload, offset)
    constant, offset = read_float32(payload, offset)
    board_pose, offset = parse_pose_stamped(payload, offset)
    camera_pose, offset = parse_pose_stamped(payload, offset)
    if offset != len(payload):
        raise ValueError(f"Unexpected trailing ObjectData bytes: {len(payload) - offset}")
    return BonnObjectData(
        header=header,
        rgb=rgb,
        depth=depth,
        constant=constant,
        reference_board_pose=board_pose,
        reference_camera_pose=camera_pose,
    )


def parse_header(payload: bytes, offset: int) -> tuple[HeaderData, int]:
    _, offset = read_uint32(payload, offset)
    stamp_sec, offset = read_uint32(payload, offset)
    stamp_nanosec, offset = read_uint32(payload, offset)
    frame_id, offset = read_string(payload, offset)
    return HeaderData(stamp_sec, stamp_nanosec, frame_id), offset


def parse_image(payload: bytes, offset: int) -> tuple[ImageData, int]:
    header, offset = parse_header(payload, offset)
    height, offset = read_uint32(payload, offset)
    width, offset = read_uint32(payload, offset)
    encoding, offset = read_string(payload, offset)
    is_bigendian, offset = read_uint8(payload, offset)
    step, offset = read_uint32(payload, offset)
    data_length, offset = read_uint32(payload, offset)
    data = payload[offset:offset + data_length]
    if len(data) != data_length:
        raise ValueError("Image payload ended before declared data length")
    return ImageData(header, height, width, encoding, is_bigendian, step, data), offset + data_length


def parse_pose_stamped(payload: bytes, offset: int) -> tuple[PoseStampedData, int]:
    header, offset = parse_header(payload, offset)
    values = []
    for _ in range(7):
        value, offset = read_float64(payload, offset)
        values.append(value)
    return (
        PoseStampedData(
            header=header,
            position=(values[0], values[1], values[2]),
            orientation_xyzw=(values[3], values[4], values[5], values[6]),
        ),
        offset,
    )


def read_uint8(payload: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<B", payload, offset)[0], offset + 1


def read_uint32(payload: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<I", payload, offset)[0], offset + 4


def read_float32(payload: bytes, offset: int) -> tuple[float, int]:
    return struct.unpack_from("<f", payload, offset)[0], offset + 4


def read_float64(payload: bytes, offset: int) -> tuple[float, int]:
    return struct.unpack_from("<d", payload, offset)[0], offset + 8


def read_string(payload: bytes, offset: int) -> tuple[str, int]:
    length, offset = read_uint32(payload, offset)
    data = payload[offset:offset + length]
    if len(data) != length:
        raise ValueError("String payload ended before declared length")
    return data.decode("utf-8"), offset + length


def write_pose_matrix(path: Path, pose: PoseStampedData) -> None:
    matrix = pose_matrix(pose.position, pose.orientation_xyzw)
    lines = [" ".join(f"{value:.9g}" for value in row) for row in matrix]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def pose_matrix(
    position: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
) -> list[list[float]]:
    x, y, z, w = orientation_xyzw
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm == 0.0:
        raise ValueError("Quaternion norm must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w), position[0]],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w), position[1]],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y), position[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
