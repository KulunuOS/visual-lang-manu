from __future__ import annotations

from dataclasses import dataclass
from math import sqrt


@dataclass(frozen=True)
class PoseData:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


def normalize_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = sqrt(sum(component * component for component in q))
    if norm == 0.0:
        raise ValueError("Quaternion norm must be non-zero")
    return tuple(component / norm for component in q)


def quaternion_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return normalize_quaternion(raw_quaternion_multiply(q1, q2))


def raw_quaternion_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vector(
    q: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    qn = normalize_quaternion(q)
    vx, vy, vz = vector
    rotated = raw_quaternion_multiply(
        raw_quaternion_multiply(qn, (vx, vy, vz, 0.0)),
        quaternion_conjugate(qn),
    )
    return rotated[:3]


def compose_pose(parent: PoseData, child: PoseData) -> PoseData:
    parent_q = normalize_quaternion(parent.orientation_xyzw)
    child_position_parent = rotate_vector(parent_q, child.position)
    position = tuple(
        parent.position[index] + child_position_parent[index]
        for index in range(3)
    )
    orientation = quaternion_multiply(parent_q, normalize_quaternion(child.orientation_xyzw))
    return PoseData(position=position, orientation_xyzw=orientation)


def inverse_pose(pose: PoseData) -> PoseData:
    orientation_inverse = quaternion_conjugate(normalize_quaternion(pose.orientation_xyzw))
    inverse_position = rotate_vector(
        orientation_inverse,
        tuple(-component for component in pose.position),
    )
    return PoseData(position=inverse_position, orientation_xyzw=orientation_inverse)
