import numpy as np

from visual_grasp_manu.bonn_scan_replay_node import (
    BONN_MOCAP_FROM_OPTICAL,
    bonn_optical_pose,
    depth_points_in_world,
)
from visual_grasp_manu.transforms import PoseData


def test_bonn_optical_pose_applies_documented_optical_calibration():
    reference_pose = PoseData(
        position=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    pose = bonn_optical_pose(
        reference_pose,
        invert_reference=False,
        invert_optical_calibration=False,
    )

    assert pose.position == (
        1.0 + BONN_MOCAP_FROM_OPTICAL.position[0],
        2.0 + BONN_MOCAP_FROM_OPTICAL.position[1],
        3.0 + BONN_MOCAP_FROM_OPTICAL.position[2],
    )


def test_depth_points_in_world_projects_depth_with_camera_pose():
    depth = np.array([[1.0]], dtype=np.float32)
    pose = PoseData(position=(1.0, 2.0, 3.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))

    points = depth_points_in_world(
        depth,
        pose,
        intrinsics={
            "fx": 1.0,
            "fy": 1.0,
            "cx": 0.0,
            "cy": 0.0,
        },
        stride=1,
    )

    assert points == [(1.0, 2.0, 4.0)]
