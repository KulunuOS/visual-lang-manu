from collections import deque

from geometry_msgs.msg import PoseStamped

from visual_grasp_manu.camera_pose_visualizer_node import (
    GT_STYLE,
    INFERRED_STYLE,
    camera_frustum_points,
    path_from_poses,
    pose_msg_from_pose_data,
    track_markers,
    unique_topics,
)
from visual_grasp_manu.transforms import PoseData


def pose_stamped(x: float, y: float, z: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def test_path_from_poses_uses_latest_header_and_all_poses():
    poses = deque([pose_stamped(0.0, 0.0, 0.0), pose_stamped(1.0, 0.0, 0.0)])

    path = path_from_poses(poses)

    assert path.header.frame_id == "map"
    assert len(path.poses) == 2
    assert path.poses[-1].pose.position.x == 1.0


def test_camera_frustum_points_returns_line_list_pairs():
    pose = pose_stamped(1.0, 2.0, 3.0)

    points = camera_frustum_points(pose.pose, scale=0.1)

    assert len(points) == 22
    assert points[0] == (1.0, 2.0, 3.0)


def test_track_markers_include_trajectory_current_and_sampled_frustums():
    poses = deque(pose_stamped(float(index), 0.0, 0.0) for index in range(5))

    markers = track_markers(
        poses,
        GT_STYLE,
        frustum_scale=0.1,
        frustum_stride=2,
        path_line_width=0.01,
        frustum_line_width=0.005,
    )

    assert len(markers) == 5
    assert markers[0].type == markers[0].LINE_STRIP
    assert markers[1].type == markers[1].LINE_LIST


def test_track_markers_can_publish_only_initial_frustum():
    poses = deque(pose_stamped(float(index), 0.0, 0.0) for index in range(5))

    markers = track_markers(
        poses,
        INFERRED_STYLE,
        frustum_scale=0.1,
        frustum_stride=2,
        path_line_width=0.01,
        frustum_line_width=0.005,
        publish_path_marker=False,
        frustum_mode="initial",
    )

    assert len(markers) == 1
    assert markers[0].type == markers[0].LINE_LIST
    assert markers[0].points[0].x == 0.0


def test_inferred_track_uses_green_path_and_blue_frustums():
    poses = deque([pose_stamped(0.0, 0.0, 0.0), pose_stamped(1.0, 0.0, 0.0)])

    markers = track_markers(
        poses,
        INFERRED_STYLE,
        frustum_scale=0.1,
        frustum_stride=10,
        path_line_width=0.01,
        frustum_line_width=0.005,
    )

    assert markers[0].color.g > markers[0].color.b
    assert markers[1].color.b > markers[1].color.g


def test_pose_msg_from_pose_data_copies_position_and_orientation():
    msg = pose_msg_from_pose_data(
        PoseData(
            position=(1.0, 2.0, 3.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
    )

    assert msg.position.x == 1.0
    assert msg.position.y == 2.0
    assert msg.position.z == 3.0
    assert msg.orientation.w == 1.0


def test_unique_topics_removes_empty_values_and_duplicates():
    assert unique_topics(["/odom", "", "/odom", "/rtabmap/odom"]) == [
        "/odom",
        "/rtabmap/odom",
    ]
