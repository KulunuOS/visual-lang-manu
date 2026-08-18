from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_manu.transforms import PoseData, compose_pose, inverse_pose, rotate_vector


@dataclass(frozen=True)
class CameraTrackStyle:
    namespace: str
    path_color: ColorRGBA
    frustum_color: ColorRGBA
    marker_id_base: int


INFERRED_STYLE = CameraTrackStyle(
    namespace="inferred_camera",
    path_color=ColorRGBA(r=0.0, g=1.0, b=0.15, a=0.95),
    frustum_color=ColorRGBA(r=0.05, g=0.8, b=1.0, a=0.95),
    marker_id_base=1000,
)
GT_STYLE = CameraTrackStyle(
    namespace="gt_camera",
    path_color=ColorRGBA(r=1.0, g=0.75, b=0.05, a=0.95),
    frustum_color=ColorRGBA(r=1.0, g=0.75, b=0.05, a=0.95),
    marker_id_base=2000,
)


class CameraPoseVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_pose_visualizer_node")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("inferred_odom_topic", "/odom")
        self.declare_parameter(
            "inferred_odom_fallback_topics",
            ["/rgbd_odometry/odom", "/rtabmap/odom"],
        )
        self.declare_parameter("inferred_pose_topic", "")
        self.declare_parameter("gt_pose_topic", "/visual_grasp_manu/gt_camera_pose")
        self.declare_parameter("inferred_path_topic", "/visual_grasp_manu/inferred_camera_path")
        self.declare_parameter("gt_path_topic", "/visual_grasp_manu/gt_camera_path")
        self.declare_parameter("marker_topic", "/visual_grasp_manu/camera_pose_markers")
        self.declare_parameter("max_path_poses", 2000)
        self.declare_parameter("frustum_scale", 0.045)
        self.declare_parameter("frustum_stride", 75)
        self.declare_parameter("path_line_width", 0.006)
        self.declare_parameter("frustum_line_width", 0.002)
        self.declare_parameter("publish_path_marker", True)
        self.declare_parameter("frustum_mode", "sampled")
        self.declare_parameter("align_inferred_to_gt", True)
        self.declare_parameter("status_log_interval_sec", 5.0)

        self.max_path_poses = int(self.get_parameter("max_path_poses").value)
        self.inferred_poses: Deque[PoseStamped] = deque(maxlen=self.max_path_poses)
        self.gt_poses: Deque[PoseStamped] = deque(maxlen=self.max_path_poses)
        self.first_raw_inferred_pose: PoseData | None = None
        self.last_inferred_source = ""

        self.inferred_path_publisher = self.create_publisher(
            Path,
            str(self.get_parameter("inferred_path_topic").value),
            10,
        )
        self.gt_path_publisher = self.create_publisher(
            Path,
            str(self.get_parameter("gt_path_topic").value),
            10,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )

        inferred_odom_topic = str(self.get_parameter("inferred_odom_topic").value).strip()
        inferred_odom_fallback_topics = [
            str(topic).strip()
            for topic in self.get_parameter("inferred_odom_fallback_topics").value
            if str(topic).strip()
        ]
        inferred_pose_topic = str(self.get_parameter("inferred_pose_topic").value).strip()
        gt_pose_topic = str(self.get_parameter("gt_pose_topic").value).strip()

        odometry_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for topic in unique_topics([inferred_odom_topic, *inferred_odom_fallback_topics]):
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, source=topic: self.inferred_odom_callback(msg, source),
                odometry_qos,
            )
            self.get_logger().info(f"Subscribed to inferred odometry topic: {topic}")
        if inferred_pose_topic:
            self.create_subscription(PoseStamped, inferred_pose_topic, self.inferred_pose_callback, 20)
        if gt_pose_topic:
            self.create_subscription(PoseStamped, gt_pose_topic, self.gt_pose_callback, 20)
        self.create_timer(
            max(1.0, float(self.get_parameter("status_log_interval_sec").value)),
            self.log_status,
        )

        self.get_logger().info(
            "Camera pose visualizer publishing inferred/GT paths and camera frustum markers"
        )

    def log_status(self) -> None:
        self.get_logger().info(
            "Camera overlay status: "
            f"inferred_poses={len(self.inferred_poses)}, "
            f"gt_poses={len(self.gt_poses)}, "
            f"last_inferred_source='{self.last_inferred_source}', "
            f"align_inferred_to_gt={bool(self.get_parameter('align_inferred_to_gt').value)}"
        )

    def inferred_odom_callback(self, msg: Odometry, source_topic: str) -> None:
        self.last_inferred_source = source_topic
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose = self.align_inferred_pose_if_requested(pose)
        self.append_and_publish(self.inferred_poses, pose, INFERRED_STYLE)

    def inferred_pose_callback(self, msg: PoseStamped) -> None:
        msg = self.align_inferred_pose_if_requested(msg)
        self.append_and_publish(self.inferred_poses, msg, INFERRED_STYLE)

    def gt_pose_callback(self, msg: PoseStamped) -> None:
        self.append_and_publish(self.gt_poses, msg, GT_STYLE)

    def align_inferred_pose_if_requested(self, pose: PoseStamped) -> PoseStamped:
        if self.first_raw_inferred_pose is None:
            self.first_raw_inferred_pose = pose_data_from_msg(pose.pose)

        if not bool(self.get_parameter("align_inferred_to_gt").value) or not self.gt_poses:
            return pose

        gt_anchor = pose_data_from_msg(self.gt_poses[0].pose)
        current_raw = pose_data_from_msg(pose.pose)
        relative = compose_pose(inverse_pose(self.first_raw_inferred_pose), current_raw)
        aligned = compose_pose(gt_anchor, relative)

        aligned_msg = PoseStamped()
        aligned_msg.header = pose.header
        aligned_msg.header.frame_id = self.gt_poses[0].header.frame_id or str(
            self.get_parameter("fixed_frame").value
        )
        aligned_msg.pose = pose_msg_from_pose_data(aligned)
        return aligned_msg

    def append_and_publish(
        self,
        poses: Deque[PoseStamped],
        pose: PoseStamped,
        style: CameraTrackStyle,
    ) -> None:
        if not pose.header.frame_id:
            pose.header.frame_id = str(self.get_parameter("fixed_frame").value)
        poses.append(pose)
        self.publish_paths()
        self.publish_markers()

    def publish_paths(self) -> None:
        if self.inferred_poses:
            self.inferred_path_publisher.publish(path_from_poses(self.inferred_poses))
        if self.gt_poses:
            self.gt_path_publisher.publish(path_from_poses(self.gt_poses))

    def publish_markers(self) -> None:
        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker())
        marker_array.markers.extend(
            track_markers(
                self.inferred_poses,
                INFERRED_STYLE,
                frustum_scale=float(self.get_parameter("frustum_scale").value),
                frustum_stride=int(self.get_parameter("frustum_stride").value),
                path_line_width=float(self.get_parameter("path_line_width").value),
                frustum_line_width=float(self.get_parameter("frustum_line_width").value),
                publish_path_marker=bool(self.get_parameter("publish_path_marker").value),
                frustum_mode=str(self.get_parameter("frustum_mode").value),
            )
        )
        marker_array.markers.extend(
            track_markers(
                self.gt_poses,
                GT_STYLE,
                frustum_scale=float(self.get_parameter("frustum_scale").value),
                frustum_stride=int(self.get_parameter("frustum_stride").value),
                path_line_width=float(self.get_parameter("path_line_width").value),
                frustum_line_width=float(self.get_parameter("frustum_line_width").value),
                publish_path_marker=bool(self.get_parameter("publish_path_marker").value),
                frustum_mode=str(self.get_parameter("frustum_mode").value),
            )
        )
        self.marker_publisher.publish(marker_array)


def path_from_poses(poses: Deque[PoseStamped]) -> Path:
    path = Path()
    path.header = poses[-1].header
    path.poses = list(poses)
    return path


def unique_topics(topics: list[str]) -> list[str]:
    unique = []
    for topic in topics:
        if topic and topic not in unique:
            unique.append(topic)
    return unique


def track_markers(
    poses: Deque[PoseStamped],
    style: CameraTrackStyle,
    *,
    frustum_scale: float,
    frustum_stride: int,
    path_line_width: float,
    frustum_line_width: float,
    publish_path_marker: bool = True,
    frustum_mode: str = "sampled",
) -> list[Marker]:
    if not poses:
        return []

    markers = []
    if publish_path_marker:
        markers.append(trajectory_marker(poses, style, path_line_width))

    mode = frustum_mode.strip().lower()
    if mode == "initial":
        markers.append(
            frustum_marker(poses[0], style, style.marker_id_base + 1, frustum_scale, frustum_line_width)
        )
        return markers

    markers.append(
        frustum_marker(poses[-1], style, style.marker_id_base + 1, frustum_scale, frustum_line_width)
    )
    if mode == "latest":
        return markers

    stride = max(1, frustum_stride)
    sampled = list(poses)[::stride]
    if poses[-1] not in sampled:
        sampled.append(poses[-1])
    for index, pose in enumerate(sampled):
        markers.append(
            frustum_marker(
                pose,
                style,
                style.marker_id_base + 100 + index,
                frustum_scale * 0.7,
                frustum_line_width,
            )
        )
    return markers


def delete_marker() -> Marker:
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def trajectory_marker(
    poses: Deque[PoseStamped],
    style: CameraTrackStyle,
    line_width: float,
) -> Marker:
    marker = Marker()
    marker.header = poses[-1].header
    marker.ns = f"{style.namespace}_trajectory"
    marker.id = style.marker_id_base
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = line_width
    marker.color = style.path_color
    marker.points = [pose_position_point(pose.pose) for pose in poses]
    return marker


def frustum_marker(
    pose: PoseStamped,
    style: CameraTrackStyle,
    marker_id: int,
    scale: float,
    line_width: float,
) -> Marker:
    marker = Marker()
    marker.header = pose.header
    marker.ns = f"{style.namespace}_frustum"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = line_width
    marker.color = style.frustum_color
    marker.points = [
        point_msg_from_tuple(point)
        for point in camera_frustum_points(pose.pose, scale=scale)
    ]
    return marker


def camera_frustum_points(pose: Pose, *, scale: float) -> list[tuple[float, float, float]]:
    near = scale
    half_width = scale * 0.55
    half_height = scale * 0.4
    origin = (0.0, 0.0, 0.0)
    corners = [
        (half_width, half_height, near),
        (-half_width, half_height, near),
        (-half_width, -half_height, near),
        (half_width, -half_height, near),
    ]
    axes = [
        ((0.0, 0.0, 0.0), (scale * 0.75, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, scale * 0.75, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, scale * 0.75)),
    ]
    local_segments = []
    for corner in corners:
        local_segments.extend([origin, corner])
    for index, corner in enumerate(corners):
        local_segments.extend([corner, corners[(index + 1) % len(corners)]])
    for start, end in axes:
        local_segments.extend([start, end])

    pose_data = pose_data_from_msg(pose)
    return [transform_local_point(pose_data, point) for point in local_segments]


def transform_local_point(
    pose: PoseData,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = rotate_vector(pose.orientation_xyzw, point)
    return tuple(pose.position[index] + rotated[index] for index in range(3))


def pose_data_from_msg(msg: Pose) -> PoseData:
    return PoseData(
        position=(msg.position.x, msg.position.y, msg.position.z),
        orientation_xyzw=(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ),
    )


def pose_msg_from_pose_data(pose: PoseData) -> Pose:
    msg = Pose()
    msg.position.x = pose.position[0]
    msg.position.y = pose.position[1]
    msg.position.z = pose.position[2]
    msg.orientation.x = pose.orientation_xyzw[0]
    msg.orientation.y = pose.orientation_xyzw[1]
    msg.orientation.z = pose.orientation_xyzw[2]
    msg.orientation.w = pose.orientation_xyzw[3]
    return msg


def pose_position_point(pose: Pose) -> Point:
    return Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)


def point_msg_from_tuple(point: tuple[float, float, float]) -> Point:
    return Point(x=point[0], y=point[1], z=point[2])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPoseVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
