from __future__ import annotations

from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_manu.transforms import PoseData, compose_pose, rotate_vector

Point3 = tuple[float, float, float]


class GraspCandidateNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_candidate_node")
        self.declare_parameter("object_pose_topic", "/visual_grasp_manu/object_pose")
        self.declare_parameter("marker_topic", "/visual_grasp_manu/grasp_markers")
        self.declare_parameter("grasp_library_path", "")
        self.declare_parameter("score_threshold", 0.0)
        self.declare_parameter("max_candidates", 20)
        self.declare_parameter("object_scale", [0.08, 0.08, 0.08])
        self.declare_parameter("gripper_width", 0.08)
        self.declare_parameter("gripper_depth", 0.08)
        self.declare_parameter("gripper_approach_length", 0.06)
        self.declare_parameter("wireframe_thickness", 0.0025)

        object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            object_pose_topic,
            self.object_pose_callback,
            10,
        )
        self.grasps = self.load_grasps()
        self.get_logger().info(f"Loaded {len(self.grasps)} grasp candidates")

    def load_grasps(self) -> list[dict[str, Any]]:
        path = Path(str(self.get_parameter("grasp_library_path").value))
        if not path.is_file():
            self.get_logger().warn(f"No grasp library found at '{path}'")
            return []

        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        score_threshold = float(self.get_parameter("score_threshold").value)
        max_candidates = int(self.get_parameter("max_candidates").value)
        grasps = [
            grasp
            for grasp in data.get("grasps", [])
            if float(grasp.get("score", 0.0)) >= score_threshold
        ]
        return sorted(grasps, key=lambda grasp: float(grasp.get("score", 0.0)), reverse=True)[
            :max_candidates
        ]

    def object_pose_callback(self, msg: PoseStamped) -> None:
        marker_array = MarkerArray()
        marker_array.markers.append(self.delete_previous_markers(msg.header.frame_id, msg.header.stamp))
        marker_array.markers.append(self.object_marker(msg))

        object_pose = pose_data_from_msg(msg.pose)
        next_marker_id = 2
        for index, grasp in enumerate(self.grasps):
            candidate_pose = compose_pose(object_pose, pose_data_from_grasp(grasp))
            marker_array.markers.extend(
                self.grasp_markers(
                    frame_id=msg.header.frame_id,
                    stamp=msg.header.stamp,
                    marker_id=next_marker_id,
                    pose=candidate_pose,
                    score=float(grasp.get("score", 0.0)),
                )
            )
            next_marker_id += 1

        self.marker_publisher.publish(marker_array)

    def delete_previous_markers(self, frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "visual_grasp_manu"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def object_marker(self, pose_msg: PoseStamped) -> Marker:
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = "visual_grasp_manu_object"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale = vector3_from_list(self.get_parameter("object_scale").value)
        marker.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=0.45)
        return marker

    def grasp_markers(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        pose: PoseData,
        score: float,
    ) -> list[Marker]:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "visual_grasp_manu_contact_graspnet_gripper"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(
            x=float(self.get_parameter("wireframe_thickness").value),
            y=0.0,
            z=0.0,
        )
        marker.color = score_color(score)
        marker.points = [
            point_msg_from_tuple(point)
            for point in contact_graspnet_gripper_polyline(
                grasp_pose=pose,
                gripper_width=float(self.get_parameter("gripper_width").value),
                gripper_depth=float(self.get_parameter("gripper_depth").value),
                approach_length=float(self.get_parameter("gripper_approach_length").value),
            )
        ]
        return [marker]


def contact_graspnet_gripper_polyline(
    grasp_pose: PoseData,
    gripper_width: float,
    gripper_depth: float,
    approach_length: float,
) -> list[Point3]:
    half_width = gripper_width / 2.0
    local_points = [
        (-approach_length, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, half_width, 0.0),
        (gripper_depth, half_width, 0.0),
        (0.0, half_width, 0.0),
        (0.0, -half_width, 0.0),
        (gripper_depth, -half_width, 0.0),
    ]
    return [transform_local_point(grasp_pose, point) for point in local_points]


def transform_local_point(pose: PoseData, point: Point3) -> Point3:
    rotated = rotate_vector(pose.orientation_xyzw, point)
    return tuple(
        pose.position[index] + rotated[index]
        for index in range(3)
    )


def point_msg_from_tuple(point: Point3) -> Point:
    return Point(x=point[0], y=point[1], z=point[2])


def pose_data_from_msg(msg: Pose) -> PoseData:
    return PoseData(
        position=(msg.position.x, msg.position.y, msg.position.z),
        orientation_xyzw=(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    )


def pose_data_from_grasp(grasp: dict[str, Any]) -> PoseData:
    pose = grasp["pose"]
    return PoseData(
        position=tuple(float(value) for value in pose["position"]),
        orientation_xyzw=tuple(float(value) for value in pose["orientation_xyzw"]),
    )


def vector3_from_list(values: Any) -> Vector3:
    values = [float(value) for value in values]
    return Vector3(x=values[0], y=values[1], z=values[2])


def score_color(score: float) -> ColorRGBA:
    clamped = max(0.0, min(1.0, score))
    return ColorRGBA(r=1.0 - clamped, g=clamped, b=0.05, a=0.85)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspCandidateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
