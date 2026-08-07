from __future__ import annotations

from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_manu.transforms import PoseData, compose_pose, rotate_vector


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
        self.declare_parameter("jaw_length", 0.08)
        self.declare_parameter("jaw_thickness", 0.008)
        self.declare_parameter("axis_length", 0.06)

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
                    label=str(grasp.get("label", f"grasp_{index}")),
                )
            )
            next_marker_id += 4

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
        label: str,
    ) -> list[Marker]:
        color = score_color(score)
        left_jaw = self.jaw_marker(frame_id, stamp, marker_id, pose, 1.0, color)
        right_jaw = self.jaw_marker(frame_id, stamp, marker_id + 1, pose, -1.0, color)
        approach = self.approach_marker(frame_id, stamp, marker_id + 2, pose, color)
        text = self.text_marker(frame_id, stamp, marker_id + 3, pose, label, score)
        return [left_jaw, right_jaw, approach, text]

    def jaw_marker(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        grasp_pose: PoseData,
        side: float,
        color: ColorRGBA,
    ) -> Marker:
        gripper_width = float(self.get_parameter("gripper_width").value)
        local_offset = (0.0, side * gripper_width / 2.0, 0.0)
        world_offset = rotate_vector(grasp_pose.orientation_xyzw, local_offset)
        jaw_pose = PoseData(
            position=tuple(grasp_pose.position[index] + world_offset[index] for index in range(3)),
            orientation_xyzw=grasp_pose.orientation_xyzw,
        )

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "visual_grasp_manu_gripper"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_msg_from_data(jaw_pose)
        marker.scale = Vector3(
            x=float(self.get_parameter("jaw_length").value),
            y=float(self.get_parameter("jaw_thickness").value),
            z=float(self.get_parameter("jaw_thickness").value),
        )
        marker.color = color
        return marker

    def approach_marker(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        grasp_pose: PoseData,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "visual_grasp_manu_approach"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_msg_from_data(grasp_pose)
        marker.scale = Vector3(
            x=float(self.get_parameter("axis_length").value),
            y=0.01,
            z=0.01,
        )
        marker.color = color
        return marker

    def text_marker(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        grasp_pose: PoseData,
        label: str,
        score: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "visual_grasp_manu_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose_msg_from_data(
            PoseData(
                position=(grasp_pose.position[0], grasp_pose.position[1], grasp_pose.position[2] + 0.06),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        )
        marker.scale.z = 0.025
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = f"{label}: {score:.2f}"
        return marker


def pose_data_from_msg(msg: Pose) -> PoseData:
    return PoseData(
        position=(msg.position.x, msg.position.y, msg.position.z),
        orientation_xyzw=(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    )


def pose_msg_from_data(pose: PoseData) -> Pose:
    msg = Pose()
    msg.position.x = pose.position[0]
    msg.position.y = pose.position[1]
    msg.position.z = pose.position[2]
    msg.orientation.x = pose.orientation_xyzw[0]
    msg.orientation.y = pose.orientation_xyzw[1]
    msg.orientation.z = pose.orientation_xyzw[2]
    msg.orientation.w = pose.orientation_xyzw[3]
    return msg


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
