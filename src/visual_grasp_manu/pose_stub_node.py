from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PoseStubNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_stub_node")
        self.declare_parameter("output_topic", "/visual_grasp_manu/object_pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("object_frame_id", "object")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("position", [0.5, 0.0, 0.35])
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])

        output_topic = self.get_parameter("output_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_pose)

    def publish_pose(self) -> None:
        position = [float(value) for value in self.get_parameter("position").value]
        orientation = [float(value) for value in self.get_parameter("orientation_xyzw").value]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseStubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
