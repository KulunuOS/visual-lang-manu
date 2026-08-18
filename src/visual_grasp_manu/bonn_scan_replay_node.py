from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from visual_grasp_manu.transforms import PoseData, compose_pose, inverse_pose, rotate_vector

BONN_MOCAP_FROM_OPTICAL = PoseData(
    position=(-0.01303, -0.14200, -0.04437),
    orientation_xyzw=(-0.71059, 0.14478, -0.67013, 0.15821),
)


class BonnScanReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("bonn_scan_replay_node")
        self.declare_parameter("scan_path", "outputs/datasets/bonn_box_slow_scan")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("gt_pose_topic", "/visual_grasp_manu/gt_camera_pose")
        self.declare_parameter("pointcloud_topic", "/visual_grasp_manu/bonn/points_gt_map")
        self.declare_parameter("camera_frame_id", "openni_rgb_optical_frame")
        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("pointcloud_stride", 8)
        self.declare_parameter("accumulate_pointcloud", True)
        self.declare_parameter("max_accumulated_points", 400000)
        self.declare_parameter("invert_reference_camera_pose", False)
        self.declare_parameter("invert_optical_calibration", False)

        self.scan_path = Path(str(self.get_parameter("scan_path").value))
        self.intrinsics = load_yaml(self.scan_path / "camera_intrinsics.yaml")
        self.frames = collect_frames(self.scan_path)
        if not self.frames:
            raise RuntimeError(f"No replay frames found in {self.scan_path}")

        self.rgb_publisher = self.create_publisher(Image, str(self.get_parameter("rgb_topic").value), 10)
        self.depth_publisher = self.create_publisher(Image, str(self.get_parameter("depth_topic").value), 10)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            10,
        )
        self.gt_pose_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("gt_pose_topic").value),
            10,
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            10,
        )

        self.frame_index = 0
        self.accumulated_points: list[tuple[float, float, float]] = []
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.publish_next_frame)
        self.get_logger().info(f"Replaying {len(self.frames)} Bonn frames from {self.scan_path}")

    def publish_next_frame(self) -> None:
        if self.frame_index >= len(self.frames):
            if bool(self.get_parameter("loop").value):
                self.frame_index = 0
                self.accumulated_points.clear()
            else:
                return

        frame = self.frames[self.frame_index]
        stamp = self.get_clock().now().to_msg()
        camera_frame = str(self.get_parameter("camera_frame_id").value)
        world_frame = str(self.get_parameter("world_frame_id").value)
        header = Header(stamp=stamp, frame_id=camera_frame)

        rgb = cv2.cvtColor(cv2.imread(str(frame["rgb"]), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        depth = np.load(frame["depth"]).astype(np.float32)
        raw_reference_pose = load_pose_matrix(frame["pose"])
        pose = bonn_optical_pose(
            raw_reference_pose,
            invert_reference=bool(self.get_parameter("invert_reference_camera_pose").value),
            invert_optical_calibration=bool(self.get_parameter("invert_optical_calibration").value),
        )

        self.rgb_publisher.publish(rgb_image_msg(rgb, header))
        self.depth_publisher.publish(depth_image_msg(depth, header))
        self.camera_info_publisher.publish(camera_info_msg(self.intrinsics, header))
        self.gt_pose_publisher.publish(pose_stamped_msg(pose, stamp, world_frame))
        current_points = depth_points_in_world(
            depth,
            pose,
            intrinsics=self.intrinsics,
            stride=int(self.get_parameter("pointcloud_stride").value),
        )
        if bool(self.get_parameter("accumulate_pointcloud").value):
            self.accumulated_points.extend(current_points)
            max_points = int(self.get_parameter("max_accumulated_points").value)
            if max_points > 0 and len(self.accumulated_points) > max_points:
                self.accumulated_points = self.accumulated_points[-max_points:]
            points = self.accumulated_points
        else:
            points = current_points

        self.pointcloud_publisher.publish(
            pointcloud_msg(points, stamp, world_frame)
        )

        self.frame_index += 1


def collect_frames(scan_path: Path) -> list[dict[str, Path]]:
    rgb_files = {path.stem: path for path in sorted((scan_path / "rgb").glob("*.png"))}
    depth_files = {path.stem: path for path in sorted((scan_path / "depth").glob("*.npy"))}
    pose_files = {
        path.stem: path
        for path in sorted((scan_path / "gt_camera_poses").glob("*.txt"))
    }
    stems = sorted(set(rgb_files) & set(depth_files) & set(pose_files))
    return [
        {"rgb": rgb_files[stem], "depth": depth_files[stem], "pose": pose_files[stem]}
        for stem in stems
    ]


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def rgb_image_msg(rgb: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = rgb.tobytes()
    return msg


def depth_image_msg(depth: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(depth.shape[0])
    msg.width = int(depth.shape[1])
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = int(depth.shape[1] * 4)
    msg.data = depth.astype(np.float32).tobytes()
    return msg


def camera_info_msg(intrinsics: dict, header: Header) -> CameraInfo:
    width = int(intrinsics["width"])
    height = int(intrinsics["height"])
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])

    msg = CameraInfo()
    msg.header = header
    msg.width = width
    msg.height = height
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    return msg


def load_pose_matrix(path: Path) -> PoseData:
    matrix = np.loadtxt(path)
    position = (float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3]))
    orientation = quaternion_from_matrix(matrix[:3, :3])
    return PoseData(position=position, orientation_xyzw=orientation)


def bonn_optical_pose(
    reference_pose: PoseData,
    *,
    invert_reference: bool,
    invert_optical_calibration: bool,
) -> PoseData:
    world_from_mocap_camera = inverse_pose(reference_pose) if invert_reference else reference_pose
    mocap_from_optical = (
        inverse_pose(BONN_MOCAP_FROM_OPTICAL)
        if invert_optical_calibration
        else BONN_MOCAP_FROM_OPTICAL
    )
    return compose_pose(world_from_mocap_camera, mocap_from_optical)


def quaternion_from_matrix(matrix: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = (trace + 1.0) ** 0.5 * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = (1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) ** 0.5 * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif axis == 1:
            scale = (1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) ** 0.5 * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = (1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) ** 0.5 * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return (float(x), float(y), float(z), float(w))


def pose_stamped_msg(pose: PoseData, stamp, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose.position[0]
    msg.pose.position.y = pose.position[1]
    msg.pose.position.z = pose.position[2]
    msg.pose.orientation.x = pose.orientation_xyzw[0]
    msg.pose.orientation.y = pose.orientation_xyzw[1]
    msg.pose.orientation.z = pose.orientation_xyzw[2]
    msg.pose.orientation.w = pose.orientation_xyzw[3]
    return msg


def depth_points_in_world(
    depth: np.ndarray,
    pose: PoseData,
    *,
    intrinsics: dict,
    stride: int,
) -> list[tuple[float, float, float]]:
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    step = max(1, stride)
    points = []
    for v in range(0, depth.shape[0], step):
        for u in range(0, depth.shape[1], step):
            z = float(depth[v, u])
            if not np.isfinite(z) or z <= 0.0:
                continue
            local = ((u - cx) * z / fx, (v - cy) * z / fy, z)
            rotated = rotate_vector(pose.orientation_xyzw, local)
            points.append(tuple(pose.position[index] + rotated[index] for index in range(3)))
    return points


def pointcloud_msg(
    points: list[tuple[float, float, float]],
    stamp,
    frame_id: str,
):
    header = Header(stamp=stamp, frame_id=frame_id)
    return point_cloud2.create_cloud_xyz32(header, points)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BonnScanReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
