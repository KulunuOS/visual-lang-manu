from __future__ import annotations

from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_manu.camera_pose_visualizer_node import INFERRED_STYLE, track_markers
from visual_grasp_manu.live_interactive_mask_mesh_node import mesh_marker_array
from visual_grasp_manu.mask_generation import mask_box_xyxy


class PrecomputedMaskPoseDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("precomputed_mask_pose_demo_node")
        self.declare_parameter("dataset_path", "outputs/datasets/object_scan_001_scan_blue_60_sam2")
        self.declare_parameter("world_frame_id", "odom")
        self.declare_parameter("rate_hz", 6.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("live_cloud_stride", 8)
        self.declare_parameter("masked_cloud_stride", 3)
        self.declare_parameter("frustum_scale", 0.04)
        self.declare_parameter("frustum_stride", 8)
        self.declare_parameter("path_line_width", 0.004)
        self.declare_parameter("frustum_line_width", 0.0015)

        self.dataset_path = Path(str(self.get_parameter("dataset_path").value)).expanduser()
        self.world_frame_id = str(self.get_parameter("world_frame_id").value)
        self.intrinsics = load_intrinsics(self.dataset_path / "camera_intrinsics.yaml")
        self.frames = collect_frames(self.dataset_path)
        if not self.frames:
            raise ValueError(f"No precomputed mask frames found in: {self.dataset_path}")

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.overlay_publisher = self.create_publisher(Image, "/visual_grasp_manu/mask_overlay/image", sensor_qos)
        self.live_cloud_publisher = self.create_publisher(PointCloud2, "/visual_grasp_manu/live/rgbd_cloud", sensor_qos)
        self.masked_cloud_publisher = self.create_publisher(PointCloud2, "/visual_grasp_manu/mask_overlay/cloud", sensor_qos)
        self.mask_marker_publisher = self.create_publisher(MarkerArray, "/visual_grasp_manu/mask_overlay/markers", marker_qos)
        self.mesh_publisher = self.create_publisher(MarkerArray, "/visual_grasp_manu/object_mesh_marker", marker_qos)
        self.path_publisher = self.create_publisher(PathMsg, "/visual_grasp_manu/inferred_camera_path", 10)
        self.pose_marker_publisher = self.create_publisher(MarkerArray, "/visual_grasp_manu/camera_pose_markers", marker_qos)

        self.poses: deque[PoseStamped] = deque(maxlen=len(self.frames))
        self.frame_index = 0
        period = 1.0 / max(0.1, float(self.get_parameter("rate_hz").value))
        self.create_timer(period, self.publish_next_frame)
        self.get_logger().info(
            f"Replaying {len(self.frames)} precomputed mask frames from {self.dataset_path}"
        )

    def publish_next_frame(self) -> None:
        if self.frame_index >= len(self.frames):
            if not bool(self.get_parameter("loop").value):
                return
            self.frame_index = 0
            self.poses.clear()

        frame = self.frames[self.frame_index]
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.world_frame_id)
        camera_pose = pose_stamped_from_matrix(read_pose_matrix(frame["pose"]), header)
        self.poses.append(camera_pose)

        rgb = read_rgb(frame["rgb"])
        depth = np.load(frame["depth"])
        mask = read_mask(frame["mask"])
        overlay = read_rgb(frame["overlay"]) if frame["overlay"].is_file() else apply_overlay(rgb, mask)

        self.overlay_publisher.publish(image_msg(overlay, Header(stamp=stamp, frame_id=self.intrinsics["frame_id"])))
        self.live_cloud_publisher.publish(
            cloud_msg(
                header,
                rgb,
                depth,
                np.ones(depth.shape, dtype=np.uint8) * 255,
                self.intrinsics,
                read_pose_matrix(frame["pose"]),
                depth_scale=float(self.get_parameter("depth_scale").value),
                stride=int(self.get_parameter("live_cloud_stride").value),
            )
        )
        self.masked_cloud_publisher.publish(
            cloud_msg(
                header,
                rgb,
                depth,
                mask,
                self.intrinsics,
                read_pose_matrix(frame["pose"]),
                depth_scale=float(self.get_parameter("depth_scale").value),
                stride=int(self.get_parameter("masked_cloud_stride").value),
            )
        )
        self.mask_marker_publisher.publish(mask_markers(header, depth, mask, self.intrinsics, read_pose_matrix(frame["pose"])))
        self.publish_camera_pose_overlay(header)
        mesh_path = self.dataset_path / "mesh" / "object.ply"
        if mesh_path.is_file():
            self.mesh_publisher.publish(mesh_marker_array(mesh_path, self.world_frame_id))

        self.frame_index += 1

    def publish_camera_pose_overlay(self, header: Header) -> None:
        path = PathMsg()
        path.header = header
        path.poses = list(self.poses)
        self.path_publisher.publish(path)
        markers = track_markers(
            self.poses,
            INFERRED_STYLE,
            frustum_scale=float(self.get_parameter("frustum_scale").value),
            frustum_stride=int(self.get_parameter("frustum_stride").value),
            path_line_width=float(self.get_parameter("path_line_width").value),
            frustum_line_width=float(self.get_parameter("frustum_line_width").value),
            publish_path_marker=True,
            frustum_mode="sampled",
        )
        self.pose_marker_publisher.publish(MarkerArray(markers=[delete_marker(), *markers]))


def collect_frames(dataset_path: Path) -> list[dict[str, Path]]:
    rgb_files = {path.stem: path for path in sorted((dataset_path / "rgb").glob("*.png"))}
    depth_files = {path.stem: path for path in sorted((dataset_path / "depth").glob("*.npy"))}
    mask_files = {path.stem: path for path in sorted((dataset_path / "masks").glob("*.png"))}
    overlay_files = {path.stem: path for path in sorted((dataset_path / "mask_overlays").glob("*.png"))}
    pose_files = {path.stem: path for path in sorted((dataset_path / "camera_poses").glob("*.txt"))}
    stems = sorted(rgb_files.keys() & depth_files.keys() & mask_files.keys() & pose_files.keys())
    return [
        {
            "rgb": rgb_files[stem],
            "depth": depth_files[stem],
            "mask": mask_files[stem],
            "overlay": overlay_files.get(stem, Path()),
            "pose": pose_files[stem],
        }
        for stem in stems
    ]


def load_intrinsics(path: Path) -> dict[str, float | int | str]:
    data = yaml.safe_load(path.read_text())
    return {
        "width": int(data["width"]),
        "height": int(data["height"]),
        "fx": float(data["fx"]),
        "fy": float(data["fy"]),
        "cx": float(data["cx"]),
        "cy": float(data["cy"]),
        "frame_id": str(data.get("frame_id", "camera_color_optical_frame")),
    }


def read_rgb(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def read_mask(path: Path) -> np.ndarray:
    mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise ValueError(f"Could not read mask: {path}")
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def read_pose_matrix(path: Path) -> np.ndarray:
    matrix = np.loadtxt(path, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected 4x4 pose matrix in {path}, got {matrix.shape}")
    return matrix


def image_msg(rgb: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def cloud_msg(
    header: Header,
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics: dict[str, float | int | str],
    camera_pose: np.ndarray,
    *,
    depth_scale: float,
    stride: int,
) -> PointCloud2:
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    points = []
    step = max(1, stride)
    for v in range(0, depth.shape[0], step):
        for u in range(0, depth.shape[1], step):
            if mask[v, u] == 0:
                continue
            z = float(depth[v, u]) * depth_scale
            if not np.isfinite(z) or z <= 0.0:
                continue
            local = np.asarray([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0], dtype=np.float64)
            world = camera_pose @ local
            color = int(rgb[v, u, 0]) << 16 | int(rgb[v, u, 1]) << 8 | int(rgb[v, u, 2])
            points.append((float(world[0]), float(world[1]), float(world[2]), color))
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, points)


def mask_markers(
    header: Header,
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics: dict[str, float | int | str],
    camera_pose: np.ndarray,
) -> MarkerArray:
    marker_array = MarkerArray(markers=[delete_marker()])
    box = mask_box_xyxy(mask)
    if box is None:
        return marker_array
    masked_depth = depth[mask > 0]
    masked_depth = masked_depth[np.isfinite(masked_depth) & (masked_depth > 0)]
    if masked_depth.size == 0:
        return marker_array
    z = float(np.median(masked_depth)) * 0.001
    x0, y0, x1, y1 = box
    corners = [
        project_pixel_to_world(x0, y0, z, intrinsics, camera_pose),
        project_pixel_to_world(x1, y0, z, intrinsics, camera_pose),
        project_pixel_to_world(x1, y1, z, intrinsics, camera_pose),
        project_pixel_to_world(x0, y1, z, intrinsics, camera_pose),
        project_pixel_to_world(x0, y0, z, intrinsics, camera_pose),
    ]

    rectangle = Marker()
    rectangle.header = header
    rectangle.ns = "precomputed_mask_bbox"
    rectangle.id = 1
    rectangle.type = Marker.LINE_STRIP
    rectangle.action = Marker.ADD
    rectangle.pose.orientation.w = 1.0
    rectangle.scale.x = 0.006
    rectangle.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.95)
    rectangle.points = [Point(x=x, y=y, z=z) for x, y, z in corners]
    marker_array.markers.append(rectangle)
    return marker_array


def project_pixel_to_world(
    u: float,
    v: float,
    z: float,
    intrinsics: dict[str, float | int | str],
    camera_pose: np.ndarray,
) -> tuple[float, float, float]:
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    local = np.asarray([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0], dtype=np.float64)
    world = camera_pose @ local
    return (float(world[0]), float(world[1]), float(world[2]))


def pose_stamped_from_matrix(matrix: np.ndarray, header: Header) -> PoseStamped:
    pose = PoseStamped()
    pose.header = header
    pose.pose = pose_from_matrix(matrix)
    return pose


def pose_from_matrix(matrix: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    qx, qy, qz, qw = quaternion_from_rotation_matrix(matrix[:3, :3])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def apply_overlay(rgb: np.ndarray, mask: np.ndarray) -> np.ndarray:
    overlay = rgb.copy()
    tint = np.zeros_like(rgb)
    tint[:, :, 2] = 255
    alpha = (mask > 0)[:, :, None].astype(np.float32) * 0.45
    return np.clip(overlay * (1.0 - alpha) + tint * alpha, 0, 255).astype(np.uint8)


def delete_marker() -> Marker:
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PrecomputedMaskPoseDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
