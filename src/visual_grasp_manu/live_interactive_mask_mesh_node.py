from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import shutil

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from visual_grasp_manu.bonn_object_data import write_yaml
from visual_grasp_manu.interactive_mask_tracking import accept_track_stats, compute_track_stats
from visual_grasp_manu.mask_generation import (
    build_backend,
    create_mask_overlay,
    ensure_binary_mask,
    mask_box_xyxy,
)
from visual_grasp_manu.scan_dataset_recorder_node import (
    TimedMessage,
    image_to_depth_array,
    image_to_rgb_array,
    nearest_message,
    pose_matrix_from_odometry,
    prepare_output_dirs,
    stamp_ns_from_msg,
    write_pose_matrix,
)
from visual_grasp_manu.tsdf_mesh import generate_tsdf_mesh


@dataclass(frozen=True)
class LiveFrame:
    rgb: Image
    depth: Image
    camera_info: CameraInfo
    odom: Odometry


class LiveInteractiveMaskMeshNode(Node):
    def __init__(self) -> None:
        super().__init__("live_interactive_mask_mesh_node")
        self.declare_parameter("output_path", "outputs/datasets/object_scan_live_interactive")
        self.declare_parameter("rgb_topic", "/camera/ee_cam/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/ee_cam/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/ee_cam/color/camera_info")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("overlay_image_topic", "/visual_grasp_manu/mask_overlay/image")
        self.declare_parameter("masked_cloud_topic", "/visual_grasp_manu/mask_overlay/cloud")
        self.declare_parameter("mask_marker_topic", "/visual_grasp_manu/mask_overlay/markers")
        self.declare_parameter("mesh_marker_topic", "/visual_grasp_manu/object_mesh_marker")
        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("object_prompt", "")
        self.declare_parameter("backend", "grounding_dino_sam2")
        self.declare_parameter(
            "grounding_config",
            "/tmp/visual_grasp_manu/model_repos/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py",
        )
        self.declare_parameter(
            "grounding_checkpoint",
            "/tmp/visual_grasp_manu/checkpoints/groundingdino_swint_ogc.pth",
        )
        self.declare_parameter("sam2_config", "configs/sam2.1/sam2.1_hiera_t.yaml")
        self.declare_parameter("sam2_checkpoint", "/tmp/visual_grasp_manu/checkpoints/sam2.1_hiera_tiny.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("grounding_device", "cpu")
        self.declare_parameter("box_threshold", 0.30)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("hsv_lower", "90,90,25")
        self.declare_parameter("hsv_upper", "135,255,255")
        self.declare_parameter("hsv_min_area", 50)
        self.declare_parameter("frame_stride", 10)
        self.declare_parameter("max_frames", 60)
        self.declare_parameter("sync_tolerance_sec", 0.06)
        self.declare_parameter("pose_tolerance_sec", 0.20)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("pointcloud_stride", 4)
        self.declare_parameter("max_center_jump_px", 80.0)
        self.declare_parameter("min_area_ratio", 0.35)
        self.declare_parameter("max_area_ratio", 2.8)
        self.declare_parameter("max_depth_jump_m", 0.12)
        self.declare_parameter("voxel_length", 0.003)
        self.declare_parameter("sdf_trunc", 0.015)
        self.declare_parameter("depth_trunc", 1.5)
        self.declare_parameter("min_mask_pixels", 50)
        self.declare_parameter("overwrite", True)
        self.declare_parameter("auto_accept_initial", False)

        self.output_path = Path(str(self.get_parameter("output_path").value))
        if self.output_path.exists() and bool(self.get_parameter("overwrite").value):
            shutil.rmtree(self.output_path)
        prepare_output_dirs(self.output_path, overwrite=bool(self.get_parameter("overwrite").value))
        (self.output_path / "masks").mkdir(parents=True, exist_ok=True)
        (self.output_path / "mask_overlays").mkdir(parents=True, exist_ok=True)

        self.rgb_queue: deque[TimedMessage] = deque(maxlen=200)
        self.depth_queue: deque[TimedMessage] = deque(maxlen=200)
        self.camera_info_queue: deque[TimedMessage] = deque(maxlen=200)
        self.odom_queue: deque[TimedMessage] = deque(maxlen=400)
        self.input_frame_count = 0
        self.accepted_frame_count = 0
        self.accepted_stems: list[str] = []
        self.previous_stats = None
        self.mesh_generated = False
        self.waiting_for_initial_acceptance = True

        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, str(self.get_parameter("rgb_topic").value), self.rgb_callback, sensor_qos)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self.depth_callback, sensor_qos)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            sensor_qos,
        )
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, sensor_qos)

        self.overlay_publisher = self.create_publisher(Image, str(self.get_parameter("overlay_image_topic").value), 10)
        self.cloud_publisher = self.create_publisher(PointCloud2, str(self.get_parameter("masked_cloud_topic").value), 10)
        self.mask_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("mask_marker_topic").value),
            10,
        )
        self.mesh_publisher = self.create_publisher(MarkerArray, str(self.get_parameter("mesh_marker_topic").value), 10)

        prompt = str(self.get_parameter("object_prompt").value).strip()
        if not prompt:
            try:
                prompt = input("Enter object query for live mask tracking: ").strip()
            except EOFError as exc:
                raise RuntimeError(
                    "No terminal input is available. Pass object_prompt:=... or run "
                    "live_interactive_mask_mesh_node directly in a foreground terminal."
                ) from exc
        if not prompt:
            raise RuntimeError("Object query cannot be empty.")
        self.prompt = prompt
        self.backend = build_backend(backend_args_from_parameters(self))
        self.write_metadata()
        self.get_logger().info(
            f"Live interactive mask/mesh node ready. Prompt='{self.prompt}', output={self.output_path}"
        )

    def rgb_callback(self, msg: Image) -> None:
        self.rgb_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))
        self.try_process_frame()

    def depth_callback(self, msg: Image) -> None:
        self.depth_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))
        self.write_camera_intrinsics(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))

    def try_process_frame(self) -> None:
        if self.mesh_generated:
            return
        while self.rgb_queue and self.depth_queue and self.camera_info_queue and self.odom_queue:
            rgb = self.rgb_queue.popleft()
            depth = nearest_message(self.depth_queue, rgb.stamp_ns)
            info = nearest_message(self.camera_info_queue, rgb.stamp_ns)
            odom = nearest_message(self.odom_queue, rgb.stamp_ns)
            if depth is None or info is None or odom is None:
                return
            if abs(depth.stamp_ns - rgb.stamp_ns) > seconds_to_ns(float(self.get_parameter("sync_tolerance_sec").value)):
                continue
            if abs(info.stamp_ns - rgb.stamp_ns) > seconds_to_ns(float(self.get_parameter("sync_tolerance_sec").value)):
                continue
            if abs(odom.stamp_ns - rgb.stamp_ns) > seconds_to_ns(float(self.get_parameter("pose_tolerance_sec").value)):
                continue

            self.input_frame_count += 1
            if (self.input_frame_count - 1) % max(1, int(self.get_parameter("frame_stride").value)) != 0:
                continue
            self.process_frame(LiveFrame(rgb.msg, depth.msg, info.msg, odom.msg))
            if self.mesh_generated:
                return

    def process_frame(self, frame: LiveFrame) -> None:
        rgb = image_to_rgb_array(frame.rgb)
        depth = image_to_depth_array(frame.depth)
        stem = f"{self.accepted_frame_count + 1:06d}"
        scratch_rgb = self.output_path / "_live_current.png"
        cv2.imwrite(str(scratch_rgb), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        prediction = self.backend.predict(scratch_rgb, self.prompt)
        mask = ensure_binary_mask(prediction.mask)
        stats = compute_track_stats(
            stem,
            mask,
            depth,
            depth_scale=float(self.get_parameter("depth_scale").value),
        )

        overlay = create_mask_overlay(rgb, mask)
        self.overlay_publisher.publish(rgb_msg(overlay, frame.rgb.header))
        self.cloud_publisher.publish(masked_cloud_msg(self, rgb, depth, mask, frame.camera_info, frame.odom))
        self.mask_marker_publisher.publish(mask_marker_array(self, depth, mask, frame.camera_info, frame.odom))

        if self.waiting_for_initial_acceptance:
            print(
                "Initial mask proposal is published in RViz on "
                f"{self.get_parameter('overlay_image_topic').value} and "
                f"{self.get_parameter('masked_cloud_topic').value} plus "
                f"{self.get_parameter('mask_marker_topic').value}",
                flush=True,
            )
            if bool(self.get_parameter("auto_accept_initial").value):
                self.get_logger().info("auto_accept_initial=true; accepting initial mask proposal.")
            else:
                try:
                    answer = input("Accept this mask and start tracking? [y/N]: ").strip().lower()
                except EOFError as exc:
                    raise RuntimeError(
                        "No terminal input is available for mask acceptance. "
                        "Run this node in a foreground terminal or set auto_accept_initial:=true."
                    ) from exc
                if answer not in {"y", "yes"}:
                    raise RuntimeError("Initial mask rejected by user.")
            self.waiting_for_initial_acceptance = False
        else:
            accepted, reason = accept_track_stats(
                stats,
                self.previous_stats,
                max_center_jump_px=float(self.get_parameter("max_center_jump_px").value),
                min_area_ratio=float(self.get_parameter("min_area_ratio").value),
                max_area_ratio=float(self.get_parameter("max_area_ratio").value),
                max_depth_jump_m=float(self.get_parameter("max_depth_jump_m").value),
            )
            if not accepted:
                self.get_logger().warn(f"Rejected frame {stem}: {reason}")
                return

        self.previous_stats = stats
        self.write_accepted_frame(stem, rgb, depth, mask, overlay, frame.camera_info, frame.odom)
        if self.accepted_frame_count >= int(self.get_parameter("max_frames").value):
            self.generate_and_publish_mesh()

    def write_accepted_frame(
        self,
        stem: str,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        overlay: np.ndarray,
        camera_info: CameraInfo,
        odom: Odometry,
    ) -> None:
        cv2.imwrite(str(self.output_path / "rgb" / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        np.save(self.output_path / "depth" / f"{stem}.npy", depth)
        cv2.imwrite(str(self.output_path / "masks" / f"{stem}.png"), mask)
        cv2.imwrite(str(self.output_path / "mask_overlays" / f"{stem}.png"), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
        write_pose_matrix(self.output_path / "camera_poses" / f"{stem}.txt", pose_matrix_from_odometry(odom))
        self.write_camera_intrinsics(camera_info)
        self.accepted_frame_count += 1
        self.accepted_stems.append(stem)
        (self.output_path / "accepted_frames.txt").write_text(
            "\n".join(self.accepted_stems) + "\n",
            encoding="utf-8",
        )
        self.write_metadata()
        self.get_logger().info(f"Accepted live mask frame {stem}")

    def generate_and_publish_mesh(self) -> None:
        self.get_logger().info("Accepted frame target reached. Generating TSDF mesh.")
        result = generate_tsdf_mesh(
            self.output_path,
            voxel_length=float(self.get_parameter("voxel_length").value),
            sdf_trunc=float(self.get_parameter("sdf_trunc").value),
            depth_trunc=float(self.get_parameter("depth_trunc").value),
            min_mask_pixels=int(self.get_parameter("min_mask_pixels").value),
            frame_list_path=self.output_path / "accepted_frames.txt",
        )
        self.mesh_publisher.publish(mesh_marker_array(result.mesh_path, str(self.get_parameter("world_frame_id").value)))
        self.mesh_generated = True
        self.get_logger().info(f"Published mesh marker for {result.mesh_path}")

    def write_camera_intrinsics(self, msg: CameraInfo) -> None:
        data = {
            "width": int(msg.width),
            "height": int(msg.height),
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "cx": float(msg.k[2]),
            "cy": float(msg.k[5]),
            "depth_scale": float(self.get_parameter("depth_scale").value),
            "depth_encoding": "npy",
            "frame_id": msg.header.frame_id,
        }
        write_yaml(self.output_path / "camera_intrinsics.yaml", data)

    def write_metadata(self) -> None:
        write_yaml(
            self.output_path / "metadata.yaml",
            {
                "dataset_id": self.output_path.name,
                "mode": "mesh_generation",
                "multi_view": self.accepted_frame_count > 1,
                "object_prompt": self.prompt,
                "mask_backend": str(self.get_parameter("backend").value),
                "pose_source": "rtabmap_rgbd_slam",
                "pose_source_mode": "odom",
                "world_frame_id": str(self.get_parameter("world_frame_id").value),
                "frames_written": self.accepted_frame_count,
            },
        )


def backend_args_from_parameters(node: Node) -> object:
    class Args:
        pass

    args = Args()
    args.backend = str(node.get_parameter("backend").value)
    args.grounding_config = Path(str(node.get_parameter("grounding_config").value))
    args.grounding_checkpoint = Path(str(node.get_parameter("grounding_checkpoint").value))
    args.sam2_config = str(node.get_parameter("sam2_config").value)
    args.sam2_checkpoint = Path(str(node.get_parameter("sam2_checkpoint").value))
    args.device = str(node.get_parameter("device").value)
    args.grounding_device = str(node.get_parameter("grounding_device").value)
    args.box_threshold = float(node.get_parameter("box_threshold").value)
    args.text_threshold = float(node.get_parameter("text_threshold").value)
    args.multimask_output = False
    args.hsv_lower = str(node.get_parameter("hsv_lower").value)
    args.hsv_upper = str(node.get_parameter("hsv_upper").value)
    args.hsv_min_area = int(node.get_parameter("hsv_min_area").value)
    return args


def rgb_msg(rgb: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def masked_cloud_msg(
    node: Node,
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    camera_info: CameraInfo,
    odom: Odometry,
) -> PointCloud2:
    k = camera_info.k
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    depth_scale = float(node.get_parameter("depth_scale").value)
    stride = max(1, int(node.get_parameter("pointcloud_stride").value))
    pose = pose_matrix_from_odometry(odom)
    points = []
    for v in range(0, depth.shape[0], stride):
        for u in range(0, depth.shape[1], stride):
            if mask[v, u] == 0:
                continue
            z = float(depth[v, u]) * depth_scale
            if not np.isfinite(z) or z <= 0.0:
                continue
            local = np.asarray([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0], dtype=np.float64)
            world = pose @ local
            points.append((float(world[0]), float(world[1]), float(world[2])))
    header = Header(stamp=odom.header.stamp, frame_id=str(node.get_parameter("world_frame_id").value))
    return point_cloud2.create_cloud_xyz32(header, points)


def mask_marker_array(
    node: Node,
    depth: np.ndarray,
    mask: np.ndarray,
    camera_info: CameraInfo,
    odom: Odometry,
) -> MarkerArray:
    marker_array = MarkerArray()
    delete = Marker()
    delete.action = Marker.DELETEALL
    marker_array.markers.append(delete)

    box = mask_box_xyxy(mask)
    if box is None:
        return marker_array

    masked_depth = depth[mask > 0]
    if masked_depth.size == 0:
        return marker_array

    depth_scale = float(node.get_parameter("depth_scale").value)
    depth_m = np.asarray(masked_depth, dtype=np.float32) * depth_scale
    depth_m = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
    if depth_m.size == 0:
        return marker_array
    z = float(np.median(depth_m))

    x0, y0, x1, y1 = box
    corners = [
        project_pixel_to_world(x0, y0, z, camera_info, odom),
        project_pixel_to_world(x1, y0, z, camera_info, odom),
        project_pixel_to_world(x1, y1, z, camera_info, odom),
        project_pixel_to_world(x0, y1, z, camera_info, odom),
        project_pixel_to_world(x0, y0, z, camera_info, odom),
    ]

    rectangle = Marker()
    rectangle.header.frame_id = str(node.get_parameter("world_frame_id").value)
    rectangle.header.stamp = odom.header.stamp
    rectangle.ns = "live_mask_bbox"
    rectangle.id = 1
    rectangle.type = Marker.LINE_STRIP
    rectangle.action = Marker.ADD
    rectangle.pose.orientation.w = 1.0
    rectangle.scale.x = 0.006
    rectangle.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.95)
    rectangle.points = [Point(x=x, y=y, z=z) for x, y, z in corners]
    marker_array.markers.append(rectangle)

    center = Marker()
    center.header = rectangle.header
    center.ns = "live_mask_center"
    center.id = 2
    center.type = Marker.SPHERE
    center.action = Marker.ADD
    center.pose.position = rectangle.points[0]
    center.pose.orientation.w = 1.0
    center.scale.x = 0.025
    center.scale.y = 0.025
    center.scale.z = 0.025
    center.color = ColorRGBA(r=0.0, g=0.85, b=1.0, a=0.85)
    center_world = project_pixel_to_world((x0 + x1) / 2.0, (y0 + y1) / 2.0, z, camera_info, odom)
    center.pose.position = Point(x=center_world[0], y=center_world[1], z=center_world[2])
    marker_array.markers.append(center)
    return marker_array


def project_pixel_to_world(
    u: float,
    v: float,
    z: float,
    camera_info: CameraInfo,
    odom: Odometry,
) -> tuple[float, float, float]:
    k = camera_info.k
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    local = np.asarray([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0], dtype=np.float64)
    world = pose_matrix_from_odometry(odom) @ local
    return (float(world[0]), float(world[1]), float(world[2]))


def mesh_marker_array(mesh_path: Path, frame_id: str) -> MarkerArray:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "object_mesh"
    marker.id = 1
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=0.75)
    marker.mesh_use_embedded_materials = False
    marker.mesh_resource = f"file://{mesh_path.resolve()}"
    return MarkerArray(markers=[marker])


def seconds_to_ns(value: float) -> int:
    return int(value * 1_000_000_000)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LiveInteractiveMaskMeshNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
