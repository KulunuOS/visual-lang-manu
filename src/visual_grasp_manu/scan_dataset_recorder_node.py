from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from visual_grasp_manu.bonn_object_data import pose_matrix, write_yaml


@dataclass(frozen=True)
class TimedMessage:
    stamp_ns: int
    msg: object


class ScanDatasetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_dataset_recorder_node")
        self.declare_parameter("output_path", "outputs/datasets/object_scan_001_scan")
        self.declare_parameter("rgb_topic", "/camera/ee_cam/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/ee_cam/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/ee_cam/color/camera_info")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("object_prompt", "object")
        self.declare_parameter("source_bag", "object_scan_001")
        self.declare_parameter("pose_source", "rtabmap_rgbd_slam")
        self.declare_parameter("pose_source_mode", "odom")
        self.declare_parameter("world_frame_id", "odom")
        self.declare_parameter("camera_frame_id", "")
        self.declare_parameter("frame_stride", 10)
        self.declare_parameter("max_frames", 120)
        self.declare_parameter("sync_tolerance_sec", 0.06)
        self.declare_parameter("pose_tolerance_sec", 0.20)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("overwrite", True)
        self.declare_parameter("queue_size", 200)

        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.overwrite = bool(self.get_parameter("overwrite").value)
        self.frame_stride = max(1, int(self.get_parameter("frame_stride").value))
        self.max_frames = int(self.get_parameter("max_frames").value)
        self.sync_tolerance_ns = seconds_to_ns(float(self.get_parameter("sync_tolerance_sec").value))
        self.pose_tolerance_ns = seconds_to_ns(float(self.get_parameter("pose_tolerance_sec").value))
        self.queue_size = max(10, int(self.get_parameter("queue_size").value))
        self.pose_source_mode = str(self.get_parameter("pose_source_mode").value)
        if self.pose_source_mode not in {"odom", "tf"}:
            raise ValueError("pose_source_mode must be either 'odom' or 'tf'")

        self.rgb_queue: Deque[TimedMessage] = deque(maxlen=self.queue_size)
        self.depth_queue: Deque[TimedMessage] = deque(maxlen=self.queue_size)
        self.camera_info_queue: Deque[TimedMessage] = deque(maxlen=self.queue_size)
        self.odom_queue: Deque[TimedMessage] = deque(maxlen=self.queue_size * 2)
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        self.input_frame_count = 0
        self.written_frame_count = 0
        self.last_camera_info: CameraInfo | None = None
        self.finished = False

        prepare_output_dirs(self.output_path, overwrite=self.overwrite)

        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self.rgb_callback,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            sensor_qos,
        )
        if self.pose_source_mode == "odom":
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odom_topic").value),
                self.odom_callback,
                odom_qos,
            )
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(5.0, self.log_status)
        pose_description = (
            str(self.get_parameter("odom_topic").value)
            if self.pose_source_mode == "odom"
            else f"TF {self.get_parameter('world_frame_id').value} -> camera"
        )
        self.get_logger().info(f"Recording scan dataset to {self.output_path} from RGB-D and {pose_description}")

    def rgb_callback(self, msg: Image) -> None:
        self.rgb_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))
        self.try_write_frames()

    def depth_callback(self, msg: Image) -> None:
        self.depth_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))
        self.try_write_frames()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.last_camera_info = msg
        self.camera_info_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))
        if self.written_frame_count == 0:
            self.write_camera_intrinsics(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_queue.append(TimedMessage(stamp_ns_from_msg(msg), msg))
        self.try_write_frames()

    def try_write_frames(self) -> None:
        if self.finished:
            return
        while self.rgb_queue and self.depth_queue and self.camera_info_queue and self.pose_source_ready():
            rgb = self.rgb_queue[0]
            depth = nearest_message(self.depth_queue, rgb.stamp_ns)
            camera_info = nearest_message(self.camera_info_queue, rgb.stamp_ns)
            pose_record = self.lookup_pose_record(rgb.msg, camera_info.msg if camera_info else None)

            if depth is None or abs(depth.stamp_ns - rgb.stamp_ns) > self.sync_tolerance_ns:
                if self.should_drop_old_rgb(rgb.stamp_ns):
                    self.rgb_queue.popleft()
                return
            if camera_info is None or abs(camera_info.stamp_ns - rgb.stamp_ns) > self.sync_tolerance_ns:
                if self.should_drop_old_rgb(rgb.stamp_ns):
                    self.rgb_queue.popleft()
                return
            if pose_record is None or abs(pose_record.stamp_ns - rgb.stamp_ns) > self.pose_tolerance_ns:
                if self.should_drop_old_rgb(rgb.stamp_ns):
                    self.rgb_queue.popleft()
                return

            self.rgb_queue.popleft()
            discard_older_than(self.depth_queue, depth.stamp_ns)
            discard_older_than(self.camera_info_queue, camera_info.stamp_ns)
            if self.pose_source_mode == "odom":
                discard_older_than(self.odom_queue, pose_record.stamp_ns)

            self.input_frame_count += 1
            if (self.input_frame_count - 1) % self.frame_stride != 0:
                continue

            self.write_frame(
                rgb_msg=rgb.msg,
                depth_msg=depth.msg,
                camera_info_msg=camera_info.msg,
                camera_pose_matrix=pose_record.msg,
                pose_stamp_ns=pose_record.stamp_ns,
            )
            if self.max_frames > 0 and self.written_frame_count >= self.max_frames:
                self.finished = True
                self.write_metadata()
                self.get_logger().info(
                    f"Reached max_frames={self.max_frames}; wrote {self.written_frame_count} frames."
                )
                return

    def should_drop_old_rgb(self, stamp_ns: int) -> bool:
        pose_stamp = self.odom_queue[-1].stamp_ns if self.pose_source_mode == "odom" and self.odom_queue else stamp_ns
        latest_available = max(
            self.depth_queue[-1].stamp_ns if self.depth_queue else 0,
            self.camera_info_queue[-1].stamp_ns if self.camera_info_queue else 0,
            pose_stamp,
        )
        return latest_available - stamp_ns > max(self.pose_tolerance_ns, self.sync_tolerance_ns)

    def pose_source_ready(self) -> bool:
        return bool(self.odom_queue) if self.pose_source_mode == "odom" else self.tf_buffer is not None

    def lookup_pose_record(
        self,
        rgb_msg: Image,
        camera_info_msg: CameraInfo | None,
    ) -> TimedMessage | None:
        if self.pose_source_mode == "odom":
            odom = nearest_message(self.odom_queue, stamp_ns_from_msg(rgb_msg))
            if odom is None:
                return None
            return TimedMessage(odom.stamp_ns, pose_matrix_from_odometry(odom.msg))

        if self.tf_buffer is None:
            return None
        camera_frame_id = self.camera_frame_id(rgb_msg, camera_info_msg)
        if not camera_frame_id:
            self.get_logger().warn("No camera_frame_id available for TF pose lookup.")
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("world_frame_id").value),
                camera_frame_id,
                Time.from_msg(rgb_msg.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter("pose_tolerance_sec").value)),
            )
        except TransformException as exc:
            self.get_logger().debug(f"TF pose lookup failed for {camera_frame_id}: {exc}")
            return None

        return TimedMessage(stamp_ns_from_msg(transform), pose_matrix_from_transform(transform))

    def camera_frame_id(self, rgb_msg: Image, camera_info_msg: CameraInfo | None) -> str:
        configured_frame = str(self.get_parameter("camera_frame_id").value).strip()
        if configured_frame:
            return configured_frame
        if camera_info_msg is not None and camera_info_msg.header.frame_id:
            return camera_info_msg.header.frame_id
        return rgb_msg.header.frame_id

    def write_frame(
        self,
        *,
        rgb_msg: Image,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
        camera_pose_matrix: np.ndarray,
        pose_stamp_ns: int,
    ) -> None:
        self.write_camera_intrinsics(camera_info_msg)
        frame_index = self.written_frame_count + 1
        stem = f"{frame_index:06d}"
        rgb = image_to_rgb_array(rgb_msg)
        depth = image_to_depth_array(depth_msg)
        cv2.imwrite(str(self.output_path / "rgb" / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        np.save(self.output_path / "depth" / f"{stem}.npy", depth)
        write_pose_matrix(self.output_path / "camera_poses" / f"{stem}.txt", camera_pose_matrix)
        self.written_frame_count += 1
        if self.written_frame_count == 1 or self.written_frame_count % 25 == 0:
            self.write_metadata()
            self.get_logger().info(
                f"Wrote scan frame {stem}: rgb_stamp={stamp_ns_from_msg(rgb_msg)} "
                f"pose_stamp={pose_stamp_ns}"
            )

    def write_camera_intrinsics(self, msg: CameraInfo) -> None:
        depth_scale = float(self.get_parameter("depth_scale").value)
        data = {
            "width": int(msg.width),
            "height": int(msg.height),
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "cx": float(msg.k[2]),
            "cy": float(msg.k[5]),
            "depth_scale": depth_scale,
            "depth_encoding": "npy",
            "frame_id": msg.header.frame_id,
        }
        write_yaml(self.output_path / "camera_intrinsics.yaml", data)

    def write_metadata(self) -> None:
        data = {
            "dataset_id": self.output_path.name,
            "mode": "mesh_generation",
            "multi_view": self.written_frame_count > 1,
            "object_prompt": str(self.get_parameter("object_prompt").value),
            "mask_backend": "grounding_dino_sam2",
            "source_bag": str(self.get_parameter("source_bag").value),
            "pose_source": str(self.get_parameter("pose_source").value),
            "pose_source_mode": self.pose_source_mode,
            "world_frame_id": str(self.get_parameter("world_frame_id").value),
            "camera_frame_id": str(self.get_parameter("camera_frame_id").value),
            "rgb_topic": str(self.get_parameter("rgb_topic").value),
            "depth_topic": str(self.get_parameter("depth_topic").value),
            "camera_info_topic": str(self.get_parameter("camera_info_topic").value),
            "odom_topic": str(self.get_parameter("odom_topic").value),
            "frame_stride": self.frame_stride,
            "frames_written": self.written_frame_count,
            "notes": metadata_notes(self.pose_source_mode),
        }
        write_yaml(self.output_path / "metadata.yaml", data)

    def log_status(self) -> None:
        self.write_metadata()
        self.get_logger().info(
            "Scan recorder status: "
            f"input_frames={self.input_frame_count}, written={self.written_frame_count}, "
            f"rgb_q={len(self.rgb_queue)}, depth_q={len(self.depth_queue)}, "
            f"info_q={len(self.camera_info_queue)}, odom_q={len(self.odom_queue)}, "
            f"pose_source_mode={self.pose_source_mode}"
        )


def prepare_output_dirs(output_path: Path, *, overwrite: bool = True) -> None:
    for dirname in ["rgb", "depth", "camera_poses"]:
        directory = output_path / dirname
        directory.mkdir(parents=True, exist_ok=True)
        if not overwrite and any(directory.iterdir()):
            raise RuntimeError(
                f"Output directory is not empty: {directory}. "
                "Use overwrite:=true or choose a new output_path."
            )


def stamp_ns_from_msg(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def seconds_to_ns(value: float) -> int:
    return int(value * 1_000_000_000)


def nearest_message(queue: Deque[TimedMessage], stamp_ns: int) -> TimedMessage | None:
    if not queue:
        return None
    return min(queue, key=lambda record: abs(record.stamp_ns - stamp_ns))


def discard_older_than(queue: Deque[TimedMessage], stamp_ns: int) -> None:
    while len(queue) > 1 and queue[0].stamp_ns < stamp_ns:
        queue.popleft()


def image_to_rgb_array(msg: Image) -> np.ndarray:
    if msg.encoding == "rgb8":
        return image_buffer_to_array(msg, np.uint8, channels=3).copy()
    if msg.encoding == "bgr8":
        bgr = image_buffer_to_array(msg, np.uint8, channels=3)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if msg.encoding == "rgba8":
        rgba = image_buffer_to_array(msg, np.uint8, channels=4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
    if msg.encoding == "bgra8":
        bgra = image_buffer_to_array(msg, np.uint8, channels=4)
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2RGB)
    raise ValueError(f"Unsupported RGB image encoding: {msg.encoding}")


def image_to_depth_array(msg: Image) -> np.ndarray:
    if msg.encoding in {"16UC1", "mono16"}:
        return image_buffer_to_array(msg, np.uint16, channels=1).copy()
    if msg.encoding == "32FC1":
        return image_buffer_to_array(msg, np.float32, channels=1).copy()
    raise ValueError(f"Unsupported depth image encoding: {msg.encoding}")


def image_buffer_to_array(msg: Image, dtype: np.dtype, *, channels: int) -> np.ndarray:
    dtype = np.dtype(dtype)
    expected_row_bytes = int(msg.width) * channels * dtype.itemsize
    row_bytes = int(msg.step) if int(msg.step) > 0 else expected_row_bytes
    if row_bytes < expected_row_bytes:
        raise ValueError(
            f"Image step {row_bytes} is smaller than expected row size {expected_row_bytes}"
        )

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected_size = int(msg.height) * row_bytes
    if raw.size < expected_size:
        raise ValueError(
            f"Image data has {raw.size} bytes, expected at least {expected_size}"
        )

    rows = raw[:expected_size].reshape(int(msg.height), row_bytes)
    pixels = rows[:, :expected_row_bytes]
    array = pixels.reshape(int(msg.height), int(msg.width), channels, dtype.itemsize)
    array = np.ascontiguousarray(array).view(dtype)
    if channels == 1:
        return array.reshape(int(msg.height), int(msg.width))
    return array.reshape(int(msg.height), int(msg.width), channels)


def write_pose_matrix_from_odometry(path: Path, msg: Odometry) -> None:
    write_pose_matrix(path, pose_matrix_from_odometry(msg))


def pose_matrix_from_odometry(msg: Odometry) -> np.ndarray:
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    return np.asarray(
        pose_matrix(
            (float(position.x), float(position.y), float(position.z)),
            (
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        ),
        dtype=np.float64,
    )


def pose_matrix_from_transform(msg: TransformStamped) -> np.ndarray:
    transform = msg.transform
    return np.asarray(
        pose_matrix(
            (
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.translation.z),
            ),
            (
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            ),
        ),
        dtype=np.float64,
    )


def write_pose_matrix(path: Path, matrix: np.ndarray) -> None:
    lines = [" ".join(f"{value:.9g}" for value in row) for row in matrix]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def metadata_notes(pose_source_mode: str) -> str:
    if pose_source_mode == "tf":
        return (
            "Camera poses are sampled from robot TF for the RGB-D camera frame. "
            "This assumes calibrated camera extrinsics and synchronized robot state."
        )
    return (
        "Camera poses are nearest RTAB-Map odometry poses associated with "
        "sampled RGB-D frames during bag replay."
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanDatasetRecorderNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.write_metadata()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
