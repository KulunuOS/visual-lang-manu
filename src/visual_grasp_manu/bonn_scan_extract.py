from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from visual_grasp_manu.bonn_object_data import (
    parse_object_data,
    write_pose_matrix,
    write_yaml,
)


def remove_ros2_python_paths() -> None:
    sys.path[:] = [path for path in sys.path if "/opt/ros/humble" not in path]


def import_rosbag():
    remove_ros2_python_paths()
    import rosbag  # pylint: disable=import-outside-toplevel

    return rosbag


def extract_bonn_bag(
    bag_path: Path,
    output_path: Path,
    *,
    stride: int,
    max_frames: int,
    topic: str,
    object_prompt: str,
) -> int:
    rosbag = import_rosbag()
    output_path.mkdir(parents=True, exist_ok=True)
    for dirname in ["rgb", "depth", "gt_camera_poses"]:
        (output_path / dirname).mkdir(exist_ok=True)

    written = 0
    with rosbag.Bag(str(bag_path), "r") as bag:
        for index, (_, raw_msg, _) in enumerate(bag.read_messages(topics=[topic], raw=True)):
            if index % stride != 0:
                continue
            object_data = parse_object_data(raw_msg[1])
            stem = f"{written + 1:06d}"
            write_rgb(output_path / "rgb" / f"{stem}.png", object_data.rgb)
            write_depth(output_path / "depth" / f"{stem}.npy", object_data.depth)
            write_pose_matrix(
                output_path / "gt_camera_poses" / f"{stem}.txt",
                object_data.reference_camera_pose,
            )
            written += 1
            if max_frames > 0 and written >= max_frames:
                break

    write_yaml(
        output_path / "metadata.yaml",
        {
            "dataset_id": output_path.name,
            "source": "bonn_rgbd_object_tracking",
            "source_bag": str(bag_path),
            "mode": "camera_pose_validation_stub",
            "multi_view": written > 1,
            "object_prompt": object_prompt,
            "gt_camera_pose_dir": "gt_camera_poses",
            "notes": "GT camera poses are for validation overlay only, not RTAB-Map input.",
        },
    )
    write_yaml(
        output_path / "camera_intrinsics.yaml",
        {
            "width": 640,
            "height": 480,
            "fx": 525.0,
            "fy": 525.0,
            "cx": 319.5,
            "cy": 239.5,
            "depth_scale": 1.0,
            "depth_encoding": "32FC1",
            "frame_id": "openni_rgb_optical_frame",
            "notes": "Default Asus Xtion/OpenNI intrinsics; replace with calibrated values if available.",
        },
    )
    return written


def write_rgb(path: Path, image) -> None:
    array = np.frombuffer(image.data, dtype=np.uint8).reshape((image.height, image.width, 3))
    cv2.imwrite(str(path), cv2.cvtColor(array, cv2.COLOR_RGB2BGR))


def write_depth(path: Path, image) -> None:
    dtype = ">f4" if image.is_bigendian else "<f4"
    array = np.frombuffer(image.data, dtype=np.dtype(dtype)).reshape((image.height, image.width))
    np.save(path, array.astype(np.float32))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Extract Bonn ObjectData ROS1 bag into scan files.")
    parser.add_argument("bag_path", type=Path, help="Extracted Bonn .bag path.")
    parser.add_argument("output_path", type=Path, help="Output scan dataset path.")
    parser.add_argument("--topic", default="object_data", help="Bonn ObjectData topic.")
    parser.add_argument("--stride", type=int, default=10, help="Keep every Nth frame.")
    parser.add_argument("--max-frames", type=int, default=120, help="Maximum frames to write; 0 means no limit.")
    parser.add_argument("--object-prompt", default="box", help="Object prompt stored in metadata.")
    args = parser.parse_args(argv)

    if args.stride < 1:
        parser.error("--stride must be >= 1")

    count = extract_bonn_bag(
        args.bag_path,
        args.output_path,
        stride=args.stride,
        max_frames=args.max_frames,
        topic=args.topic,
        object_prompt=args.object_prompt,
    )
    print(f"Wrote {count} frames to {args.output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
