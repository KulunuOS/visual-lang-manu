from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    recorder_node = Node(
        package="visual_grasp_manu",
        executable="scan_dataset_recorder_node",
        name="scan_dataset_recorder_node",
        output="screen",
        parameters=[
            {
                "output_path": LaunchConfiguration("output_path"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "object_prompt": LaunchConfiguration("object_prompt"),
                "source_bag": "",
                "pose_source": "robot_tf",
                "pose_source_mode": "tf",
                "world_frame_id": LaunchConfiguration("world_frame_id"),
                "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                "frame_stride": LaunchConfiguration("frame_stride"),
                "max_frames": LaunchConfiguration("max_frames"),
                "sync_tolerance_sec": LaunchConfiguration("sync_tolerance_sec"),
                "pose_tolerance_sec": LaunchConfiguration("pose_tolerance_sec"),
                "depth_scale": LaunchConfiguration("depth_scale"),
                "overwrite": LaunchConfiguration("overwrite"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_path",
                default_value="outputs/datasets/robot_object_scan_001",
                description="Output scan dataset directory.",
            ),
            DeclareLaunchArgument(
                "object_prompt",
                default_value="object",
                description="Text query for the object to reconstruct.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/color/image_raw",
                description="RGB image topic from the robot-mounted RGB-D camera.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
                description="Aligned depth topic from the robot-mounted RGB-D camera.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/color/camera_info",
                description="CameraInfo topic for the RGB camera intrinsics.",
            ),
            DeclareLaunchArgument(
                "world_frame_id",
                default_value="base_link",
                description="Robot/world frame used as the scan dataset world frame.",
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="",
                description="Camera optical frame. Empty uses CameraInfo header.frame_id.",
            ),
            DeclareLaunchArgument(
                "frame_stride",
                default_value="3",
                description="Record every Nth synchronized RGB-D frame while the robot moves.",
            ),
            DeclareLaunchArgument(
                "max_frames",
                default_value="120",
                description="Maximum scan frames to write. Use 0 for no limit.",
            ),
            DeclareLaunchArgument(
                "sync_tolerance_sec",
                default_value="0.04",
                description="Maximum RGB/depth/CameraInfo timestamp separation.",
            ),
            DeclareLaunchArgument(
                "pose_tolerance_sec",
                default_value="0.10",
                description="Maximum RGB/TF timestamp separation.",
            ),
            DeclareLaunchArgument(
                "depth_scale",
                default_value="0.001",
                description="Scale from stored depth units to meters.",
            ),
            DeclareLaunchArgument(
                "overwrite",
                default_value="true",
                description="Overwrite existing files in output scan directories.",
            ),
            recorder_node,
        ]
    )
