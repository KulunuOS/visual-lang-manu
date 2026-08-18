from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_grasp_manu")
    rtabmap_launch_file = PathJoinSubstitution(
        [package_share, "launch", "rtabmap_rgbd_slam.launch.py"]
    )

    bag_path = LaunchConfiguration("bag_path")
    play_bag = LaunchConfiguration("play_bag")
    bag_rate = LaunchConfiguration("bag_rate")
    loop_bag = LaunchConfiguration("loop_bag")

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
                "odom_topic": LaunchConfiguration("odom_topic"),
                "object_prompt": LaunchConfiguration("object_prompt"),
                "source_bag": LaunchConfiguration("bag_path"),
                "pose_source": "rtabmap_rgbd_slam",
                "world_frame_id": "odom",
                "frame_stride": LaunchConfiguration("frame_stride"),
                "max_frames": LaunchConfiguration("max_frames"),
                "sync_tolerance_sec": LaunchConfiguration("sync_tolerance_sec"),
                "pose_tolerance_sec": LaunchConfiguration("pose_tolerance_sec"),
                "depth_scale": 0.001,
                "overwrite": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="outputs/datasets/object_scan_001",
                description="Playable ROS 2 bag directory for the RealSense object scan.",
            ),
            DeclareLaunchArgument(
                "play_bag",
                default_value="true",
                description="Replay the bag with /clock while exporting the scan dataset.",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="0.5",
                description="Replay rate. Slower playback gives RTAB-Map and export time to process.",
            ),
            DeclareLaunchArgument(
                "loop_bag",
                default_value="false",
                description="Loop rosbag playback until the recorder reaches max_frames.",
            ),
            DeclareLaunchArgument(
                "output_path",
                default_value="outputs/datasets/object_scan_001_scan",
                description="Output scan dataset directory.",
            ),
            DeclareLaunchArgument(
                "object_prompt",
                default_value="object",
                description="Text prompt recorded in metadata.yaml for later mask generation.",
            ),
            DeclareLaunchArgument(
                "frame_stride",
                default_value="10",
                description="Write every Nth synchronized RGB-D frame.",
            ),
            DeclareLaunchArgument(
                "max_frames",
                default_value="120",
                description="Maximum scan frames to write. Use 0 for no limit.",
            ),
            DeclareLaunchArgument(
                "sync_tolerance_sec",
                default_value="0.06",
                description="Maximum RGB/depth/CameraInfo timestamp separation.",
            ),
            DeclareLaunchArgument(
                "pose_tolerance_sec",
                default_value="0.20",
                description="Maximum RGB/odometry timestamp separation.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="ee_cam_color_optical_frame",
                description="Camera optical frame used by the RealSense recording.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/ee_cam/color/image_raw",
                description="RGB image topic in the RealSense object scan bag.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/ee_cam/aligned_depth_to_color/image_raw",
                description="Aligned depth image topic in the RealSense object scan bag.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/ee_cam/color/camera_info",
                description="RGB CameraInfo topic in the RealSense object scan bag.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/rtabmap/odom",
                description="RTAB-Map odometry topic to export as camera poses.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/visual_grasp_manu/rtabmap_object_scan_001_export.db",
                description="Writable RTAB-Map database path for this export run.",
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate, "--loop"],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", play_bag, "' == 'true' and '", loop_bag, "' == 'true'"])
                ),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", play_bag, "' == 'true' and '", loop_bag, "' != 'true'"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
                launch_arguments={
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "approx_sync": "true",
                    "use_sim_time": "true",
                    "rtabmap_viz": "false",
                    "rviz": "false",
                    "visual_odometry": "true",
                    "rtabmap_args": "--delete_db_on_start",
                    "database_path": LaunchConfiguration("database_path"),
                    "odom_always_process_most_recent_frame": "false",
                }.items(),
            ),
            recorder_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=recorder_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="scan dataset recorder completed"
                            )
                        )
                    ],
                )
            ),
        ]
    )
