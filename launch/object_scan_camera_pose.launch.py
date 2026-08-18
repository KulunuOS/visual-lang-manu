from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_grasp_manu")
    overlay_launch = PathJoinSubstitution([package_share, "launch", "camera_pose_overlay.launch.py"])
    rviz_config = PathJoinSubstitution([package_share, "config", "object_scan_camera_pose.rviz"])

    bag_path = LaunchConfiguration("bag_path")
    play_bag = LaunchConfiguration("play_bag")
    bag_rate = LaunchConfiguration("bag_rate")

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
                description="Replay the bag with /clock before running RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="0.5",
                description="Replay rate. Slower playback helps RTAB-Map on first tests.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with camera-pose overlay displays.",
            ),
            DeclareLaunchArgument(
                "run_rtabmap",
                default_value="true",
                description="Run RTAB-Map RGB-D SLAM on the RealSense bag topics.",
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
                "database_path",
                default_value="/tmp/visual_grasp_manu/rtabmap_object_scan_001.db",
                description="Writable RTAB-Map database path for this object scan.",
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate],
                output="screen",
                condition=IfCondition(play_bag),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(overlay_launch),
                launch_arguments={
                    "run_rtabmap": LaunchConfiguration("run_rtabmap"),
                    "start_rviz": LaunchConfiguration("rviz"),
                    "rviz_config": rviz_config,
                    "publish_debug_cloud": "true",
                    "debug_cloud_topic": "/visual_grasp_manu/debug/rgbd_cloud",
                    "fixed_frame": "map",
                    "inferred_odom_topic": "/odom",
                    "gt_pose_topic": "",
                    "align_inferred_to_gt": "false",
                    "frustum_scale": "0.035",
                    "frustum_stride": "120",
                    "path_line_width": "0.004",
                    "frustum_line_width": "0.0015",
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "approx_sync": "true",
                    "use_sim_time": "true",
                    "rtabmap_viz": "false",
                    "visual_odometry": "true",
                    "rtabmap_args": "--delete_db_on_start",
                    "database_path": LaunchConfiguration("database_path"),
                    "odom_always_process_most_recent_frame": "false",
                }.items(),
            ),
        ]
    )
