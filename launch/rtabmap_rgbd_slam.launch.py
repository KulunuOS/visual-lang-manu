from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtabmap_launch_file = PathJoinSubstitution(
        [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"]
    )

    launch_arguments = {
        "rgb_topic": LaunchConfiguration("rgb_topic"),
        "depth_topic": LaunchConfiguration("depth_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "frame_id": LaunchConfiguration("frame_id"),
        "approx_sync": LaunchConfiguration("approx_sync"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
        "rviz": LaunchConfiguration("rviz"),
        "visual_odometry": LaunchConfiguration("visual_odometry"),
        "rtabmap_args": LaunchConfiguration("rtabmap_args"),
        "database_path": LaunchConfiguration("database_path"),
        "odom_always_process_most_recent_frame": LaunchConfiguration(
            "odom_always_process_most_recent_frame"
        ),
        "vo_frame_id": LaunchConfiguration("vo_frame_id"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/color/image_raw",
                description="RGB image topic from the RealSense bag or live camera.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
                description="Depth image topic aligned to the RGB camera.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/color/camera_info",
                description="CameraInfo topic for the RGB camera.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_color_optical_frame",
                description="Camera optical frame used by the RGB-D stream.",
            ),
            DeclareLaunchArgument(
                "approx_sync",
                default_value="true",
                description="Use approximate RGB/depth/camera-info synchronization.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock from rosbag playback.",
            ),
            DeclareLaunchArgument(
                "rtabmap_viz",
                default_value="false",
                description="Start RTAB-Map visualizer.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz from the RTAB-Map launch file.",
            ),
            DeclareLaunchArgument(
                "visual_odometry",
                default_value="true",
                description="Run RTAB-Map visual odometry for RGB-D trajectory estimation.",
            ),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value="--delete_db_on_start",
                description="Additional command-line arguments passed to rtabmap.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="/tmp/visual_grasp_manu/rtabmap.db",
                description="Writable RTAB-Map database path.",
            ),
            DeclareLaunchArgument(
                "odom_always_process_most_recent_frame",
                default_value="false",
                description="Process every replayed bag frame instead of dropping older frames.",
            ),
            DeclareLaunchArgument(
                "vo_frame_id",
                default_value="odom",
                description="TF frame id published by RTAB-Map RGB-D visual odometry.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
                launch_arguments=launch_arguments.items(),
            ),
        ]
    )
