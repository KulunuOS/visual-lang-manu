from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_grasp_manu")
    rtabmap_launch_file = PathJoinSubstitution(
        [package_share, "launch", "rtabmap_rgbd_slam.launch.py"]
    )

    run_rtabmap = LaunchConfiguration("run_rtabmap")
    start_rviz = LaunchConfiguration("start_rviz")
    publish_debug_cloud = LaunchConfiguration("publish_debug_cloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_rtabmap",
                default_value="true",
                description="Run the project RTAB-Map RGB-D SLAM wrapper.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Compatibility alias for start_rviz.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value=LaunchConfiguration("rviz"),
                description="Start RViz with camera-pose overlay displays.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "camera_pose_overlay.rviz"]
                ),
                description="RViz config file for camera-pose visualization.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="map",
                description="Fixed frame for RViz paths and camera markers.",
            ),
            DeclareLaunchArgument(
                "inferred_odom_topic",
                default_value="/odom",
                description="Inferred camera odometry topic, typically from RTAB-Map odometry.",
            ),
            DeclareLaunchArgument(
                "inferred_pose_topic",
                default_value="",
                description="Optional inferred PoseStamped topic.",
            ),
            DeclareLaunchArgument(
                "gt_pose_topic",
                default_value="/visual_grasp_manu/gt_camera_pose",
                description="Ground-truth camera PoseStamped topic for overlay.",
            ),
            DeclareLaunchArgument(
                "align_inferred_to_gt",
                default_value="true",
                description="Anchor inferred odometry path to the first GT pose for overlay visualization.",
            ),
            DeclareLaunchArgument(
                "frustum_scale",
                default_value="0.045",
                description="Camera frustum marker size in meters.",
            ),
            DeclareLaunchArgument(
                "frustum_stride",
                default_value="75",
                description="Publish one sampled camera frustum every N poses, plus the current pose.",
            ),
            DeclareLaunchArgument(
                "path_line_width",
                default_value="0.006",
                description="Camera path marker line width in meters.",
            ),
            DeclareLaunchArgument(
                "frustum_line_width",
                default_value="0.002",
                description="Camera frustum marker line width in meters.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/color/image_raw",
                description="RGB image topic for RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
                description="Depth image topic aligned to RGB for RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/color/camera_info",
                description="CameraInfo topic for RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_color_optical_frame",
                description="Camera optical frame for RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "approx_sync",
                default_value="true",
                description="Use approximate sync in RTAB-Map.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use rosbag /clock.",
            ),
            DeclareLaunchArgument(
                "rtabmap_viz",
                default_value="false",
                description="Start RTAB-Map visualizer.",
            ),
            DeclareLaunchArgument(
                "visual_odometry",
                default_value="true",
                description="Run RTAB-Map visual odometry.",
            ),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value="--delete_db_on_start",
                description="Additional RTAB-Map arguments.",
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
                "publish_debug_cloud",
                default_value="false",
                description="Publish a dense RGB-D debug cloud from RGB, aligned depth, and CameraInfo.",
            ),
            DeclareLaunchArgument(
                "debug_cloud_topic",
                default_value="/visual_grasp_manu/debug/rgbd_cloud",
                description="Output topic for the dense RGB-D debug cloud.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
                condition=IfCondition(run_rtabmap),
                launch_arguments={
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "approx_sync": LaunchConfiguration("approx_sync"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                    "rviz": "false",
                    "visual_odometry": LaunchConfiguration("visual_odometry"),
                    "rtabmap_args": LaunchConfiguration("rtabmap_args"),
                    "database_path": LaunchConfiguration("database_path"),
                    "odom_always_process_most_recent_frame": LaunchConfiguration(
                        "odom_always_process_most_recent_frame"
                    ),
                }.items(),
            ),
            Node(
                package="rtabmap_util",
                executable="point_cloud_xyzrgb",
                name="visual_grasp_manu_debug_rgbd_cloud",
                output="screen",
                parameters=[
                    {
                        "decimation": 1,
                        "voxel_size": 0.0,
                        "approx_sync": LaunchConfiguration("approx_sync"),
                        "qos": 0,
                        "qos_camera_info": 0,
                    }
                ],
                remappings=[
                    ("rgb/image", LaunchConfiguration("rgb_topic")),
                    ("depth/image", LaunchConfiguration("depth_topic")),
                    ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
                    ("cloud", LaunchConfiguration("debug_cloud_topic")),
                ],
                condition=IfCondition(publish_debug_cloud),
            ),
            Node(
                package="visual_grasp_manu",
                executable="camera_pose_visualizer_node",
                name="camera_pose_visualizer_node",
                output="screen",
                parameters=[
                    {
                        "fixed_frame": LaunchConfiguration("fixed_frame"),
                        "inferred_odom_topic": LaunchConfiguration("inferred_odom_topic"),
                        "inferred_pose_topic": LaunchConfiguration("inferred_pose_topic"),
                        "gt_pose_topic": LaunchConfiguration("gt_pose_topic"),
                        "align_inferred_to_gt": LaunchConfiguration("align_inferred_to_gt"),
                        "frustum_scale": LaunchConfiguration("frustum_scale"),
                        "frustum_stride": LaunchConfiguration("frustum_stride"),
                        "path_line_width": LaunchConfiguration("path_line_width"),
                        "frustum_line_width": LaunchConfiguration("frustum_line_width"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_camera_pose_overlay",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
