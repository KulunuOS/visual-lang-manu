from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_grasp_manu")
    rtabmap_launch_file = PathJoinSubstitution(
        [package_share, "launch", "rtabmap_rgbd_slam.launch.py"]
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, "config", "live_interactive_mask_mesh.rviz"]
    )

    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    loop_bag = LaunchConfiguration("loop_bag")
    publish_clock = LaunchConfiguration("publish_clock")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    frame_id = LaunchConfiguration("frame_id")
    odom_topic = LaunchConfiguration("odom_topic")
    fixed_frame = LaunchConfiguration("fixed_frame")
    tracking_frame = LaunchConfiguration("tracking_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path", default_value="outputs/datasets/object_scan_001"),
            DeclareLaunchArgument("bag_rate", default_value="0.5"),
            DeclareLaunchArgument("loop_bag", default_value="true"),
            DeclareLaunchArgument("publish_clock", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("fixed_frame", default_value="odom"),
            DeclareLaunchArgument("tracking_frame", default_value="odom"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/ee_cam/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/ee_cam/aligned_depth_to_color/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/ee_cam/color/camera_info"),
            DeclareLaunchArgument("frame_id", default_value="ee_cam_color_optical_frame"),
            DeclareLaunchArgument("odom_topic", default_value="/rtabmap/odom"),
            DeclareLaunchArgument("output_path", default_value="outputs/datasets/object_scan_live_interactive"),
            DeclareLaunchArgument("object_prompt", default_value=""),
            DeclareLaunchArgument("backend", default_value="grounding_dino_sam2"),
            DeclareLaunchArgument("start_interactive_node", default_value="true"),
            DeclareLaunchArgument("max_frames", default_value="60"),
            DeclareLaunchArgument("frame_stride", default_value="10"),
            DeclareLaunchArgument("auto_accept_initial", default_value="false"),
            DeclareLaunchArgument("database_path", default_value="/tmp/visual_grasp_manu/rtabmap_live_interactive.db"),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate, "--loop"],
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                condition=IfCondition(
                    PythonExpression(["'", loop_bag, "' == 'true' and '", publish_clock, "' == 'true'"])
                ),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--rate", bag_rate, "--loop"],
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                condition=IfCondition(
                    PythonExpression(["'", loop_bag, "' == 'true' and '", publish_clock, "' != 'true'"])
                ),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", loop_bag, "' != 'true' and '", publish_clock, "' == 'true'"])
                ),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--rate", bag_rate],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", loop_bag, "' != 'true' and '", publish_clock, "' != 'true'"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
                launch_arguments={
                    "rgb_topic": rgb_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "frame_id": frame_id,
                    "approx_sync": "true",
                    "use_sim_time": use_sim_time,
                    "rtabmap_viz": "false",
                    "rviz": "false",
                    "visual_odometry": "true",
                    "rtabmap_args": "--delete_db_on_start",
                    "database_path": LaunchConfiguration("database_path"),
                    "odom_always_process_most_recent_frame": "false",
                }.items(),
            ),
            Node(
                package="rtabmap_util",
                executable="point_cloud_xyzrgb",
                name="visual_grasp_manu_live_rgbd_cloud",
                output="screen",
                parameters=[
                    {
                        "decimation": 1,
                        "voxel_size": 0.0,
                        "approx_sync": True,
                        "qos": 0,
                        "qos_camera_info": 0,
                    }
                ],
                remappings=[
                    ("rgb/image", rgb_topic),
                    ("depth/image", depth_topic),
                    ("rgb/camera_info", camera_info_topic),
                    ("cloud", "/visual_grasp_manu/live/rgbd_cloud"),
                ],
            ),
            Node(
                package="visual_grasp_manu",
                executable="camera_pose_visualizer_node",
                name="initial_camera_frustum_node",
                output="screen",
                parameters=[
                    {
                        "fixed_frame": tracking_frame,
                        "inferred_odom_topic": odom_topic,
                        "gt_pose_topic": "",
                        "inferred_path_topic": "/visual_grasp_manu/unused_camera_path",
                        "marker_topic": "/visual_grasp_manu/initial_camera_frustum",
                        "align_inferred_to_gt": False,
                        "publish_path_marker": False,
                        "frustum_mode": "initial",
                        "frustum_scale": 0.035,
                        "frustum_line_width": 0.0015,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="visual_grasp_manu",
                executable="live_interactive_mask_mesh_node",
                name="live_interactive_mask_mesh_node",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_interactive_node")),
                parameters=[
                    {
                        "output_path": LaunchConfiguration("output_path"),
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "odom_topic": odom_topic,
                        "world_frame_id": tracking_frame,
                        "object_prompt": LaunchConfiguration("object_prompt"),
                        "backend": LaunchConfiguration("backend"),
                        "max_frames": LaunchConfiguration("max_frames"),
                        "frame_stride": LaunchConfiguration("frame_stride"),
                        "auto_accept_initial": LaunchConfiguration("auto_accept_initial"),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            LogInfo(
                msg=["Starting RViz with config: ", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_live_interactive_mask_mesh",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
