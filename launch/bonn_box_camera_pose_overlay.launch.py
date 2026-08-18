from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_grasp_manu")
    overlay_launch = PathJoinSubstitution([package_share, "launch", "camera_pose_overlay.launch.py"])
    rviz_config = PathJoinSubstitution([package_share, "config", "camera_pose_overlay.rviz"])

    scan_path = LaunchConfiguration("scan_path")
    run_replay = LaunchConfiguration("run_replay")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scan_path",
                default_value="outputs/datasets/bonn_box_slow_scan",
                description="Extracted Bonn scan dataset path.",
            ),
            DeclareLaunchArgument(
                "run_replay",
                default_value="true",
                description="Run the Bonn scan replay node.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="2.0",
                description="Replay rate for extracted Bonn RGB-D frames.",
            ),
            DeclareLaunchArgument(
                "pointcloud_stride",
                default_value="20",
                description="Depth sampling stride for the RViz GT-frame point cloud.",
            ),
            DeclareLaunchArgument(
                "accumulate_pointcloud",
                default_value="true",
                description="Accumulate replayed depth clouds in map frame for stable RViz scene visualization.",
            ),
            DeclareLaunchArgument(
                "invert_reference_camera_pose",
                default_value="false",
                description="Invert Bonn reference_camera_pose before applying optical calibration.",
            ),
            DeclareLaunchArgument(
                "invert_optical_calibration",
                default_value="false",
                description="Invert the documented Bonn optical-to-mocap calibration transform.",
            ),
            DeclareLaunchArgument(
                "run_rtabmap",
                default_value="true",
                description="Run RTAB-Map on the replayed RGB-D stream.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Compatibility alias for start_rviz.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value=LaunchConfiguration("rviz"),
                description="Start RViz with point cloud and trajectory displays.",
            ),
            Node(
                package="visual_grasp_manu",
                executable="bonn_scan_replay_node",
                name="bonn_scan_replay_node",
                output="screen",
                parameters=[
                    {
                        "scan_path": scan_path,
                        "rgb_topic": "/camera/color/image_raw",
                        "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                        "camera_info_topic": "/camera/color/camera_info",
                        "gt_pose_topic": "/visual_grasp_manu/gt_camera_pose",
                        "pointcloud_topic": "/visual_grasp_manu/bonn/points_gt_map",
                        "camera_frame_id": "openni_rgb_optical_frame",
                        "world_frame_id": "map",
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "pointcloud_stride": LaunchConfiguration("pointcloud_stride"),
                        "accumulate_pointcloud": LaunchConfiguration("accumulate_pointcloud"),
                        "invert_reference_camera_pose": LaunchConfiguration(
                            "invert_reference_camera_pose"
                        ),
                        "invert_optical_calibration": LaunchConfiguration(
                            "invert_optical_calibration"
                        ),
                    }
                ],
                condition=IfCondition(run_replay),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(overlay_launch),
                launch_arguments={
                    "run_rtabmap": LaunchConfiguration("run_rtabmap"),
                    "rviz": "false",
                    "fixed_frame": "map",
                    "inferred_odom_topic": "/odom",
                    "gt_pose_topic": "/visual_grasp_manu/gt_camera_pose",
                    "align_inferred_to_gt": "true",
                    "rgb_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "frame_id": "openni_rgb_optical_frame",
                    "approx_sync": "false",
                    "use_sim_time": "false",
                    "database_path": "/tmp/visual_grasp_manu/rtabmap_bonn_box.db",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_bonn_camera_pose_overlay",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
