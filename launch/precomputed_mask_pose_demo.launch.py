from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("visual_grasp_manu")
    rviz_config = PathJoinSubstitution([package_share, "config", "precomputed_mask_pose_demo.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_path",
                default_value="outputs/datasets/object_scan_001_scan_blue_60_sam2",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("world_frame_id", default_value="odom"),
            DeclareLaunchArgument("rate_hz", default_value="6.0"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            Node(
                package="visual_grasp_manu",
                executable="precomputed_mask_pose_demo_node",
                name="precomputed_mask_pose_demo_node",
                output="screen",
                parameters=[
                    {
                        "dataset_path": LaunchConfiguration("dataset_path"),
                        "world_frame_id": LaunchConfiguration("world_frame_id"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "loop": LaunchConfiguration("loop"),
                    }
                ],
            ),
            LogInfo(msg=["Starting RViz with config: ", LaunchConfiguration("rviz_config")], condition=IfCondition(LaunchConfiguration("rviz"))),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_precomputed_mask_pose_demo",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
