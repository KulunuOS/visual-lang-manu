from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("visual_grasp_manu"), "config", "pipeline.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="visual_grasp_manu",
                executable="grasp_candidate_node",
                name="grasp_candidate_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
