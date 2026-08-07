from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_grasp_manu")
    pose_config = PathJoinSubstitution([package_share, "config", "pose_stub.yaml"])
    grasp_config = PathJoinSubstitution([package_share, "config", "grasp_demo.yaml"])
    grasp_library = PathJoinSubstitution([package_share, "config", "demo_grasps.yaml"])

    return LaunchDescription(
        [
            Node(
                package="visual_grasp_manu",
                executable="pose_stub_node",
                name="pose_stub_node",
                output="screen",
                parameters=[pose_config],
            ),
            Node(
                package="visual_grasp_manu",
                executable="grasp_candidate_node",
                name="grasp_candidate_node",
                output="screen",
                parameters=[
                    grasp_config,
                    {"grasp_library_path": grasp_library},
                ],
            ),
        ]
    )
