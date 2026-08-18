from visual_grasp_manu.grasp_candidate_node import contact_graspnet_gripper_polyline
from visual_grasp_manu.transforms import (
    PoseData,
    compose_pose,
    inverse_pose,
    normalize_quaternion,
    rotate_vector,
)


def test_normalize_quaternion_returns_unit_quaternion():
    assert normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == (0.0, 0.0, 0.0, 1.0)


def test_compose_pose_translates_child_in_parent_frame():
    parent = PoseData(position=(1.0, 2.0, 3.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))
    child = PoseData(position=(0.1, 0.2, 0.3), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))

    composed = compose_pose(parent, child)

    assert composed.position == (1.1, 2.2, 3.3)
    assert composed.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_inverse_pose_composes_to_identity():
    pose = PoseData(position=(1.0, 2.0, 3.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))

    identity = compose_pose(inverse_pose(pose), pose)

    assert identity.position == (0.0, 0.0, 0.0)
    assert identity.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_contact_graspnet_gripper_polyline_uses_seven_wireframe_points():
    grasp = PoseData(position=(1.0, 2.0, 3.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))

    points = contact_graspnet_gripper_polyline(
        grasp_pose=grasp,
        gripper_width=0.08,
        gripper_depth=0.08,
        approach_length=0.06,
    )

    assert points == [
        (0.94, 2.0, 3.0),
        (1.0, 2.0, 3.0),
        (1.0, 2.04, 3.0),
        (1.08, 2.04, 3.0),
        (1.0, 2.04, 3.0),
        (1.0, 1.96, 3.0),
        (1.08, 1.96, 3.0),
    ]


def test_demo_side_grasps_point_toward_object_center():
    side_grasps = [
        PoseData(position=(0.08, 0.0, 0.0), orientation_xyzw=(0.0, 0.0, 1.0, 0.0)),
        PoseData(position=(-0.08, 0.0, 0.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0)),
        PoseData(position=(0.0, 0.08, 0.0), orientation_xyzw=(0.0, 0.0, -0.7071068, 0.7071068)),
        PoseData(position=(0.0, -0.08, 0.0), orientation_xyzw=(0.0, 0.0, 0.7071068, 0.7071068)),
    ]

    for grasp in side_grasps:
        depth_direction = rotate_vector(grasp.orientation_xyzw, (1.0, 0.0, 0.0))
        toward_object = tuple(-component for component in grasp.position)
        assert sum(depth_direction[index] * toward_object[index] for index in range(3)) > 0.0
