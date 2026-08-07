from visual_grasp_manu.transforms import PoseData, compose_pose, normalize_quaternion


def test_normalize_quaternion_returns_unit_quaternion():
    assert normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == (0.0, 0.0, 0.0, 1.0)


def test_compose_pose_translates_child_in_parent_frame():
    parent = PoseData(position=(1.0, 2.0, 3.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))
    child = PoseData(position=(0.1, 0.2, 0.3), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))

    composed = compose_pose(parent, child)

    assert composed.position == (1.1, 2.2, 3.3)
    assert composed.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
