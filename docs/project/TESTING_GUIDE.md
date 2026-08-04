# Testing Guide

## Automated Checks

Run these from the ROS 2 workspace after sourcing the base ROS environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
python -m pytest
```

When ROS 2 tests are added, also run:

```bash
colcon test
colcon test-result --verbose
```

## Manual Checks

Current launch scaffold:

```bash
ros2 launch visual_grasp_manu grasp_candidates.launch.py
```

As the node is implemented, document manual checks for:

- required input topics for each backend,
- topic remapping examples,
- camera frame and target frame consistency,
- segmentation or object-pose prerequisites,
- expected grasp candidate output topic,
- latency and dropped-frame checks,
- debug image or marker visualization,
- simulator or recorded-bag demo commands.

## Public-Safety Check

Before committing:

```bash
git status --short
git diff --cached
```

Confirm that staged files contain no secrets, private paths, local machine metadata, raw datasets, checkpoints, or private workflow provenance.
