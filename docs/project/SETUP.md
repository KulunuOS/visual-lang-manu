# Setup

## Requirements

- ROS 2 Humble is the current default in the dev container.
- Python package build uses `ament_python`.
- Runtime dependencies currently declared in `package.xml` include `rclpy`, `sensor_msgs`, `geometry_msgs`, and `std_msgs`.
- Model-specific dependencies are not selected yet and should be added only when a backend is implemented.
- GPU, CUDA, simulator, camera, or inference-server requirements must be documented here before they become required.

## Installation

Recommended path: use the dev container and build inside a ROS 2 workspace.

Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace
mkdir -p ros2_ws/src
ln -s /workspace/visual_grasp_manu ros2_ws/src/visual_grasp_manu
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If the workspace path, ROS 2 distribution, or container image changes, update this file and `README.md` in the same change.

## Running

Current launch scaffold:

```bash
ros2 launch visual_grasp_manu grasp_candidates.launch.py
```

The node entry point is a placeholder until the first pipeline contract is implemented.

## Data and Assets

Do not commit private datasets, raw recordings, model checkpoints, or generated artifacts by default.

For each required dataset or asset, document:

- source,
- license,
- expected local path or download command,
- preprocessing command,
- whether it is safe for public release.

## Environment Variables

Do not commit `.env` files. Document required variable names here without values.

## Dev Container

Development is expected to happen inside `.devcontainer/devcontainer.json`.

Update the dev container when adding:

- ROS 2 system dependencies,
- camera drivers,
- simulator libraries,
- CUDA or GPU runtime requirements,
- model-specific Python dependencies,
- external build tools.
