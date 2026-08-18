# Installation

This package targets ROS 2 Humble on Ubuntu 22.04 or a compatible container.
Build artifacts are expected to live under `/tmp/visual_grasp_manu`, keeping the
repository free of local build products.

## Local ROS 2 Workspace

From the repository root:

```bash
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon --log-base /tmp/visual_grasp_manu/log build \
  --symlink-install \
  --build-base /tmp/visual_grasp_manu/build \
  --install-base /tmp/visual_grasp_manu/install

source /tmp/visual_grasp_manu/install/setup.bash
```

Optional workspace shortcut:

```bash
export ws="$HOME/path/to/visual_grasp_manu"
cd "$ws"
```

Persist it for interactive shells:

```bash
echo 'export ws="$HOME/path/to/visual_grasp_manu"' >> ~/.bashrc
source ~/.bashrc
```

## Runtime Dependencies

Core runtime:

```bash
sudo apt update
sudo apt install \
  ros-humble-rtabmap-ros \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  python3-opencv \
  ffmpeg
```

Python packages used by the offline mesh and preview tools:

```bash
python3 -m pip install open3d pyyaml numpy
```

Grounding DINO and SAM2 are optional model backends. Their repositories and
checkpoints are not committed to this repository; keep them in an external cache
such as `/tmp/visual_grasp_manu/model_repos` and
`/tmp/visual_grasp_manu/checkpoints`.

## Apptainer

For GPU-capable systems:

```bash
apptainer build visual_grasp_manu.sif containers/apptainer/visual_grasp_manu.def
apptainer shell --nv --bind "$PWD":/workspace/visual_grasp_manu visual_grasp_manu.sif

cd /workspace/visual_grasp_manu
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/visual_grasp_manu/log build \
  --symlink-install \
  --build-base /tmp/visual_grasp_manu/build \
  --install-base /tmp/visual_grasp_manu/install
source /tmp/visual_grasp_manu/install/setup.bash
```

## Tests

```bash
source /opt/ros/humble/setup.bash
source /tmp/visual_grasp_manu/install/setup.bash
python -m pytest
```
