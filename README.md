# visual_grasp_manu

ROS 2 package for configurable visual grasp candidate generation from RGB-D sensor streams.

The package is intended to subscribe to one or more sensor streams, run a configurable perception and grasp-inference pipeline, and publish grasp candidates that a downstream manipulation stack can evaluate or execute.

The inference backend is intentionally pluggable. A pipeline may use direct RGB-D grasp generation, semantic segmentation, 6-DoF object pose localization, point-cloud processing, or model-specific inputs required by methods such as 6DoF-GraspNet or Contact-GraspNet.

## Repository Layout

```text
.
├── .devcontainer/
├── assets/
├── config/
├── docs/
│   ├── adr/
│   ├── project/
│   └── workflows/
├── launch/
├── outputs/
│   ├── datasets/
│   ├── logs/
│   └── videos/
├── resource/
├── scripts/
├── src/visual_grasp_manu/
├── tests/
├── AGENTS.md
├── package.xml
├── README.md
├── setup.py
└── .gitignore
```

## Installation

Development is expected to happen inside the dev container. Open the repository in a dev-container-capable editor and rebuild the container from `.devcontainer/devcontainer.json`.

Inside the container, create or enter a ROS 2 workspace and build the package:

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

## Configuration

The default pipeline configuration lives in `config/pipeline.yaml`.

The subscribed topics are model-dependent. For example:

- an RGB-D image model may need RGB, depth, and camera info topics;
- a point-cloud model may use a registered cloud topic instead of raw image topics;
- a semantic grasping pipeline may subscribe to segmentation masks or class labels;
- an object-centric pipeline may subscribe to 6-DoF object pose estimates before generating grasps.

Keep each backend selectable by parameters rather than hard-coding one sensor layout or model family.

## Demo

The launch file is currently a scaffold for the main node:

```bash
ros2 launch visual_grasp_manu grasp_candidates.launch.py
```

The first planned demo will replay a ROS 2 bag containing an object of interest, localize the object using a CAD-aware pose stage or pose-topic stub, generate grasp candidates, and publish RViz visualization markers.

Update this README whenever a runnable demo, model backend, required topic set, or example bag file is added.
