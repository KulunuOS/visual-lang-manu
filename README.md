# Visual-Language Manipulation Prototype

This repository is an early prototype of a visual-language RGB-D perception
pipeline for robotic manipulation. The long-term goal is a real-time system
where a robot receives a language query, observes the scene with a mounted RGB-D
camera, segments the referred object, reconstructs object geometry, estimates
pose, and produces grasp candidates for manipulation.

The current implementation is not yet that final online system. It uses offline
and precomputed stages around a recorded RealSense ROS 2 bag: camera motion is
estimated from RGB-D data, object masks are generated ahead of playback,
selected masked frames are fused into a mesh, and the intermediate results are
visualized in RViz.

For setup instructions, see [INSTALLATION.md](INSTALLATION.md).
For demo replication commands, see [DEMO.md](DEMO.md).

## Demo Overview

The demonstration starts with a recorded RGB-D scan. The target task is to
separate one object from a cluttered tabletop, recover the camera trajectory, and
produce an object mesh that can be passed to later pose-estimation and grasp
generation stages.

**Input RGB frames from the ROS bag**

![RGB frames from the object scan](assets/demo/rosbag_rgb_input.gif)

**RGB-D camera odometry**

RTAB-Map RGB-D odometry estimates the camera motion from the bag. The RViz view
shows the reconstructed scene cloud, the inferred camera path in green, and
sampled camera frustums in blue/cyan.

![Camera pose visualization in RViz](assets/demo/camera_pose_rviz.gif)

**Object mask overlay**

In this prototype, the object masks are generated offline with Grounding DINO
and SAM2 using the text query `blue object`. During RViz playback, the replay
node publishes the saved mask overlay image, the masked object cloud, camera
pose markers, and the generated mesh marker at a stable playback rate.

![Mask overlay visualization in RViz](assets/demo/mask_overlay_rviz.gif)

**Reconstructed object mesh**

The masked RGB-D frames are fused into a compact object mesh:

[Open generated object mesh](assets/demo/generated_object_mesh.stl)

Local mesh inspection commands are listed in [DEMO.md](DEMO.md).

## Prototype Pipeline

```text
Recorded ROS 2 RGB-D bag
-> RGB/depth/camera-info extraction
-> RTAB-Map RGB-D camera odometry
-> text-conditioned object masking with Grounding DINO + SAM2
-> mask tracking over selected frames
-> object-only TSDF fusion with Open3D
-> mesh and visual grasp output visualization in RViz
```

This offline prototype keeps each stage inspectable. Live model inference is
available through the mask generation scripts and interactive ROS node, but the
real-time version is still under development.

## Repository Layout

```text
.
├── assets/demo/              # demo media and mesh artifacts
├── config/                   # RViz and node configuration
├── containers/               # Apptainer definition
├── launch/                   # ROS 2 launch files
├── scripts/                  # command-line wrappers
├── src/visual_grasp_manu/    # package implementation
├── tests/                    # unit and smoke tests
├── INSTALLATION.md
├── README.md
├── package.xml
└── setup.py
```

## Dataset

Offline mask, mesh, and replay stages use this scan dataset layout:

```text
scan_001/
├── rgb/000001.png
├── depth/000001.npy
├── masks/000001.png
├── mask_overlays/000001.png
├── camera_poses/000001.txt
├── camera_intrinsics.yaml
├── metadata.yaml
└── mesh/object.ply
```

Validation commands are listed in [DEMO.md](DEMO.md).

## Mask And Mesh Generation tools

The package includes command-line tools for text-conditioned mask generation,
mask preview, TSDF mesh fusion, mesh inspection, and dataset validation. See
[DEMO.md](DEMO.md) for the current commands used to reproduce the prototype
demonstration.

## Testing

```bash
source /opt/ros/humble/setup.bash
source /tmp/visual_grasp_manu/install/setup.bash
python -m pytest
```
