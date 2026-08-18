# visual_grasp_manu

ROS 2 package for visual object masking, RGB-D reconstruction, and grasp
candidate visualization.

The package is structured around a modular RGB-D perception pipeline:

```text
RGB-D input
-> camera trajectory estimation
-> object mask proposal and tracking
-> object-only TSDF mesh generation
-> grasp marker visualization
```

The current implementation supports recorded ROS 2 bags and local smoke-test
masking for a tabletop object scan. The intended next step is a reviewable
operator interface for approving the first mask and tracking quality before mesh
generation.

## Repository Layout

```text
.
├── .devcontainer/
├── assets/
├── config/
├── containers/
├── launch/
├── outputs/
│   ├── datasets/
│   ├── logs/
│   └── videos/
├── resource/
├── scripts/
├── src/visual_grasp_manu/
├── tests/
├── colcon_defaults.yaml
├── package.xml
├── README.md
├── setup.cfg
└── setup.py
```

Large bags, model checkpoints, generated datasets, meshes, videos, and logs are
ignored by default. Keep demo media small and place reviewed public examples
under `assets/demo/`.

## Environment

The project targets ROS 2 Humble.

Build from the repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source /tmp/visual_grasp_manu/install/setup.bash
```

`colcon_defaults.yaml` sends `build/`, `install/`, and `log/` to
`/tmp/visual_grasp_manu/` so build artifacts do not live in the repository.

For Apptainer-based GPU systems:

```bash
apptainer build visual_grasp_manu.sif containers/apptainer/visual_grasp_manu.def
apptainer shell --nv --bind "$PWD":/workspace/visual_grasp_manu visual_grasp_manu.sif
cd /workspace/visual_grasp_manu
source /opt/ros/humble/setup.bash
colcon build
source /tmp/visual_grasp_manu/install/setup.bash
```

## Current Demo Inputs

The demo expects a ROS 2 SQLite bag directory containing RGB, aligned depth, and
camera info topics:

```text
/camera/ee_cam/color/image_raw
/camera/ee_cam/aligned_depth_to_color/image_raw
/camera/ee_cam/color/camera_info
```

If you have only a loose `.db3` file, prepare a playable bag directory:

```bash
ros2 run visual_grasp_manu prepare_rosbag2_sqlite \
  outputs/datasets/object_scan_001_0.db3 \
  outputs/datasets/object_scan_001 \
  --force
```

The raw `.db3` should remain outside Git. Publish it separately as a release
asset or dataset download if needed.

## RViz Camera Pose Demo

Launch the current RealSense object-scan visualization:

```bash
source /opt/ros/humble/setup.bash
source /tmp/visual_grasp_manu/install/setup.bash

ros2 launch visual_grasp_manu object_scan_camera_pose.launch.py \
  bag_path:=outputs/datasets/object_scan_001 \
  rviz:=true
```

Expected RViz result:

- dense RGB-D tabletop cloud on `/visual_grasp_manu/debug/rgbd_cloud`,
- RTAB-Map RGB-D odometry trajectory,
- green camera path,
- thin blue/cyan camera frustums.

This visualization is the best candidate for a public GIF such as:

```text
assets/demo/camera_pose_rviz.gif
```

## Interactive Mask And Mesh Demo

Terminal 1 starts bag playback, RTAB-Map, point-cloud visualization, and RViz:

```bash
source /opt/ros/humble/setup.bash
source /tmp/visual_grasp_manu/install/setup.bash

ROS_LOG_DIR=/tmp/visual_grasp_manu/log/live_interactive \
ros2 launch visual_grasp_manu live_interactive_mask_mesh_demo.launch.py \
  bag_path:=outputs/datasets/object_scan_001 \
  loop_bag:=false \
  start_rviz:=true \
  start_interactive_node:=false \
  publish_clock:=true \
  use_sim_time:=true
```

Terminal 2 runs the mask and mesh node:

```bash
source /opt/ros/humble/setup.bash
source /tmp/visual_grasp_manu/install/setup.bash

ROS_LOG_DIR=/tmp/visual_grasp_manu/log/live_interactive_node \
ros2 run visual_grasp_manu live_interactive_mask_mesh_node --ros-args \
  -p use_sim_time:=true \
  -p output_path:=outputs/datasets/object_scan_live_interactive \
  -p backend:=hsv_color \
  -p object_prompt:="blue object" \
  -p auto_accept_initial:=false \
  -p max_frames:=60 \
  -p frame_stride:=5
```

The live overlay topics are:

```text
/visual_grasp_manu/mask_overlay/image
/visual_grasp_manu/mask_overlay/cloud
/visual_grasp_manu/mask_overlay/markers
/visual_grasp_manu/object_mesh_marker
```

For the current bag, use `hsv_color` for a fast local mask smoke test. The
Grounding DINO + SAM2 backend is wired through the CLI/node interface, but it
requires external model repositories and checkpoints in the active environment.

Use `loop_bag:=false` with RTAB-Map. RViz and the RGB-D cloud require bag
`/clock` plus `use_sim_time:=true`; looped bag playback restarts timestamps, so
RTAB-Map rejects the second pass as non-monotonic input.

Recommended public demo artifacts:

```text
assets/demo/mask_overlay_blue_object.png
assets/demo/mask_tracking_preview.gif
assets/demo/generated_mesh_preview.png
assets/demo/object_mesh.ply
```

Only include `object_mesh.ply` if it is small and reviewed.

## Scan Dataset Contract

Offline mask, mesh, and grasp stages use this layout:

```text
scan_001/
├── rgb/000001.png
├── depth/000001.npy
├── masks/000001.png
├── camera_poses/000001.txt
├── camera_intrinsics.yaml
├── metadata.yaml
└── mesh/object.ply
```

Validate a capture:

```bash
ros2 run visual_grasp_manu validate_scan_dataset outputs/datasets/scan_001
```

Validate stricter stages:

```bash
ros2 run visual_grasp_manu validate_scan_dataset outputs/datasets/scan_001 --stage masks
ros2 run visual_grasp_manu validate_scan_dataset outputs/datasets/scan_001 --stage mesh
```

## Mask Generation

Generate masks from a scan dataset:

```bash
ros2 run visual_grasp_manu generate_grounded_sam2_masks \
  outputs/datasets/object_scan_001_scan \
  --prompt "blue object" \
  --grounding-config /path/to/GroundingDINO_SwinT_OGC.py \
  --grounding-checkpoint /path/to/groundingdino_swint_ogc.pth \
  --sam2-config configs/sam2.1/sam2.1_hiera_t.yaml \
  --sam2-checkpoint /path/to/sam2.1_hiera_tiny.pt \
  --device cuda \
  --grounding-device cpu \
  --save-overlays
```

For color-isolated local tests:

```bash
ros2 run visual_grasp_manu generate_grounded_sam2_masks \
  outputs/datasets/object_scan_001_scan \
  --backend hsv_color \
  --prompt "blue object" \
  --overwrite \
  --save-overlays \
  --hsv-lower 90,90,25 \
  --hsv-upper 135,255,255
```

Preview masks:

```bash
ros2 run visual_grasp_manu preview_scan_masks \
  outputs/datasets/object_scan_001_scan \
  --columns 1 \
  --tile-width 320
```

## Mesh Generation

Fuse reviewed masked RGB-D frames into an object mesh:

```bash
ros2 run visual_grasp_manu generate_tsdf_mesh \
  outputs/datasets/object_scan_001_scan \
  --voxel-length 0.004 \
  --sdf-trunc 0.02 \
  --depth-trunc 1.5
```

Inspect the mesh:

```bash
ros2 run visual_grasp_manu visualize_mesh outputs/datasets/object_scan_001_scan
```

Headless mesh check:

```bash
ros2 run visual_grasp_manu visualize_mesh \
  outputs/datasets/object_scan_001_scan \
  --no-view
```

## Pose Stub Grasp Demo

The package also includes a simple grasp-marker visualization:

```bash
ros2 launch visual_grasp_manu pose_stub_grasp_demo.launch.py rviz:=true
```

Expected result:

- blue object marker in `world`,
- Contact-GraspNet-style wireframe grasp handles,
- green-to-red score coloring.

## Tests

```bash
source /opt/ros/humble/setup.bash
colcon build
source /tmp/visual_grasp_manu/install/setup.bash
python -m pytest
```
