# Demo Replication

These commands reproduce the current offline prototype demonstration from a ROS
2 RGB-D bag and a precomputed scan dataset.

Build and source the package first. See [INSTALLATION.md](INSTALLATION.md) for
setup details.

```bash
source /opt/ros/humble/setup.bash
source /tmp/visual_grasp_manu/install/setup.bash
```

## Prepare The Bag Directory

If the input is a loose `.db3` file, create the ROS 2 bag metadata directory:

```bash
ros2 run visual_grasp_manu prepare_rosbag2_sqlite \
  outputs/datasets/object_scan_001_0.db3 \
  outputs/datasets/object_scan_001 \
  --force
```

The expected bag topics are:

```text
/camera/ee_cam/color/image_raw
/camera/ee_cam/aligned_depth_to_color/image_raw
/camera/ee_cam/color/camera_info
```

## Camera Pose Visualization

Launch the RGB-D bag, RTAB-Map RGB-D odometry, point cloud generation, and RViz:

```bash
ros2 launch visual_grasp_manu object_scan_camera_pose.launch.py \
  bag_path:=outputs/datasets/object_scan_001 \
  rviz:=true
```

Expected RViz output:

```text
/visual_grasp_manu/debug/rgbd_cloud
/visual_grasp_manu/inferred_camera_path
/visual_grasp_manu/camera_pose_markers
```

## Mask, Pose, And Mesh Replay

Launch the deterministic replay of the precomputed mask dataset:

```bash
ros2 launch visual_grasp_manu precomputed_mask_pose_demo.launch.py \
  dataset_path:=outputs/datasets/object_scan_001_scan_blue_60_sam2 \
  rviz:=true \
  rate_hz:=6.0 \
  loop:=true
```

The replay publishes:

```text
/visual_grasp_manu/live/rgbd_cloud
/visual_grasp_manu/mask_overlay/image
/visual_grasp_manu/mask_overlay/cloud
/visual_grasp_manu/mask_overlay/markers
/visual_grasp_manu/inferred_camera_path
/visual_grasp_manu/camera_pose_markers
/visual_grasp_manu/object_mesh_marker
```

## Validate A Scan Dataset

```bash
ros2 run visual_grasp_manu validate_scan_dataset outputs/datasets/object_scan_001_scan
ros2 run visual_grasp_manu validate_scan_dataset outputs/datasets/object_scan_001_scan --stage masks
ros2 run visual_grasp_manu validate_scan_dataset outputs/datasets/object_scan_001_scan --stage mesh
```

## Generate Masks Offline

Generate text-conditioned masks for an extracted scan dataset:

```bash
ros2 run visual_grasp_manu generate_grounded_sam2_masks \
  outputs/datasets/object_scan_001_scan \
  --prompt "blue object" \
  --grounding-config /path/to/GroundingDINO_SwinT_OGC.py \
  --grounding-checkpoint /path/to/groundingdino_swint_ogc.pth \
  --sam2-config configs/sam2.1/sam2.1_hiera_t.yaml \
  --sam2-checkpoint /path/to/sam2.1_hiera_tiny.pt \
  --device cpu \
  --grounding-device cpu \
  --save-overlays \
  --overwrite
```

Preview generated masks:

```bash
ros2 run visual_grasp_manu preview_scan_masks \
  outputs/datasets/object_scan_001_scan \
  --output outputs/datasets/object_scan_001_scan/mask_preview/preview.png
```

## Generate The Object Mesh

Fuse masked RGB-D frames into an object mesh:

```bash
ros2 run visual_grasp_manu generate_tsdf_mesh \
  outputs/datasets/object_scan_001_scan \
  --voxel-length 0.003 \
  --sdf-trunc 0.015 \
  --depth-trunc 1.5 \
  --min-mask-pixels 50
```

Inspect the mesh locally:

```bash
ros2 run visual_grasp_manu visualize_mesh outputs/datasets/object_scan_001_scan
```
