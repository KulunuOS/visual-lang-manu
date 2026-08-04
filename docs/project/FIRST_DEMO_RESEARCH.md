# First Demo Research

## Demo Goal

The first demo should replay a ROS 2 bag containing an object of interest, infer grasp candidates, and visualize the result.

Recommended initial workflow:

1. Play a recorded ROS 2 bag with RGB-D, camera info, and optional point cloud topics.
2. Localize the object of interest using the available CAD mesh or a segmentation-plus-pose stage.
3. Generate grasp candidates in the object or camera frame.
4. Filter and rank candidates using depth, point-cloud collision checks, gripper constraints, and approach direction constraints.
5. Publish grasp candidates and visualization markers.
6. View the result in RViz with RGB/depth context, object pose, candidate frames, and candidate scores.

## Recommended First Pipeline

Use a CAD/object-pose-first pipeline for the first demo.

Rationale:

- The object of interest is known and has a CAD file.
- Pose estimation reduces the search space before grasp generation.
- Candidate visualization is easier to debug because candidates can be expressed in the object frame and transformed into camera or world frames.
- The grasp-generation stage can start with precomputed CAD-frame candidates, then later be replaced by learned backends.

Pipeline:

```text
rosbag RGB-D topics
  -> RGB-D synchronization
  -> object segmentation or ROI selection
  -> CAD-based 6-DoF object pose estimation
  -> CAD-frame grasp candidate transform
  -> depth/point-cloud collision filtering
  -> scored grasp candidate publication
  -> RViz marker visualization
```

## Candidate Methods

### CAD-Based Object Localization

FoundationPose is a strong candidate when a CAD mesh is available. It supports model-based 6-DoF pose estimation and tracking for novel objects without object-specific fine-tuning. NVIDIA reports real-time pose tracking performance and provides NGC model packaging.

MegaPose is another CAD-based option. It estimates the 6-DoF pose of novel objects from RGB images, camera intrinsics, an object mesh, and an image ROI. It is older than FoundationPose but still a useful fallback because the assumptions match a known-object setup.

For the first demo, start with a narrow pose-estimation contract:

- input: RGB, depth, camera info, object mesh, optional mask or ROI;
- output: `geometry_msgs/PoseStamped` or a custom object-pose message;
- optional output: segmentation mask and pose confidence.

### Lightweight Grasp Generation

For a known CAD object, the lightest practical grasp generator is not necessarily a neural network. Start with an object-frame grasp library:

- precompute parallel-jaw grasp poses on the CAD mesh;
- store grasps in object coordinates;
- transform candidates by the estimated object pose;
- filter against observed depth and scene point cloud;
- score by collision clearance, approach direction, antipodal quality, pose confidence, and robot reachability hooks.

This gives a fast, explainable baseline. It also makes the first ROS 2 demo independent of heavyweight model checkpoints.

### Learned Grasp Backends To Evaluate

GraspFast is a 2024 RGB-D method focused on lightweight and fast 6-DoF grasp detection. It uses foreground-guided point-cloud preprocessing and a lightweight hierarchical backbone. It is a strong candidate if the project needs scene-level general grasp detection.

Region-aware Normalized Grasp Network from the RegionNormalizedGrasp project is a 2024 CoRL method that reports efficient real-time inference around 50 FPS and strong GraspNet benchmark gains. It is worth evaluating for a fast learned backend.

FlexLoG is a 2024 grasp-centric framework designed for both scene-level and target-oriented grasping. This is relevant because the project may switch between generic scene grasping and object-of-interest grasping.

CenterGrasp is object-aware and predicts shape plus 6-DoF grasps from RGB-D. It is attractive if CAD is unavailable or partial views make shape reasoning important, but it may be heavier than a CAD-pose-plus-grasp-library demo.

GraspGen is a newer 2025 diffusion-based grasp framework from NVIDIA. It is promising and reports real-time performance before TensorRT, but diffusion-based generation is likely more complex than needed for the first demo.

AnyGrasp provides a practical RGB-D to ranked 6-DoF grasp SDK. It may be useful as a quick baseline, but licensing, binary dependencies, and integration constraints should be checked before making it a core public dependency.

Contact-GraspNet remains a solid baseline for raw point-cloud 6-DoF grasp generation, especially with segmentation-based cropping, but it is from 2021 and typically needs a GPU-class environment.

6-DoF GraspNet is older and less attractive for new integration because the reference implementation depends on old TensorFlow/Python tooling.

## First Demo Interfaces

Suggested topics:

- subscribe:
  - `/camera/color/image_raw`
  - `/camera/depth/image_rect_raw`
  - `/camera/color/camera_info`
  - optional `/camera/depth/color/points`
  - optional `/object_mask`
  - optional `/object_pose`
- publish:
  - `/visual_grasp_manu/object_pose`
  - `/visual_grasp_manu/grasp_candidates`
  - `/visual_grasp_manu/grasp_markers`
  - optional `/visual_grasp_manu/debug_image`

Visualization:

- publish `visualization_msgs/MarkerArray` for candidate frames, gripper jaws, object axes, and selected top candidate;
- color candidates by score;
- hide or fade collision-filtered candidates;
- include the object mesh marker transformed by the estimated pose when the CAD file is public-safe.

## Recommendation

Implement the first demo in two layers:

1. CAD-pose baseline:
   object pose from FoundationPose or a pose-topic stub, CAD-frame grasp library, collision filtering, RViz markers.
2. Learned backend adapter:
   add GraspFast or RegionNormalizedGrasp as the first scene-level learned method after the ROS 2 message contract is stable.

This keeps the first demo achievable while preserving the repository goal of comparing multiple reconfigurable pipelines.

## References To Revisit

- FoundationPose, CVPR 2024: CAD-aware 6-DoF pose estimation and tracking for novel objects.
- MegaPose, CoRL 2022: render-and-compare 6-DoF pose estimation for novel CAD objects.
- GraspFast, Pattern Recognition 2024: lightweight RGB-D 6-DoF grasp detection.
- RegionNormalizedGrasp / RNGNet, CoRL 2024: efficient region-aware 6-DoF grasp detection.
- FlexLoG, 2024: flexible framework for scene-level and target-oriented grasping.
- CenterGrasp, RA-L 2024: object-aware shape reconstruction and 6-DoF grasp estimation.
- GraspGen, 2025: diffusion-based 6-DoF grasp generation.
- Contact-GraspNet, ICRA 2021: efficient raw point-cloud 6-DoF grasp generation.
