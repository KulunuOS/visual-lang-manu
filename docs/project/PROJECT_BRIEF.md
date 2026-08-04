# Project Brief

## Objective

Build a ROS 2 package that converts visual perception streams into grasp candidates for object grasping.

The main node should subscribe to model-appropriate ROS 2 topics, run a configurable perception and grasp-inference pipeline, and publish scored grasp candidates for a downstream manipulation planner or controller.

## Scope

- ROS 2 package named `visual_grasp_manu`.
- Main runtime component: a grasp candidate generation node.
- Supported input families:
  - RGB-D image streams and camera info,
  - registered point clouds,
  - segmentation masks or class-labeled object detections,
  - 6-DoF object pose estimates,
  - model-specific auxiliary topics.
- Supported pipeline styles:
  - direct RGB-D or point-cloud grasp inference,
  - semantic segmentation followed by grasp generation,
  - 6-DoF object pose localization followed by grasp generation,
  - configurable multi-stage pipelines for comparing speed and quality.
- Candidate inference backends should remain pluggable. Examples include 6DoF-GraspNet, Contact-GraspNet, custom learned models, geometric methods, or hybrid pipelines.
- Outputs should include grasp pose, score, frame id, model/backend metadata, and any fields needed by downstream manipulation logic.

## Non-Goals

- Do not hard-code one camera topic layout or one inference backend.
- Do not make the main ROS 2 node depend directly on one model implementation.
- Do not commit private model checkpoints, raw datasets, robot credentials, or local configuration.
- Do not treat generated candidates as executable robot commands until a downstream planner and safety layer validate them.

## Data Sources

Potential data sources include public RGB-D datasets, simulator outputs, recorded ROS 2 bags, object pose datasets, and backend-specific model assets.

For each dataset, bag, checkpoint, or asset, document:

- source and license,
- expected topic schema or file format,
- preprocessing requirements,
- whether the artifact is safe to publish,
- where large or private files should live outside git.

## Evaluation

Track both grasp quality and runtime behavior:

- valid candidate count per frame,
- candidate score distribution,
- grasp success rate when evaluated in simulation or hardware,
- pose error when object-pose ground truth exists,
- segmentation or localization quality when those stages are enabled,
- end-to-end latency from sensor timestamp to published candidates,
- per-stage latency for segmentation, pose estimation, and grasp inference,
- dropped frames and synchronization failures,
- memory and GPU usage for each backend.

## Public Release Boundary

Only commit files that are safe for a public repository. Generated outputs, raw datasets, credentials, local configuration, and private notes stay out of git unless explicitly reviewed for publication.
