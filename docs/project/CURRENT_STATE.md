# Current State

## Implemented

- Workspace scaffold.
- Documentation structure.
- Public-safe ignore rules.
- ROS 2 package metadata scaffold.
- Default pipeline parameter file at `config/pipeline.yaml`.
- Launch scaffold at `launch/grasp_candidates.launch.py`.
- Dev-container scaffold using ROS 2 Humble.
- First demo research note at `docs/project/FIRST_DEMO_RESEARCH.md`.
- FoundationPose first-demo feasibility note at `docs/project/FOUNDATIONPOSE_DEMO_FEASIBILITY.md`.
- Inference runtime background note at `docs/project/INFERENCE_RUNTIME_BACKGROUND.md`.

## Known Gaps

- Main grasp candidate node is not implemented yet.
- Grasp candidate message contract is not finalized yet.
- Inference backend interface is not implemented yet.
- Topic synchronization and input validation are not implemented yet.
- Segmentation and 6-DoF pose localization stages are not implemented yet.
- No tests are defined yet.

## Next Work

1. Define the grasp candidate message contract.
2. Implement ROS 2 parameter parsing and required-topic validation.
3. Add a pose-topic stub so the grasp and visualization path can run before GPU pose estimation is integrated.
4. Add an inference adapter interface with a CAD-pose baseline backend.
5. Add RViz marker visualization for candidate grasps.
6. Add FoundationPose as a replaceable pose provider.
7. Add the first runnable rosbag demo.
