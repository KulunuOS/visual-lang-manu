# Current State

## Implemented

- Workspace scaffold.
- Documentation structure.
- Public-safe ignore rules.
- ROS 2 package metadata scaffold.
- Default pipeline parameter file at `config/pipeline.yaml`.
- Launch scaffold at `launch/grasp_candidates.launch.py`.
- Dev-container scaffold using ROS 2 Humble.
- Phase-1 pose-stub grasp visualization demo.
- First demo research note at `docs/project/FIRST_DEMO_RESEARCH.md`.
- FoundationPose first-demo feasibility note at `docs/project/FOUNDATIONPOSE_DEMO_FEASIBILITY.md`.
- Inference runtime background note at `docs/project/INFERENCE_RUNTIME_BACKGROUND.md`.

## Known Gaps

- Grasp candidate message contract is not finalized yet.
- Inference backend interface is not implemented yet.
- Topic synchronization and input validation are not implemented yet.
- Segmentation and 6-DoF pose localization stages are not implemented yet.
- Phase-1 demo publishes visualization markers only, not a structured grasp candidate message yet.

## Next Work

1. Define the grasp candidate message contract.
2. Add a structured grasp candidate publisher alongside the RViz markers.
3. Implement ROS 2 parameter parsing and required-topic validation for real sensor topics.
4. Add an inference adapter interface with a CAD-pose baseline backend.
5. Add FoundationPose as a replaceable pose provider.
6. Add the first runnable rosbag demo.
