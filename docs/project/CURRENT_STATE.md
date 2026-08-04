# Current State

## Implemented

- Workspace scaffold.
- Documentation structure.
- Public-safe ignore rules.
- ROS 2 package metadata scaffold.
- Default pipeline parameter file at `config/pipeline.yaml`.
- Launch scaffold at `launch/grasp_candidates.launch.py`.
- Dev-container scaffold using ROS 2 Humble.

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
3. Add an inference adapter interface with a placeholder backend.
4. Add launch tests or unit tests for configuration loading.
5. Add the first runnable demo using either recorded sensor data, simulated topics, or a simple synthetic publisher.
