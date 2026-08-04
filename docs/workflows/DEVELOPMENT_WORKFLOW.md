# Development Workflow

1. Update `docs/project/PROJECT_BRIEF.md` when project scope changes.
2. Update `README.md` when installation, launch commands, demo examples, required topics, or backend setup changes.
3. Keep `.devcontainer/devcontainer.json` current as ROS 2, GPU, simulator, camera, or model dependencies change.
4. Add or update tests before changing shared behavior when practical.
5. Keep reusable implementation in `src/visual_grasp_manu/`.
6. Keep launch files and parameter files explicit and reviewable.
7. Keep scripts thin and task-specific.
8. Write generated outputs under `outputs/`.
9. Update `docs/project/CURRENT_STATE.md` after meaningful changes.
10. Review staged changes for public-safety before committing.

For experiments, record:

- command,
- ROS 2 topic set and remappings,
- backend or pipeline configuration,
- dataset or asset version,
- metrics,
- latency and hardware notes when relevant,
- known limitations,
- whether results are public-safe.

## Pipeline Configuration Workflow

For each new grasp pipeline:

1. Define required input topics and message types.
2. Define optional preprocessing stages such as segmentation, detection, point-cloud filtering, or 6-DoF pose localization.
3. Implement the backend behind a stable adapter interface.
4. Add a parameter file or documented parameter block for the pipeline.
5. Add tests for parameter validation and message conversion.
6. Add README instructions for installing backend dependencies and running a demo.
