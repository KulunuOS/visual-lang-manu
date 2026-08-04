# Repository Guidelines

## Project Purpose & Context

This repository is a ROS 2 package for configurable visual grasp candidate generation from RGB-D and related perception streams.

The main node is expected to subscribe to one or more ROS 2 sensor or perception topics, run a selected grasp-inference pipeline, and publish grasp candidates for a downstream manipulation stack.

Current project context:

- Project objective: generate grasp candidates for an object from visual sensor streams.
- Primary inputs: RGB-D image streams, camera info, registered point clouds, segmentation outputs, object detections, or 6-DoF pose estimates depending on the selected backend.
- Primary output: scored grasp candidates in a documented ROS 2 message contract.
- Inference methods: keep room for multiple backends, including methods such as 6DoF-GraspNet, Contact-GraspNet, custom RGB-D models, geometric heuristics, or object-pose-driven pipelines.
- Pipeline flexibility: semantic segmentation and 6-DoF object pose localization may be optional stages before grasp candidate generation.
- Development environment: maintain the dev container as dependencies and runtime requirements evolve.
- Public documentation: keep `README.md` updated with install instructions, launch commands, demo examples, required topics, and backend-specific setup.

Do not leave private paths, private dataset names, credentials, tokens, hostnames, usernames, unpublished results, or local machine details in committed files.

## Public Repository Hygiene

This repository is intended to be connected to a public git remote. Treat every committed file as publishable by default.

- Do not commit secrets, credentials, API keys, SSH keys, tokens, `.env` files, private configuration, or private dataset artifacts.
- Do not commit generated outputs unless they are deliberately reviewed public examples.
- Do not mention private tooling, local workflow provenance, or internal development process details in docstrings, README content, commit messages, comments, notebooks, generated docs, or release artifacts.
- Keep implementation comments focused on domain logic, assumptions, tensor shapes, algorithms, and reproducibility.
- Before committing, inspect staged changes with `git diff --cached` and confirm that no local-only metadata or sensitive content is included.
- Keep private automation state, editor folders, local caches, checkpoints, and raw datasets untracked.

## Project Structure & Module Organization

Keep the workspace predictable:

- `src/visual_grasp_manu/` for ROS 2 node code, pipeline interfaces, inference adapters, message conversion, and shared utilities.
- `config/` for parameter files that select topics, frames, inference backend, thresholds, synchronization policy, and debug behavior.
- `launch/` for ROS 2 launch files.
- `scripts/` for local entry points such as dataset conversion, evaluation, profiling, and visualization.
- `tests/` for unit and integration tests.
- `assets/` for reviewed public assets only, with source and license notes when needed.
- `docs/project/` for project brief, setup, current state, and testing guide.
- `docs/workflows/` for repeatable development, data, training, and evaluation workflows.
- `docs/adr/` for architecture decision records.
- `outputs/` for generated datasets, logs, videos, and temporary experiment products that should usually remain untracked.

Prefer small, focused modules. Keep model-specific code behind adapter interfaces so one backend does not leak assumptions into the main ROS 2 node.

## Build, Test, and Development Commands

Use the dev container for normal development. Keep `.devcontainer/devcontainer.json` current as ROS 2, simulator, GPU, model, or system-library requirements change.

Initial checks:

- `source /opt/ros/humble/setup.bash`: load the default ROS 2 environment used by the current dev container.
- `colcon build --symlink-install`: build the package from a ROS 2 workspace.
- `source install/setup.bash`: overlay the built workspace.
- `python -m pytest`: run Python tests after test files are added.
- `ros2 launch visual_grasp_manu grasp_candidates.launch.py`: launch the current node scaffold.
- `rg --files`: inspect tracked source layout quickly.

When introducing model dependencies, CUDA requirements, simulator dependencies, external checkpoints, message packages, or GPU-specific setup, document exact installation and execution commands in `README.md`, `docs/project/SETUP.md`, and `docs/project/TESTING_GUIDE.md`.

## Coding Style & Naming Conventions

Use standard Python conventions unless a future project-specific toolchain says otherwise:

- 4-space indentation.
- `snake_case` for functions, variables, modules, and script names.
- `PascalCase` for classes.
- Type hints for public interfaces.
- Short docstrings for public functions and classes.

Comments should clarify non-obvious assumptions, coordinate frames, camera conventions, topic synchronization, message schemas, model inputs, tensor shapes, dataset schemas, and evaluation thresholds. Avoid comment noise.

## Documentation Requirements

Keep these files current:

- `README.md`: public overview, repository layout, and basic usage.
- `docs/project/PROJECT_BRIEF.md`: purpose, scope, assumptions, and non-goals.
- `docs/project/SETUP.md`: installation, dependencies, environment variables, simulator requirements, and data access rules.
- `docs/project/CURRENT_STATE.md`: implemented components, known gaps, active decisions, and next work.
- `docs/project/TESTING_GUIDE.md`: validation commands, expected outputs, and manual checks.
- `docs/workflows/DEVELOPMENT_WORKFLOW.md`: repeatable implementation and review workflow.
- `docs/adr/0001-workspace-template.md`: initial architecture decision record.

Update documentation in the same change that introduces new commands, data formats, topics, frames, message contracts, inference backends, experiments, or public-facing behavior.

## Testing Guidelines

Place tests under `tests/` and name them `test_<module>.py`.

Prioritize tests for:

- ROS 2 parameter parsing and backend selection,
- topic remapping and required-input validation,
- dataset schema validation,
- camera and coordinate-frame transforms,
- grasp and placement success criteria,
- grasp candidate message conversion,
- inference adapter interfaces,
- reproducibility-sensitive sampling,
- file writing behavior that could accidentally publish generated or private artifacts.

Add a regression test for every bug fix when practical. For ROS graph, visual, or simulator behavior, include a lightweight smoke test plus documented manual verification steps.

## Commit & Pull Request Guidelines

Use short, imperative commit subjects, for example:

- `Initialize workspace scaffold`
- `Add grasp dataset schema`
- `Document camera calibration workflow`

Keep commits focused on one logical change. Avoid references to private tooling or development provenance in commit messages.

Pull requests should include:

- concise change summary,
- test evidence,
- documentation updates,
- data or artifact release notes,
- any new dependency or environment requirement.
