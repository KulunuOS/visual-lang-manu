# Repository Guidelines

## Project Purpose & Context

This workspace is a reusable public-safe template for visual grasping and manipulation research.

Fill in these sections before implementation begins:

- Project objective:
- Target robot, simulator, or hardware:
- Main manipulation tasks:
- Data sources:
- Evaluation metrics:
- Public release boundary:

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

- `src/visual_grasp_manu/` for reusable Python package code.
- `scripts/` for local entry points such as dataset generation, evaluation, conversion, and visualization.
- `tests/` for unit and integration tests.
- `assets/` for reviewed public assets only, with source and license notes when needed.
- `docs/project/` for project brief, setup, current state, and testing guide.
- `docs/workflows/` for repeatable development, data, training, and evaluation workflows.
- `docs/adr/` for architecture decision records.
- `outputs/` for generated datasets, logs, videos, and temporary experiment products that should usually remain untracked.

Prefer small, focused modules. Avoid growing notebooks or scripts into hidden application code; extract reusable logic into `src/visual_grasp_manu/`.

## Build, Test, and Development Commands

No fixed toolchain is required by the template. Add exact commands here as the project becomes concrete.

Initial checks:

- `python -m pytest`: run tests after test files are added.
- `python scripts/<name>.py`: run a project script.
- `rg --files`: inspect tracked source layout quickly.

When introducing `pyproject.toml`, `uv`, `poetry`, `make`, Docker, ROS, simulator dependencies, or GPU-specific setup, document the exact installation and execution commands in `docs/project/SETUP.md` and `docs/project/TESTING_GUIDE.md`.

## Coding Style & Naming Conventions

Use standard Python conventions unless a future project-specific toolchain says otherwise:

- 4-space indentation.
- `snake_case` for functions, variables, modules, and script names.
- `PascalCase` for classes.
- Type hints for public interfaces.
- Short docstrings for public functions and classes.

Comments should clarify non-obvious assumptions, coordinate frames, camera conventions, control semantics, tensor shapes, dataset schemas, and evaluation thresholds. Avoid comment noise.

## Documentation Requirements

Keep these files current:

- `README.md`: public overview, repository layout, and basic usage.
- `docs/project/PROJECT_BRIEF.md`: purpose, scope, assumptions, and non-goals.
- `docs/project/SETUP.md`: installation, dependencies, environment variables, simulator requirements, and data access rules.
- `docs/project/CURRENT_STATE.md`: implemented components, known gaps, active decisions, and next work.
- `docs/project/TESTING_GUIDE.md`: validation commands, expected outputs, and manual checks.
- `docs/workflows/DEVELOPMENT_WORKFLOW.md`: repeatable implementation and review workflow.
- `docs/adr/0001-workspace-template.md`: initial architecture decision record.

Update documentation in the same change that introduces new commands, data formats, experiments, or public-facing behavior.

## Testing Guidelines

Place tests under `tests/` and name them `test_<module>.py`.

Prioritize tests for:

- dataset schema validation,
- camera and coordinate-frame transforms,
- grasp and placement success criteria,
- policy/action interfaces,
- reproducibility-sensitive sampling,
- file writing behavior that could accidentally publish generated or private artifacts.

Add a regression test for every bug fix when practical. For visual or simulator behavior, include a lightweight smoke test plus documented manual verification steps.

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
