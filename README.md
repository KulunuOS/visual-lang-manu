# visual_grasp_manu

Reusable research workspace for visual grasping and manipulation experiments.

This repository starts as a public-safe template. Fill in the project purpose, task scope, data sources, and experiment context before adding implementation-specific code.

## Repository Layout

```text
.
├── assets/
├── docs/
│   ├── adr/
│   ├── project/
│   └── workflows/
├── outputs/
│   ├── datasets/
│   ├── logs/
│   └── videos/
├── scripts/
├── src/visual_grasp_manu/
├── tests/
├── AGENTS.md
├── README.md
└── .gitignore
```

## Getting Started

1. Populate the project purpose in `docs/project/PROJECT_BRIEF.md`.
2. Record the current implementation state in `docs/project/CURRENT_STATE.md`.
3. Add repeatable commands to `docs/project/TESTING_GUIDE.md`.
4. Move reusable code into `src/visual_grasp_manu/`.
5. Keep generated datasets, videos, logs, and model checkpoints out of git unless explicitly reviewed for public release.

## Development

No build system is committed yet. Until one is added, keep workflows simple and scriptable:

```bash
python -m pytest
python scripts/<name>.py
```

Document new dependencies, datasets, and commands in `docs/project/SETUP.md` before relying on them.
