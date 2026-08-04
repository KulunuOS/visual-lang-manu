# 0001: Workspace Template

## Status

Accepted

## Context

The project starts as a reusable public-safe research workspace for visual grasping and manipulation. The exact project objective, simulator or hardware target, data sources, and evaluation protocol will be filled in later.

## Decision

Use a predictable repository layout:

- `src/visual_grasp_manu/` for reusable package code,
- `scripts/` for runnable utilities,
- `tests/` for validation,
- `docs/` for project memory and workflows,
- `outputs/` for generated artifacts that are ignored by default,
- `assets/` for reviewed public assets.

## Consequences

The scaffold is immediately usable for public git hosting while keeping generated artifacts and local-only files out of version control. Future work must update documentation and ignore rules whenever new data, assets, tools, or outputs are introduced.
