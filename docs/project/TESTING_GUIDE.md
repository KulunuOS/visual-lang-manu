# Testing Guide

## Automated Checks

Add concrete commands as the toolchain becomes available.

Initial placeholder:

```bash
python -m pytest
```

## Manual Checks

TODO: Document simulator launch, visualization, dataset inspection, robot safety checks, camera calibration checks, and expected outputs.

## Public-Safety Check

Before committing:

```bash
git status --short
git diff --cached
```

Confirm that staged files contain no secrets, private paths, local machine metadata, raw datasets, checkpoints, or private workflow provenance.
