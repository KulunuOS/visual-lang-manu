# 0002: Configurable ROS 2 Grasp Pipeline

## Status

Accepted

## Context

Different grasp inference methods require different inputs. Some methods operate directly on RGB-D images or point clouds. Others require semantic segmentation, object detections, or 6-DoF object pose localization before grasp candidates can be generated.

The package should support comparing methods and pipeline variants without rewriting the main ROS 2 node for each backend.

## Decision

Use a configurable pipeline architecture:

- keep the main ROS 2 node responsible for parameters, subscriptions, synchronization, frame metadata, and publishing;
- keep model-specific inference behind backend adapters;
- allow parameter files to select topic names, required preprocessing stages, inference method, thresholds, frames, and debug outputs;
- document each backend's required topics, dependencies, and demo commands in public-facing documentation.

## Consequences

The initial implementation has more interface design work than a single-model node, but it keeps the package useful for evaluating multiple grasp methods. Backend-specific assumptions should remain localized, and new methods should be added by configuration plus adapter code rather than by forking the node.
