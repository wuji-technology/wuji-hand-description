# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.1] - 2026-03-30

### Fixed

- Fixed installed ROS URDF mesh paths to match the packaged `meshes` layout
- Removed fingertip marker spheres from MJCF models

## [1.0.0] - 2026-02-11

Major update with corrected kinematics, calibrated dynamics, and new simulation formats.

### Added

- MJX models optimized for JAX-based simulation (Brax)
- Per-joint calibrated actuator parameters in `params.csv`
- Chinese documentation (`README_zh.md`)
- Git LFS support (`.gitattributes`) for large binary assets
- ROS URDF variants with `package://` paths (`left-ros.urdf`, `right-ros.urdf`)
- Sparse checkout support for downloading individual models
- GitHub Actions CI/CD workflows
  - `sync_public.yaml`: Tag push syncs to public repo
  - `ci_test.yaml`: MuJoCo/MJX/Isaac Sim validation tests
  - `auto-release.yml`: Automatic GitHub Release creation

### Changed

- USD assets redesigned from monolithic configuration to modular mesh references
- Launch argument changed from `hand:=left` to `robot:=left`
- STEP files removed from public release (contact support@wuji.tech if needed)

### Kinematics changes (relative to v0.2.4)

- Corrected finger 2–5 joint 2 axis from `-1 0 0` to `0 1 0`
- Corrected finger 1 joint 1 axis (left hand reversed)
- Joint limits are now asymmetric and individually calibrated per finger and per hand, replacing the previous uniform ±0.37 rad MCP2 range
- MuJoCo default `armature` changed from 0.005 to 0.0002; `damping` changed from 0.05 to 0.0
- Actuator gains (`kp`, `kv`) are now per-joint calibrated values; see `params.csv`

### Notes

- Model parameters are based on CAD data and real-world measurements

## [0.2.4] - 2026-03-23

### Added

- Added USD assets for NVIDIA Isaac Sim with fused meshes, PBR materials, physics properties, and collision filter pairs.
- Added README guidance for loading the USD assets in Isaac Sim.
- Added an MIT license file for the public repository.

### Changed

- Standardized the MCP2 joint limits of the four non-thumb fingers to `-0.37` to `0.37` across the URDF, MJCF, and USD models.
- Refreshed the README structure and usage examples to match the current repository contents.

### Fixed

- Corrected repository metadata and package links for the current `wuji-hand-description` repository.
- Corrected package version metadata for the `0.2.4` release.

## [0.2.3] - 2026-03-19

### Fixed

- Standardized left and right hand joint limits using averaged calibrated values across the hand models.

## [0.2.2] - 2026-02-02

### Fixed

- Fixed mesh references in the URDF models so left and right hand assets load correctly in local tools.

## [0.2.1] - 2026-01-20

### Fixed

- Fixed the robot name in the left-hand models.
- Fixed inertia values.
- Fixed joint motion range limits.
- Fixed joint torque limits.
- Fixed self-collision groups.
- Fixed the RViz fixed frame for right hand visualization.

## [0.2.0] - 2026-01-19

### Changed

- Removed the version suffix from the robot name.
- Unified the left and right hand ROS2 visualization entry points behind a single launch interface with a `hand` selector.
- Streamlined the ROS2 package installation layout for visualization workflows.

### Fixed

- Fixed the RViz robot description topic handling so visualization works correctly inside a ROS2 namespace.

## [0.1.0] - 2025-11-27

### Added

- Added URDF models for Wuji Hand left and right hands.
- Added MJCF models for MuJoCo simulation.
- Added high-quality STL meshes for visualization and collision.
- Added ROS2 launch files for left and right hands.
- Added RViz configuration files for robot display.
- Added ROS2 package build configuration.

[Unreleased]: https://github.com/wuji-technology/wuji-hand-description/compare/v1.0.1...HEAD
[1.0.1]: https://github.com/wuji-technology/wuji-hand-description/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.2.4...v1.0.0
[0.2.4]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.2.3...v0.2.4
[0.2.3]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.2.2...v0.2.3
[0.2.2]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/wuji-technology/wuji-hand-description/releases/tag/v0.1.0
