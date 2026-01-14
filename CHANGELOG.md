# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Remove version suffix from robot name (e.g., `wujihand-left-v1.0.2` → `wujihand-left`)
- Merge separate `display.left.py` and `display.right.py` into unified `display.launch.py` with `hand` parameter
- Exclude `mjcf/` directory from ROS2 installation

### Fixed

- RViz config uses relative topic path `robot_description` instead of absolute `/robot_description`
- This allows RViz running in a ROS2 namespace to correctly subscribe to the robot_description topic

## [0.1.0] - 2025-11-27

### Added

- URDF models for Wuji Hand left and right hands
- MJCF (MuJoCo XML) models for simulation
- High-quality STL mesh files for visualization and collision
- ROS2 launch files for left and right hands
- RViz configuration file for robot display
- CMakeLists.txt for ROS2 package build

[Unreleased]: https://github.com/wuji-technology/wuji-hand-description/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/wuji-technology/wuji-hand-description/releases/tag/v0.1.0
