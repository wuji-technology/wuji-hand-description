# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Fixed

- RViz config uses relative topic path `robot_description` instead of absolute `/robot_description`
- This allows RViz running in a ROS2 namespace to correctly subscribe to the robot_description topic
