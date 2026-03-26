# wuji-hand-description

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-hand-description)](https://github.com/wuji-technology/wuji-hand-description/releases)

Robot model description package for Wuji Hand. Provides URDF, MuJoCo (MJCF), and USD assets for simulation and visualization. Includes ROS2 launch and RViz configuration files for quick inspection of left and right hand models.

## Table of Contents

- [Repository Structure](#repository-structure)
- [Usage](#usage)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Running](#running)
  - [Output](#output)
- [Contact](#contact)

## Repository Structure

```text
├── launch/              // ROS2 launch files for model visualization
├── meshes/              // STL meshes for visual and collision geometry
├── mjcf/                // MuJoCo XML models for left and right hands
├── rviz/                // RViz presets for left and right hands
├── step/                // STEP source files for CAD exchange
├── usd/                 // Isaac Sim USD assets for left and right hands
├── urdf/                // URDF models for local tools and ROS2
├── CMakeLists.txt       // ROS2 package installation rules
├── package.xml          // ROS2 package manifest
└── README.md
```

## Usage

### Prerequisites

- Python 3 and the `mujoco` package for MuJoCo viewing
- ROS2 Humble or Rolling for RViz visualization
- NVIDIA Isaac Sim for USD asset inspection

### Installation

```bash
git clone https://github.com/wuji-technology/wuji-hand-description.git
cd wuji-hand-description
pip install mujoco
```

For ROS2 workspace usage:

```bash
cd ~/ros2_ws/src
git clone https://github.com/wuji-technology/wuji-hand-description.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wuji_hand_description
source install/setup.bash
```

### Running

#### MuJoCo

```bash
python -m mujoco.viewer --mjcf=mjcf/right.xml
python -m mujoco.viewer --mjcf=mjcf/left.xml
```

#### ROS2 and RViz

```bash
ros2 launch wuji_hand_description display.launch.py
ros2 launch wuji_hand_description display.launch.py hand:=right
```

#### Isaac Sim (USD)

Load `usd/left/wujihand.usd` or `usd/right/wujihand.usd` directly in Isaac Sim.
For a complete simulation example, see [isaaclab-sim](https://github.com/wuji-technology/isaaclab-sim).

### Output

```text
- MuJoCo loads the left or right hand model for interactive inspection.
- ROS2 launches robot_state_publisher, joint_state_publisher_gui, and RViz.
- Isaac Sim opens the prebuilt USD asset with materials, physics, and collision filters.
```

## Contact

For any questions, please contact support@wuji.tech.
