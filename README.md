# wuji-hand-description

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-hand-description)](https://github.com/wuji-technology/wuji-hand-description/releases)

Robot model description package for Wuji Hand. Provides URDF models, MuJoCo (MJCF) models, and high-quality meshes for simulation and visualization. Includes configuration files for ROS2 visualization (RViz).

## Table of Contents

- [Repository Structure](#repository-structure)
- [Usage](#usage)
  - [1. MuJoCo Usage](#1-mujoco-usage)
  - [2. ROS2 and RViz Usage](#2-ros2-and-rviz-usage)
- [Contact](#contact)

## Repository Structure

```text
├── launch/
│   └── ...
├── meshes/
│   └── ...
├── mjcf/
│   ├── left.xml
│   └── right.xml
├── rviz/
│   └── ...
├── urdf/
│   ├── left.urdf
│   ├── right.urdf
│   ├── left-ros.urdf
│   └── right-ros.urdf
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Directory Description

| Directory | Description |
|-----------|-------------|
| `urdf/` | Contains URDF files for Left and Right hands. `left.urdf`/`right.urdf` use relative paths for local tools; `left-ros.urdf`/`right-ros.urdf` use absolute package paths for RViz and Launch files. |
| `mjcf/` | Contains MuJoCo XML model files (`left.xml`, `right.xml`) for simulation. |
| `meshes/` | High-quality STL files for visualization and collision. |
| `launch/` | Python launch scripts to visualize the model in RViz. |
| `rviz/` | Default RViz configuration files. |

## Usage

### 1. MuJoCo Usage

If you only want to view the model in MuJoCo, you don't need to build the ROS package. Just ensure you have the `mujoco` python package installed.

```
git clone https://github.com/wuji-technology/wuji_hand_description.git

pip install mujoco
```

#### View Right Hand

```
python -m mujoco.viewer --mjcf=mjcf/right.xml
```

#### View Left Hand

```
python -m mujoco.viewer --mjcf=mjcf/left.xml
```

### 2. ROS2 and RViz Usage

If you want to use this robot in ROS 2 (Humble/Rolling) with RViz visualization, follow these steps. The launch files automatically use the ROS-compatible URDFs (*-ros.urdf).

#### 2.1 Clone into Workspace

Navigate to the `src` directory of your ROS 2 workspace(for example, `~/ros2_ws/src`):

```
cd ~/ros2_ws/src
git clone https://github.com/wuji-technology/wuji_hand_description.git
```

#### 2.2 Install Dependencies

Install required ROS 2 dependencies (e.g., joint_state_publisher_gui):

```
rosdep install --from-paths src --ignore-src -r -y
```

#### 2.3 Build the Package and Source the Environment

Compile the package using colcon. Note that the package name uses underscores (_).

```
colcon build --packages-select wuji_hand_description
source install/setup.bash
```

#### 2.4 RViz Visualization

These commands will launch robot_state_publisher, joint_state_publisher_gui, and RViz.

Visualize Right Hand

```
ros2 launch wuji_hand_description display.right.py
```

Visualize Left Hand

```
ros2 launch wuji_hand_description display.left.py
```

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
