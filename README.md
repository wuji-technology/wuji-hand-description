# WujiHand Description Package

This package provides the URDF model, MuJoCo (MJCF) models, and meshes for the Wuji Hand. It also includes configuration files for ROS 2 visualization using RViz.

## Project Structure

- **`urdf/`**: Contains the Unified Robot Description Format files for the Left and Right hands.
    - `left.urdf` / `right.urdf`: Standard URDFs using relative paths (../meshes). Ideal for local tools, scripts, or converters that don't depend on ROS.
    - `left-ros.urdf` / `right-ros.urdf`: ROS-specific URDFs using absolute package paths (package://wuji_hand_description/...). Used by RViz and Launch files.
- **`mjcf/`**: Contains MuJoCo XML model files (`left.xml`, `right.xml`) for simulation.
- **`meshes/`**: High-quality STL files for visualization and collision.
- **`launch/`**: Python launch scripts to visualize the model in RViz.
- **`rviz/`**: Default RViz configuration files.

## 1. Mujoco Usage

If you only want to view the model in MuJoCo, you don't need to build the ROS package. Just ensure you have the `mujoco` python package installed.

```
git clone https://github.com/wuji-technology/wuji_hand_description.git

pip install mujoco
```

### View Right Hand

```
python -m mujoco.viewer --mjcf=mjcf/right.xml
```

### View Left Hand

```
python -m mujoco.viewer --mjcf=mjcf/left.xml
```

## 2. ROS2 and RViz Usage

If you want to use this robot in ROS 2 (Humble/Rolling) with RViz visualization, follow these steps. The launch files automatically use the ROS-compatible URDFs (*-ros.urdf).

### 2.1 Clone into Workspace

Navigate to the `src` directory of your ROS 2 workspace(for example, `~/ros2_ws/src`):

```
cd ~/ros2_ws/src
git clone https://github.com/wuji-technology/wuji_hand_description.git
```

### 2.2 Install Dependencies

Install required ROS 2 dependencies (e.g., joint_state_publisher_gui):

```
rosdep install --from-paths src --ignore-src -r -y
```

### 2.3 Build the Package and Source the Environment

Compile the package using colcon. Note that the package name uses underscores (_).

```
colcon build --packages-select wuji_hand_description
source install/setup.bash
```

### 2.4 RVIZ Visualization

These commands will launch robot_state_publisher, joint_state_publisher_gui, and RViz.

Visualize Right Hand

```
ros2 launch wuji_hand_description display.right.py
```

Visualize Left Hand

```
ros2 launch wuji_hand_description display.left.py
```
