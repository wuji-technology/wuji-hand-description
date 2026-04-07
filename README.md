# wuji-hand-description

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-hand-description)](https://github.com/wuji-technology/wuji-hand-description/releases)

WUJI hand description package with simulation and visualization assets for MuJoCo, MJX, Isaac Sim, and ROS 2. The repository includes left and right hand models, calibrated inertial and actuator parameters, RViz launch files, and install-safe ROS URDF variants.

## Highlights

- Left and right 20-DoF hand models in URDF, MJCF, MJX, and USD.
- Shared mesh assets for visualization, collision, and simulator export.
- Calibrated actuator gains and limits in [`params.csv`](params.csv).
- ROS 2 package with RViz launch support for both installed and source-tree use.

## Repository Layout

```text
wuji-hand-description/
├── .github/workflows/      # Release automation
├── launch/
│   └── display.launch.py   # RViz / robot_state_publisher entrypoint
├── meshes/                 # STL meshes used by URDF and MuJoCo
│   ├── left/
│   └── right/
├── mjcf/                   # MuJoCo XML models
├── mjx/                    # MJX-ready MuJoCo models
├── urdf/                   # Source-tree and ROS-install URDF variants
├── usd/                    # Isaac Sim USD files and referenced USD meshes
├── rviz/
│   ├── left.rviz
│   └── right.rviz
├── params.csv              # Joint gains, limits, and armature values
├── CHANGELOG.md
├── CMakeLists.txt
├── LICENSE
├── package.xml
└── README.md
```

## Format Guide

| Use case | Entry file |
|----------|------------|
| Local URDF loading from a source checkout | `urdf/left.urdf` or `urdf/right.urdf` |
| ROS 2 package installation | `urdf/left-ros.urdf` or `urdf/right-ros.urdf` |
| MuJoCo | `mjcf/left.xml` or `mjcf/right.xml` |
| MJX / JAX | `mjx/left_mjx.xml` or `mjx/right_mjx.xml` |
| Isaac Sim | `usd/left.usd` or `usd/right.usd` |

## Installation

### Sparse checkout

```bash
git clone --filter=blob:none --sparse https://github.com/wuji-technology/wuji-hand-description.git
cd wuji-hand-description
git sparse-checkout set meshes urdf mjcf mjx
```

### Full clone

```bash
git clone https://github.com/wuji-technology/wuji-hand-description.git
cd wuji-hand-description
```

### ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/wuji-technology/wuji-hand-description.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wuji_hand_description
source install/setup.bash
```

## Usage

### MuJoCo

```bash
pip install mujoco
python -m mujoco.viewer --mjcf=mjcf/right.xml
```

```python
import mujoco

model = mujoco.MjModel.from_xml_path("mjcf/right.xml")
data = mujoco.MjData(model)

for _ in range(1000):
    mujoco.mj_step(model, data)
```

### MJX

```python
import jax
import mujoco
from mujoco import mjx

model = mujoco.MjModel.from_xml_path("mjx/right_mjx.xml")
mjx_model = mjx.put_model(model)
mjx_data = mjx.make_data(mjx_model)

@jax.jit
def step(model, data):
    return mjx.step(model, data)

for _ in range(1000):
    mjx_data = step(mjx_model, mjx_data)
```

### Isaac Sim

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

add_reference_to_stage("usd/right.usd", "/World/RightHand")
```

### ROS 2 and RViz

From a source checkout:

```bash
# Left hand (default)
ros2 launch launch/display.launch.py

# Right hand
ros2 launch launch/display.launch.py robot:=right

# Headless joint publisher
ros2 launch launch/display.launch.py use_gui:=false
```

After installing the ROS 2 package:

```bash
ros2 launch wuji_hand_description display.launch.py
ros2 launch wuji_hand_description display.launch.py robot:=right
```

## Notes

- `left.urdf` and `right.urdf` use relative mesh paths for direct loading from the repository.
- `left-ros.urdf` and `right-ros.urdf` use `package://wuji_hand_description/meshes/` paths for installed ROS 2 packages.
- `launch/display.launch.py` automatically falls back to source-tree URDF files when the package is not installed.
- `params.csv` is the compact actuator parameter table corresponding to the MuJoCo and MJX models.

## Model Specifications

| Parameter | Value |
|-----------|-------|
| Platform | WUJI Hand |
| Variants | Left and right |
| DOF | 20 per hand |
| Joint type | Revolute |
| Actuation | Position control (PD) |
| Included formats | URDF, MJCF, MJX, USD |

## Upgrading from v0.2.4

This release is a major update with corrected kinematics, calibrated dynamics, and new simulation formats.

### Breaking changes

| Item | v0.2.x | v1.x |
|------|--------|------|
| Launch argument | `hand:=left` | `robot:=left` |
| STEP files | Included in `step/` | Removed (contact support@wuji.tech) |
| USD structure | `usd/left/wujihand.usd` (monolithic) | `usd/left.usd` (modular mesh refs) |
| MuJoCo `armature` | 0.005 | 0.0002 |
| MuJoCo `damping` | 0.05 | 0.0 |
| MCP2 joint limits | Uniform ±0.37 rad | Per-finger, per-hand calibrated |

### New features

- **MJX models** (`mjx/`): JAX-optimized MuJoCo models for Brax and GPU-accelerated simulation.
- **params.csv**: Per-joint calibrated actuator gains (`kp`, `kv`), armature, damping, and control limits.
- **Chinese documentation** (`README_zh.md`).
- **Git LFS** (`.gitattributes`): Large binary assets tracked with Git LFS.
- **ROS URDF variants**: `left-ros.urdf` / `right-ros.urdf` with `package://` paths for installed packages.

### Kinematics changes

- **Joint axes corrected**: Finger 2–5 joint 2 axis changed from `-1 0 0` to `0 1 0`. Finger 1 joint 1 axis reversed for the left hand.
- **Joint limits re-calibrated**: MCP2 limits are now asymmetric and individually calibrated per finger and per hand, replacing the previous uniform ±0.37 rad range.
- If your control policy only involves joint 1/3/4 (grasping), the impact is minimal — fine-tune and verify on the new model. If your policy involves joint 2 (lateral movement), we recommend retraining on the new model for better sim-to-real performance.

### MuJoCo parameter changes

- Default `armature` reduced from 0.005 to 0.0002.
- Default `damping` reduced from 0.05 to 0.0.
- Actuator gains (`kp`, `kv`) are now per-joint calibrated; see [`params.csv`](params.csv).

## License

[MIT](LICENSE)

## Contact

For questions, contact [support@wuji.tech](mailto:support@wuji.tech).
