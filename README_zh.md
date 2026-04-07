# wuji-hand-description

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-hand-description)](https://github.com/wuji-technology/wuji-hand-description/releases)

WUJI 灵巧手模型描述包，提供 MuJoCo、MJX、Isaac Sim 和 ROS 2 的仿真与可视化资源。仓库包含左右手模型、标定后的惯性与执行器参数、RViz 启动文件，以及适配 ROS 安装的 URDF 变体。

## 特性

- 左右手 20 自由度模型，支持 URDF、MJCF、MJX 和 USD 格式。
- 共享 mesh 资源，用于可视化、碰撞检测和仿真器导出。
- 标定后的执行器增益与限位参数，见 [`params.csv`](params.csv)。
- ROS 2 功能包，支持安装模式和源码模式下的 RViz 启动。

## 仓库结构

```text
wuji-hand-description/
├── .github/workflows/      # 发布自动化
├── launch/
│   └── display.launch.py   # RViz / robot_state_publisher 入口
├── meshes/                 # URDF 和 MuJoCo 使用的 STL 网格
│   ├── left/
│   └── right/
├── mjcf/                   # MuJoCo XML 模型
├── mjx/                    # MJX 优化的 MuJoCo 模型
├── urdf/                   # 源码树和 ROS 安装版 URDF
├── usd/                    # Isaac Sim USD 文件及引用的 USD 网格
├── rviz/
│   ├── left.rviz
│   └── right.rviz
├── params.csv              # 关节增益、限位和电机惯量参数
├── CHANGELOG.md
├── CMakeLists.txt
├── LICENSE
├── package.xml
└── README.md
```

## 格式指南

| 使用场景 | 入口文件 |
|----------|----------|
| 从源码目录直接加载 URDF | `urdf/left.urdf` 或 `urdf/right.urdf` |
| ROS 2 功能包安装后使用 | `urdf/left-ros.urdf` 或 `urdf/right-ros.urdf` |
| MuJoCo 仿真 | `mjcf/left.xml` 或 `mjcf/right.xml` |
| MJX / JAX 加速仿真 | `mjx/left_mjx.xml` 或 `mjx/right_mjx.xml` |
| Isaac Sim | `usd/left.usd` 或 `usd/right.usd` |

## 安装

### 稀疏检出（仅下载手部模型）

```bash
git clone --filter=blob:none --sparse https://github.com/wuji-technology/wuji-hand-description.git
cd wuji-hand-description
git sparse-checkout set meshes urdf mjcf mjx
```

### 完整克隆

```bash
git clone https://github.com/wuji-technology/wuji-hand-description.git
cd wuji-hand-description
```

### ROS 2 工作空间

```bash
cd ~/ros2_ws/src
git clone https://github.com/wuji-technology/wuji-hand-description.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wuji_hand_description
source install/setup.bash
```

## 使用示例

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

### ROS 2 与 RViz

从源码目录启动：

```bash
# 左手（默认）
ros2 launch launch/display.launch.py

# 右手
ros2 launch launch/display.launch.py robot:=right

# 无 GUI 模式
ros2 launch launch/display.launch.py use_gui:=false
```

安装 ROS 2 功能包后启动：

```bash
ros2 launch wuji_hand_description display.launch.py
ros2 launch wuji_hand_description display.launch.py robot:=right
```

## 说明

- `left.urdf` 和 `right.urdf` 使用相对路径引用 mesh，适合从仓库目录直接加载。
- `left-ros.urdf` 和 `right-ros.urdf` 使用 `package://wuji_hand_description/meshes/` 路径，适合 ROS 2 安装后使用。
- `launch/display.launch.py` 在未安装功能包时会自动回退到源码树中的 URDF 文件。
- `params.csv` 是与 MuJoCo 和 MJX 模型对应的执行器参数表。

## 模型参数

| 参数 | 值 |
|------|-----|
| 平台 | WUJI Hand |
| 型号 | 左手和右手 |
| 自由度 | 每只手 20 个 |
| 关节类型 | 旋转关节 |
| 驱动方式 | 位置控制（PD） |
| 包含格式 | URDF、MJCF、MJX、USD |

## 从 v0.2.4 升级

本版本为重大更新，包含运动学修正、动力学标定和新仿真格式支持。

### 破坏性变更

| 变更项 | v0.2.x | v1.x |
|--------|--------|------|
| Launch 参数 | `hand:=left` | `robot:=left` |
| STEP 文件 | 包含在 `step/` | 已移除（如需请联系 support@wuji.tech） |
| USD 结构 | `usd/left/wujihand.usd`（整体式） | `usd/left.usd`（模块化 mesh 引用） |
| MuJoCo `armature` | 0.005 | 0.0002 |
| MuJoCo `damping` | 0.05 | 0.0 |
| MCP2 关节限位 | 统一 ±0.37 rad | 按手指、按左右手分别标定 |

### 新增功能

- **MJX 模型**（`mjx/`）：JAX 优化的 MuJoCo 模型，支持 Brax 和 GPU 加速仿真。
- **params.csv**：按关节标定的执行器参数（`kp`、`kv`）、armature、damping 和控制限位。
- **中文文档**（`README_zh.md`）。
- **Git LFS**（`.gitattributes`）：大型二进制资源使用 Git LFS 追踪。
- **ROS URDF 变体**：`left-ros.urdf` / `right-ros.urdf`，使用 `package://` 路径。

### 运动学变更

- **关节轴方向修正**：Finger 2–5 的 joint 2 轴从 `-1 0 0` 修正为 `0 1 0`；左手 Finger 1 的 joint 1 轴方向反转。
- **关节限位重新标定**：MCP2 关节限位改为非对称值，按手指和左右手分别标定，替代旧版统一的 ±0.37 rad 范围。
- 如果您的控制策略仅涉及 joint 1/3/4（抓握动作），影响较小，可在新模型上微调验证。如果策略涉及 joint 2（侧摆动作），建议基于新模型重新训练，以获得更好的 sim-to-real 效果。

### MuJoCo 参数变更

- 默认 `armature` 从 0.005 降低至 0.0002。
- 默认 `damping` 从 0.05 降低至 0.0。
- 执行器增益（`kp`、`kv`）改为按关节分别标定，详见 [`params.csv`](params.csv)。

## 许可证

[MIT](LICENSE)

## 联系方式

如有问题，请联系 [support@wuji.tech](mailto:support@wuji.tech)。
