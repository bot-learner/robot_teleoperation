# 第零步：准备 A3 Ultra 遥操作环境

本文只包含当前仓库中“PICO 数据接收 → 双臂 IK → A3 Ultra 适配”所需的环境。所有命令均在项目根目录 `teleoperation_a3_ultra/` 下执行。

## 当前支持范围

- PC：Ubuntu 22.04 或 24.04，`amd64`；
- Python：最低 3.10，推荐使用 Conda 创建独立的 Python 3.11 环境；
- PICO 数据：XRoboToolkit PC Service 1.0.0 和本地修正版 `xrobotoolkit-sdk` 1.1.0；
- 双臂 IK：Pinocchio 3.x 和 NumPy；
- 仿真预览：MuJoCo 3.x。

当前仓库已经实现 `A3UltraInterface` 双臂 IK 和 MuJoCo 运动学预览，但尚未实现真机命令发送，因此还不能直接控制 A3 Ultra 真机。

ROS 2 以及 `third_party/EgoHumanoid` 的训练依赖都不是当前接口的必需项。

## 1. 安装系统工具

```bash
sudo apt update
sudo apt install -y build-essential git wget adb
```

这些工具分别用于编译 XRoboToolkit Python 扩展、下载模型和安装 PICO APK。PC Service 本身不安装在 Conda 环境中，安装方法见 [`02_start_pc_service.md`](02_start_pc_service.md)。

## 2. 创建 Conda 环境

推荐直接运行一键安装脚本，它会创建或复用 `a3_teleop` 环境，并完成本节和下一节的所有 Python 包安装：

```bash
./usage/setup_conda_env.sh
```

也可以按下面的命令手动安装。

```bash
conda create -n a3_teleop -c conda-forge \
  python=3.11 pip 'cmake>=3.26' ninja
conda activate a3_teleop
```

`xrobotoolkit-sdk` 的最低要求是 Python 3.10；它使用 CPython 3.10 Stable ABI 构建，因此同样兼容 Python 3.11。这里选择 3.11，是为了继续获得 Conda 软件包更新和安全修复，同时保持 NumPy、Pinocchio及后续机器人 SDK 的兼容性。不建议把依赖直接装进当前 Conda `base` 环境。

确认后续安装确实进入 `a3_teleop`：

```bash
which python
which python3
python --version
python -m pip --version
```

`python` 和 `python3` 都应指向 `.../envs/a3_teleop/`，Python 版本应为 `3.11.x`。

## 3. 安装 Python 包

先安装 IK 依赖：

```bash
python -m pip install -r \
  pico_teleop_interface/robot_interfaces/a3_ultra/requirements_ik.txt
```

该文件当前包含：

| 包 | 版本范围 | 用途 |
| --- | --- | --- |
| `pin` | `>=3.8,<4` | Pinocchio 运动学和雅可比计算；导入名是 `pinocchio` |
| `numpy` | `>=1.26` | 位姿、矩阵和关节数组计算 |
| `mujoco` | `>=3.1,<4` | A3 Ultra 双臂仿真预览 |

再安装仓库内的 XRoboToolkit Python SDK 修正版：

```bash
./pico_teleop_interface/third_party/install_xrobotoolkit_python_sdk.sh
```

安装脚本会使用当前环境的 `python3` 编译并安装本地 `xrobotoolkit-sdk` 1.1.0。它依赖 C++17、CMake 3.26 以上和 Ninja；这些已经在前两节准备好。项目使用本地修正版是为了取消官方 Python Binding 最多返回 3 个独立 Tracker 的限制。

不要同时使用 Conda 的 `pinocchio` 和 PyPI 的 `pin`，否则两套 C++ 依赖可能相互覆盖。本项目统一使用 `requirements_ik.txt` 中的 PyPI 方案。

## 4. 准备 A3 Ultra 模型

仓库已经包含模型时可以跳过。需要重新下载或更新时执行：

```bash
./pico_teleop_interface/robot_interfaces/a3_ultra/download_ik_resources.sh
```

当前默认模型为 `a3_ultra_t3d0`，双臂 IK 使用：

```text
pico_teleop_interface/robot_interfaces/a3_ultra/models/
  A3-A3U-robot-model/a3_ultra_t3d0/urdf/model_planning.urdf
```

## 5. 验证 Conda 环境

```bash
python - <<'PY'
from importlib.metadata import version
import numpy
import pinocchio
import mujoco
import xrobotoolkit_sdk

print("Python packages OK")
print("numpy:", numpy.__version__)
print("pin:", version("pin"))
print("mujoco:", mujoco.__version__)
print("xrobotoolkit-sdk:", version("xrobotoolkit-sdk"))
PY
```

导入成功只代表 Python 环境正确。读取到实时 PICO 数据还需要：

1. 安装并启动 PC Service；
2. PICO 和 PC 处于同一局域网；
3. PICO 中的 XRoboToolkit 显示 `WORKING` 并启用 `Send`。

## 依赖边界

| 组件 | 是否必需 | 安装位置 |
| --- | --- | --- |
| Python 3.11、pip、CMake、Ninja | 是 | Conda 环境 `a3_teleop` |
| `numpy`、`pin`、`mujoco`、`xrobotoolkit-sdk` | 是 | Conda 环境内的 pip |
| XRoboToolkit PC Service | 是 | 系统目录 `/opt/apps/roboticsservice/` |
| A3/A3 Ultra URDF、SRDF 和 meshes | 是 | 项目目录 |
| A3 Ultra 真机 SDK | 最终真机控制需要，当前未确定 | 待实现发送接口后补充 |
| ROS 2 | 当前不需要 | — |
| A3 Ultra MJCF 和 MuJoCo meshes | 仿真需要 | `a3_ultra/sim/` |
| EgoHumanoid/OpenPI 训练依赖 | 当前不需要 | — |
