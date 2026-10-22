#!/usr/bin/env bash
# 发生错误、使用未定义变量或管道中的命令失败时立即退出，避免留下不完整的环境。
set -euo pipefail

# [1] 确定环境名称和项目根目录
# 核心命令：dirname、cd、pwd
# 用途：根据本脚本的位置计算项目根目录，使脚本可以从任意目录启动。
env_name="a3_teleop"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
echo "[1] Project root: ${project_root}"

# [2] 检查并加载 Conda
# 核心命令：command -v conda、conda info --base、source conda.sh
# 用途：确认 Conda 已安装，并在非交互式脚本中启用 conda activate 命令。
echo "[2] Checking and loading Conda."
if ! command -v conda >/dev/null 2>&1; then
  echo "Error: conda was not found. Install Miniconda or Anaconda first." >&2
  exit 1
fi

conda_base="$(conda info --base)"
conda_init="${conda_base}/etc/profile.d/conda.sh"
if [[ ! -f "${conda_init}" ]]; then
  echo "Error: Conda initialization script was not found: ${conda_init}" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${conda_init}"

# [3] 创建或更新 Conda 环境
# 核心命令：conda create / conda install
# 用途：准备 Python 3.11、pip、CMake 3.26 以上和 Ninja。
# 已存在同名环境时使用 conda install 校正版本，重复运行不会因环境已存在而失败。
echo "[3] Creating or updating Conda environment: ${env_name}."
if conda env list | awk -v name="${env_name}" '$1 == name { found=1 } END { exit !found }'; then
  echo "Conda environment ${env_name} already exists; updating its dependencies."
  conda install -y -n "${env_name}" -c conda-forge \
    python=3.11 pip 'cmake>=3.26' ninja
else
  conda create -y -n "${env_name}" -c conda-forge \
    python=3.11 pip 'cmake>=3.26' ninja
fi

# [4] 安装 ROS 2 Jazzy Python 通信依赖
# 核心命令：conda install ros-jazzy-rclpy ros-jazzy-sensor-msgs
# 用途：让 Python 3.11 的 a3_teleop 环境直接收发 A3 Ultra ROS 2 双臂话题。
echo "[4] Installing ROS 2 Jazzy Python dependencies."
conda install -y -n "${env_name}" --override-channels \
  -c conda-forge -c robostack-jazzy \
  ros-jazzy-rclpy ros-jazzy-sensor-msgs

# [5] 在本脚本中激活环境
# 核心命令：conda activate a3_teleop
# 用途：保证后续 python、python3 和 pip 都来自 a3_teleop，而不是 Conda base 或系统环境。
# 注意：子脚本不能改变父终端；脚本结束后，用户仍需在当前终端手动激活该环境。
echo "[5] Activating Conda environment: ${env_name}."
conda activate "${env_name}"

# [6] 检查实际使用的 Python
# 核心命令：which python、python --version、python -m pip --version
# 用途：显示解释器和 pip 的真实路径，及时发现依赖被装进错误环境的问题。
echo "[6] Checking Python and pip:"
which python
which python3
python --version
python -m pip --version

# [7] 切换到项目根目录
# 核心命令：cd "${project_root}"
# 用途：让下面的项目相对路径不受用户启动脚本时所在目录的影响。
echo "[7] Changing to the project root."
cd "${project_root}"

# [8] 安装 A3 Ultra 双臂 IK 依赖
# 核心命令：python -m pip install -r requirements_ik.txt
# 用途：安装 NumPy、Pinocchio 和 MuJoCo；PyPI 包名是 pin，Python 导入名是 pinocchio。
echo "[8] Installing NumPy, Pinocchio, and MuJoCo."
python -m pip install -r \
  pico_teleop_interface/robot_interfaces/a3_ultra/requirements_ik.txt

# [9] 安装 XRoboToolkit Python SDK
# 核心命令：install_xrobotoolkit_python_sdk.sh
# 用途：编译并安装仓库内的本地修正版，使 Python 可以读取 PICO 追踪数据，
#       并支持返回超过 3 个独立 Motion Tracker。
echo "[9] Installing the locally patched xrobotoolkit-sdk."
./pico_teleop_interface/third_party/install_xrobotoolkit_python_sdk.sh

# [10] 安装 SONIC CPU 推理依赖
# 核心命令：python -m pip install torch / scipy / PyYAML
# 用途：让 a3_teleop 直接运行默认的 SONIC 仿真入口，无需切换到独立 venv。
# CPU 版不依赖 NVIDIA 驱动；版本与本机已验证的 SONIC 环境一致。
# 此步骤不下载 SONIC 源码、模型权重或闭链求解器，资源准备见 sonic_for_a3.md。
echo "[10] Installing SONIC CPU inference dependencies."
python -m pip install 'torch==2.14.1+cpu' --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'scipy==1.17.1' 'PyYAML==6.0.3'

echo
echo "A3 Ultra teleoperation Python environment setup is complete."
echo "Before future use, run: conda activate ${env_name}"
