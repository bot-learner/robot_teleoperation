#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_source="${script_dir}/XRoboToolkit-PC-Service-Pybind"

if [[ ! -f "${sdk_source}/pyproject.toml" ]]; then
  echo "错误：未找到本地 Python SDK 源码：${sdk_source}" >&2
  exit 1
fi

# 官方未修改版本可使用：python3 -m pip install xrobotoolkit-sdk
# 本项目安装本地修正版，以支持任意数量的 Motion Tracker。
python3 -m pip install --force-reinstall "${sdk_source}"

python3 -c "import xrobotoolkit_sdk; print('xrobotoolkit_sdk 安装成功')"
