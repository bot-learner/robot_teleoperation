#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
model_repo="${script_dir}/models/A3-A3U-robot-model"
pinocchio_repo="${script_dir}/references/pinocchio"

mkdir -p "${script_dir}/models" "${script_dir}/references"

if [[ -d "${model_repo}/.git" ]]; then
  git -C "${model_repo}" pull --ff-only
else
  git clone --depth 1 \
    https://github.com/AgibotTech/A3-A3U-robot-model.git \
    "${model_repo}"
fi

if [[ -d "${pinocchio_repo}/.git" ]]; then
  git -C "${pinocchio_repo}" pull --ff-only
else
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/stack-of-tasks/pinocchio.git \
    "${pinocchio_repo}"
fi

git -C "${pinocchio_repo}" sparse-checkout set --no-cone \
  /examples/inverse-kinematics.py \
  /LICENSE

echo "A3 Ultra 双臂 IK 参考材料已准备完成。"
