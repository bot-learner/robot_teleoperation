#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

wget -c -O "${script_dir}/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb" \
  "https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"

wget -c -O "${script_dir}/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb" \
  "https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb"
