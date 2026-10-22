#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
PROTOCOL_PREFIX=/opt/agibot/share/ros2_package/aima_protocol_ros2_package
export CMAKE_PREFIX_PATH="$PROTOCOL_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PROTOCOL_PREFIX/lib:${LD_LIBRARY_PATH:-}"
BUILD_DIR="${A3_BRIDGE_BUILD_DIR:-build}"
cmake -S . -B "$BUILD_DIR" "$@"
cmake --build "$BUILD_DIR" -j2
