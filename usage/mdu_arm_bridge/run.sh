#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source /agibot/software/v0/entry/env/env.sh
PROTOCOL_PREFIX=/opt/agibot/share/ros2_package/aima_protocol_ros2_package
# ROS 运行库优先，避免固件目录中另一版本 Fast-CDR 覆盖 Jazzy 的依赖。
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$PROTOCOL_PREFIX/lib:/opt/agibot/bin:${LD_LIBRARY_PATH:-}"
# 固件可执行文件的 RPATH 会抢先加载旧 Fast-CDR；仅在本进程预加载 Jazzy 版本。
export LD_PRELOAD="/opt/ros/jazzy/lib/libfastcdr.so.2${LD_PRELOAD:+:$LD_PRELOAD}"
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=232
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/cfg/ros_dds_configuration.xml
export A3_ENABLE_COMMANDS="${A3_ENABLE_COMMANDS:-0}"
export A3_MDU_BRIDGE_LIBRARY="${A3_MDU_BRIDGE_LIBRARY:-$PWD/build/liba3_mdu_arm_bridge.so}"
if [[ "$(/opt/agibot/bin/aimrt_main --version)" != *"1.7.0"* ]]; then
  echo "ERROR: This bridge requires the firmware AimRT 1.7.0 runtime." >&2
  exit 1
fi
# 防止同一部署目录同时启动两个控制桥；进程退出后自动释放文件锁。
exec flock -n "$PWD/.bridge.lock" /opt/agibot/bin/aimrt_main --cfg_file_path="$PWD/bridge.yaml"
