#!/usr/bin/env bash

set -euo pipefail

# 无论从哪个目录执行，都从本脚本所在的 usage/ 目录查找桥接程序。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_BRIDGE="$SCRIPT_DIR/a3_ultra_state_bridge.py"
LOCAL_DIAGNOSTIC_LOG="$SCRIPT_DIR/../pico_teleop_interface/robot_interfaces/arm_diagnostic_log.py"
ROBOT_HOST="agi@192.168.31.85"
REMOTE_BRIDGE="/agibot/data/home/agi/Desktop/a3_ultra_state_bridge.py"
REMOTE_BRIDGE_COMMAND="^/usr/bin/python3 ${REMOTE_BRIDGE}$"

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

# [1] HDU 跨 Wi-Fi/内网转发 ROS 消息；Iceoryx 控制桥在 MDU 单独运行。
log "[1/4] Preparing to upload the arm state and ROS relay bridge."
log "      Local file : $LOCAL_BRIDGE"
log "      Destination: $ROBOT_HOST:$REMOTE_BRIDGE"
log "      Enter the HDU password if SSH requests it."
scp "$LOCAL_BRIDGE" "$LOCAL_DIAGNOSTIC_LOG" "$ROBOT_HOST:$(dirname "$REMOTE_BRIDGE")/"
log "[1/4] Upload completed successfully."

# [2] 登录 HDU；不自动停止 motion_control，避免意外解除整机运控。
# [3] 在 HDU 上加载 ROS 2 环境并以前台方式运行桥接。
log "[2/4] Connecting to the HDU: $ROBOT_HOST"

# 无论正常退出、报错还是 Ctrl+C，都精确终止本次远端桥接进程。
cleanup_remote_bridge() {
  local status=$?
  trap - EXIT
  log "      Stopping the remote bridge process..."
  if ssh -o BatchMode=yes "$ROBOT_HOST" \
      "pkill -TERM -f '$REMOTE_BRIDGE_COMMAND' 2>/dev/null || true"; then
    log "      Remote bridge cleanup completed."
  else
    log "WARNING: Remote cleanup failed; the next start will remove stale bridges."
  fi
  exit "$status"
}
trap cleanup_remote_bridge EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

ssh "$ROBOT_HOST" 'bash -s' <<'HDU_COMMANDS'
set -euo pipefail

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

log "[2/4] HDU connection established."
log "[3/4] Loading the A3 Ultra ROS 2 environment on the HDU."
# ROS 2 的 setup.bash 会读取部分可选环境变量，加载期间不能启用 nounset。
set +u
source /opt/ros/jazzy/setup.bash
source /agibot/software/v0/entry/env/env.sh
LOW_LEVEL_SETUP=/agibot/a3_deploy/setup_ros2_msgs.bash
if [[ -f "$LOW_LEVEL_SETUP" ]]; then
  source "$LOW_LEVEL_SETUP"
  log "      Loaded optional low-level message environment: $LOW_LEVEL_SETUP"
fi
set -u
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=232
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/cfg/ros_dds_configuration.xml
log "[3/4] ROS 2 environment loaded."
log "      ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
log "      RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
log "      DDS profile=$FASTRTPS_DEFAULT_PROFILES_FILE"

# ros2 可能使用专用 Python；直接调用系统默认 python3 会找不到 rclpy。
ROS2_EXECUTABLE="$(command -v ros2 || true)"
if [[ -z "$ROS2_EXECUTABLE" ]]; then
  log "ERROR: The ros2 executable was not found after loading the environment."
  exit 1
fi
read -r -a ROS_PYTHON <<< "$(head -n 1 "$ROS2_EXECUTABLE")"
ROS_PYTHON[0]="${ROS_PYTHON[0]#\#!}"
log "      ROS 2 executable=$ROS2_EXECUTABLE"
log "      ROS 2 Python=${ROS_PYTHON[*]}"
# HDU 固件自带消息包，但其 setup 脚本未必加入正确的 Python 版本目录。
# 优先复用固件协议定义，无需部署整个 low-level 策略程序。
PROTOCOL_PREFIX=/opt/agibot/share/ros2_package/aima_protocol_ros2_package
ROS_PYTHON_VERSION="$("${ROS_PYTHON[@]}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
PROTOCOL_PYTHON="$PROTOCOL_PREFIX/lib/python$ROS_PYTHON_VERSION/site-packages"
if [[ -d "$PROTOCOL_PYTHON/joint_msgs" ]]; then
  export PYTHONPATH="$PROTOCOL_PYTHON:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="$PROTOCOL_PREFIX/lib:${LD_LIBRARY_PATH:-}"
  export AMENT_PREFIX_PATH="$PROTOCOL_PREFIX:${AMENT_PREFIX_PATH:-}"
  log "      Using firmware joint_msgs: $PROTOCOL_PYTHON/joint_msgs"
fi
if ! "${ROS_PYTHON[@]}" -c \
    'import rclpy; from sensor_msgs.msg import JointState; from joint_msgs.msg import JointState as LowLevelJointState; from rosidl_generator_py import import_type_support; import_type_support("joint_msgs")'; then
  log "ERROR: ROS 2 Python cannot import rclpy, sensor_msgs, or joint_msgs."
  log "      Checked firmware protocol path: $PROTOCOL_PYTHON"
  log "      Optional custom message setup: $LOW_LEVEL_SETUP"
  exit 1
fi
log "      ROS 2 Python dependencies verified."

BRIDGE_COMMAND="^/usr/bin/python3 /agibot/data/home/agi/Desktop/a3_ultra_state_bridge.py$"
OLD_BRIDGE_PIDS="$(pgrep -f "$BRIDGE_COMMAND" || true)"
if [[ -n "$OLD_BRIDGE_PIDS" ]]; then
  log "      Stopping stale bridge process(es): ${OLD_BRIDGE_PIDS//$'\n'/, }"
  # PID 列表只来自上面的精确命令行匹配。
  kill $OLD_BRIDGE_PIDS
  for _ in {1..20}; do
    pgrep -f "$BRIDGE_COMMAND" >/dev/null || break
    sleep 0.1
  done
  if pgrep -f "$BRIDGE_COMMAND" >/dev/null; then
    log "ERROR: A stale bridge did not stop; refusing to start another instance."
    exit 1
  fi
fi

log "[4/4] Starting the arm state bridge: /agibot/data/home/agi/Desktop/a3_ultra_state_bridge.py"
log "      The bridge runs in the foreground. Press Ctrl+C to stop it."
exec "${ROS_PYTHON[@]}" /agibot/data/home/agi/Desktop/a3_ultra_state_bridge.py
HDU_COMMANDS
