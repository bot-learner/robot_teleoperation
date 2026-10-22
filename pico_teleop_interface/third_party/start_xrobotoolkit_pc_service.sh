#!/usr/bin/env bash
set -euo pipefail

service_dir="/opt/apps/roboticsservice"
service_launcher="${service_dir}/runService.sh"
process_pattern='[R]oboticsServiceProcess'
mode="service"

usage() {
  cat <<'EOF'
用法：start_xrobotoolkit_pc_service.sh [选项]

选项：
  --service   仅启动核心服务（默认）
  --2d        启动核心服务和 Qt 二维演示界面
  --3d        启动核心服务和 Unity 三维演示界面
  --recorder  启动核心服务和数据录制界面
  -h, --help  显示帮助
EOF
}

if (( $# > 1 )); then
  usage >&2
  exit 2
fi

case "${1:-}" in
  ""|--service) mode="service" ;;
  --2d) mode="2d" ;;
  --3d) mode="3d" ;;
  --recorder) mode="recorder" ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    echo "错误：未知选项：$1" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ ! -x "${service_launcher}" ]]; then
  echo "错误：未找到 XRoboToolkit PC Service：${service_launcher}" >&2
  echo "请先安装与当前 Ubuntu 版本匹配的 roboticsservice 安装包。" >&2
  exit 1
fi

ensure_service() {
  if service_pids="$(pgrep -f "${process_pattern}" 2>/dev/null)"; then
    echo "XRoboToolkit PC Service 已经在运行，PID：${service_pids//$'\n'/, }"
    return
  fi

  echo "正在启动 XRoboToolkit PC Service..."
  bash "${service_launcher}"

  for _ in {1..20}; do
    if service_pids="$(pgrep -f "${process_pattern}" 2>/dev/null)"; then
      echo "启动成功，PID：${service_pids//$'\n'/, }"
      return
    fi
    sleep 0.1
  done

  echo "错误：启动命令已执行，但未检测到 RoboticsServiceProcess。" >&2
  exit 1
}

ensure_service

if [[ "${mode}" == "service" ]]; then
  exit 0
fi

case "${mode}" in
  2d)
    frontend="${service_dir}/RobotDemoQt"
    frontend_dir="${service_dir}"
    ;;
  3d)
    frontend="${service_dir}/SDKDemo/RobotUnityDemo/RobotLinuxDemo.x86_64"
    frontend_dir="${service_dir}/SDKDemo/RobotUnityDemo"
    ;;
  recorder)
    frontend="${service_dir}/RobotDataRecorder"
    frontend_dir="${service_dir}"
    ;;
esac

if [[ ! -r "${frontend}" ]]; then
  echo "错误：未找到对应界面程序：${frontend}" >&2
  exit 1
fi

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${service_dir}:${service_dir}/lib:${service_dir}/SDK/x64"
export QT_PLUGIN_PATH="${service_dir}/plugins/:${QT_PLUGIN_PATH:-}"
export QT_QML_PATH="${service_dir}/qml/:${QT_QML_PATH:-}"
cd "${frontend_dir}"

echo "正在启动 ${mode} 界面；关闭界面后当前命令才会退出。"
if [[ -x "${frontend}" ]]; then
  exec "${frontend}"
fi

# 官方安装包遗漏了部分 ELF 文件的执行位，交给动态加载器可避免修改 /opt。
loader="/lib64/ld-linux-x86-64.so.2"
if [[ ! -x "${loader}" ]]; then
  echo "错误：${frontend} 没有执行权限，且未找到兼容的动态加载器。" >&2
  exit 1
fi
exec "${loader}" "${frontend}"
