#!/usr/bin/env bash
set -euo pipefail

if ! command -v adb >/dev/null 2>&1; then
  echo "Error: adb was not found. Install it with: sudo apt install adb" >&2
  exit 1
fi

# [1] 显示已连接设备；首次连接需在 PICO 中允许 USB 调试。
echo "[1] Listing ADB devices."
adb devices

# [2] 仅选择 USB 设备，将 PICO 的 63901 端口转发到 PC Service。
echo "[2] Forwarding PICO port 63901 to PC port 63901 over USB."
if ! adb -d reverse tcp:63901 tcp:63901; then
  echo "Error: USB forwarding failed. Connect one USB device and authorize USB debugging in PICO." >&2
  exit 1
fi

# [3] 显示转发规则；规则存在不代表 PC Service 已启动或应用已连接。
echo "[3] Listing USB reverse port mappings."
adb -d reverse --list

echo "USB port forwarding is configured."
echo "Start PC Service, then set its address to 127.0.0.1 in the PICO XRoboToolkit app."
echo "Connect until WORKING, enable Head and Object Tracking, then enable Send."
echo "Run this script again if the mapping is lost after reconnecting or restarting PICO."
