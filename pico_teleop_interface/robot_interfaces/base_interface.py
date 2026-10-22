"""基于 XRoboToolkit SDK 的机器人遥操作基础接口。

本模块只负责机器人无关的公共流程：

1. 启动并初始化 XRoboToolkit PC Service 和 Python SDK；
2. 从 SDK 读取头显、控制器、手部、人体或 Motion Tracker 数据；
3. 为位置和旋转提供统一的低通滤波入口；
4. 按固定频率执行“采样 → 解算 → 下发”的控制循环；
5. 异步记录程序生命周期、跟踪状态、性能指标、数据快照和异常。

坐标转换、左右 Tracker 绑定、逆运动学以及机器人通信协议都由具体机器人的
子类实现。例如 A3 Ultra 的实现位于 ``a3_ultra/a3_ultra_interface.py``。
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
from pathlib import Path
import ctypes
import logging
import math
import multiprocessing
import os
import queue
import re
import subprocess
from typing import Any
import uuid

import numpy as np
import xrobotoolkit_sdk

from .filters import (
    RotationSpaceLowPassFilter,
    SecondOrderButterworthLowPassFilter,
)


class TrackingMode(str, Enum):
    """与 PICO XRoboToolkit App 中发送模式对应的追踪数据类型。

    两种模式是互斥的：全身模式读取人体关节点，物体模式读取独立的
    Motion Tracker。头显、控制器和手部状态不受该枚举限制，每帧都会读取。
    """

    # PICO App 选择 Full Body 时使用，主要数据在 body_pose 中。
    WHOLE_BODY = "whole_body"
    # PICO App 选择 Object Tracking 时使用，主要数据在 motion_tracker_* 中。
    OBJECT = "object"


@dataclass(frozen=True)
class TeleoperationInput:
    """一次 SDK 采样得到的完整 PICO 遥操作输入快照。

    该对象使用 ``frozen=True``，防止下游在解算过程中意外替换字段。字段内部
    如果是 NumPy 数组，其内容仍可修改，因此机器人实现需要自行决定是否复制。

    SDK 的姿态通常为 ``[x, y, z, qx, qy, qz, qw]``，但这里保留 ``Any``，
    是因为 XRoboToolkit 对头显、手部和人体数据返回的结构并不完全一致。具体
    机器人必须在使用前检查形状、有限值和四元数是否合法。
    """

    # 本机完成本次读取时的单调时钟，不是 PICO 设备端采样时间戳。
    captured_at_ns: int
    # 本帧采用的追踪模式，供下游确认应该读取哪组可选字段。
    tracking_mode: TrackingMode
    # 头显及左右控制器的当前姿态，格式由 XRoboToolkit SDK 定义。
    headset_pose: Any
    left_controller_pose: Any
    right_controller_pose: Any
    # 左右手追踪状态，可能包含关节点、置信度等 SDK 原始信息。
    left_hand_state: Any
    right_hand_state: Any
    # 全身模式的数据；没有可用人体数据时 body_pose 为 None。
    body_pose: Any | None
    body_available: bool
    # Object Tracking 模式的数据；没有 Tracker 时这些字段为 None。
    motion_tracker_pose: Any | None
    motion_tracker_velocity: Any | None
    motion_tracker_acceleration: Any | None
    # 序列号用于稳定绑定左右设备，不能依赖 SDK 返回数组的先后顺序。
    motion_tracker_serial_numbers: Any | None
    motion_tracker_count: int
    # PICO Motion Tracker 数据帧时间戳；用于区分数据停滞和本机计算阻塞。
    motion_timestamp_ns: int | None
    # SDK 报告的左右手追踪是否处于活动状态。
    left_hand_active: bool
    right_hand_active: bool
    # PICO 整包源时间戳；部分 APK 的 Motion 子包时间戳恒为 0，需用它识别旧缓存。
    pico_timestamp_ns: int | None = None


class TeleoperationMonitor:
    """在屏幕右下角显示遥操帧率和数据状态。"""

    WIDTH = 390
    HEIGHT = 300
    MARGIN = 12
    # 连续 1 秒没有观察到源时间戳推进时，窗口和文件日志中的遥操 FPS 归零。
    STALE_AFTER_SECONDS = 1.0

    def __init__(
        self, target_rate_hz: float, warning_fps: float,
        metrics_interval_seconds: float,
    ) -> None:
        self.target_rate_hz = target_rate_hz
        self.warning_fps = warning_fps
        self.metrics_interval_seconds = metrics_interval_seconds
        self.mujoco_window_count = 1
        context = multiprocessing.get_context("spawn")
        self._updates = context.Queue(maxsize=1)
        self._process: multiprocessing.Process | None = None
        self._window_started = time.monotonic()
        self._received_frame = False
        self._sample_count = self._valid_count = 0
        self._sample_fps = self._valid_fps = self._elapsed_ms = 0.0
        self._sdk_read_total_ms = self._ik_total_ms = 0.0
        self._sdk_read_max_ms = self._ik_max_ms = 0.0
        self._timed_sample_count = 0
        self._tracker_count = 0
        self._last_valid_at: float | None = None
        self._last_source_timestamp_ns: int | None = None
        self._status = "STARTING"
        self._robot_feedback_at = None
        self._robot_feedback_warning_seconds = .25
        self._bridge_heartbeat_at = None
        self._bridge_heartbeat_warning_seconds = 1.0
        self._calibration_deadline = None
        self._stage = "STARTING"

    def set_stage(self, stage: str) -> None:
        self._stage = stage
        self._publish()

    def set_robot_feedback(
        self, received_at: float, warning_seconds: float,
        bridge_heartbeat_at: float = 0.0,
        bridge_heartbeat_warning_seconds: float = 1.0,
    ) -> None:
        """显示真机反馈与 MDU 心跳的接收间隔，不据此中止控制。"""
        self._robot_feedback_at = received_at or None
        self._robot_feedback_warning_seconds = warning_seconds
        self._bridge_heartbeat_at = bridge_heartbeat_at or None
        self._bridge_heartbeat_warning_seconds = bridge_heartbeat_warning_seconds
        self._publish()

    def set_calibration_countdown(self, seconds: float) -> None:
        self._stage = "CALIBRATION"
        self._calibration_deadline = time.monotonic() + seconds
        self._publish()

    def start(self) -> None:
        """启动独立 GUI 进程，避免 Tk 事件循环阻塞机器人控制。"""

        if self._process is None or not self._process.is_alive():
            context = multiprocessing.get_context("spawn")
            self._process = context.Process(
                target=self._run,
                args=(self._updates, self.target_rate_hz, self.warning_fps,
                      os.getpid(), self.mujoco_window_count),
                name="teleoperation-monitor", daemon=True,
            )
            self._process.start()
            self._publish()

    def update(
        self, tracking_input: TeleoperationInput, command_available: bool,
        elapsed_seconds: float, sdk_read_seconds: float | None = None,
        ik_seconds: float | None = None,
    ) -> dict[str, float | int | str] | None:
        """只将源时间戳变化计为新帧；重复读取 SDK 缓存只计入循环帧率。"""

        now = time.monotonic()
        if (command_available and self._calibration_deadline is not None
                and now >= self._calibration_deadline):
            self._calibration_deadline = None
        if not self._received_frame:
            self._window_started = now
            self._received_frame = True
        self._sample_count += 1
        has_tracking = (tracking_input.body_available
                        if tracking_input.tracking_mode is TrackingMode.WHOLE_BODY
                        else tracking_input.motion_tracker_count > 0)
        source_timestamp_ns = tracking_input.motion_timestamp_ns
        if (tracking_input.tracking_mode is TrackingMode.WHOLE_BODY
                or not source_timestamp_ns or source_timestamp_ns < 0):
            # shortcut: 整包时间戳只能证明整包在更新；需识别单独 Tracker 停流时应升级 SDK 元数据。
            source_timestamp_ns = tracking_input.pico_timestamp_ns
        if has_tracking and source_timestamp_ns is not None and source_timestamp_ns > 0:
            if source_timestamp_ns != self._last_source_timestamp_ns:
                self._valid_count += 1
                self._last_valid_at = tracking_input.captured_at_ns / 1e9
            self._last_source_timestamp_ns = source_timestamp_ns
        self._elapsed_ms = elapsed_seconds * 1000.0
        if sdk_read_seconds is not None and ik_seconds is not None:
            sdk_read_ms = sdk_read_seconds * 1000.0
            ik_ms = ik_seconds * 1000.0
            self._sdk_read_total_ms += sdk_read_ms
            self._ik_total_ms += ik_ms
            self._sdk_read_max_ms = max(self._sdk_read_max_ms, sdk_read_ms)
            self._ik_max_ms = max(self._ik_max_ms, ik_ms)
            self._timed_sample_count += 1
        self._tracker_count = tracking_input.motion_tracker_count
        stale = (self._last_valid_at is None
                 or now - self._last_valid_at >= self.STALE_AFTER_SECONDS)
        self._status = ("NO TRACKING DATA" if not has_tracking else
                        "NO FRESH PICO DATA" if stale else
                        "WORKING" if command_available else "NO COMMAND")
        duration = now - self._window_started
        metrics = None
        if duration >= self.metrics_interval_seconds:
            self._sample_fps = self._sample_count / duration
            self._valid_fps = self._valid_count / duration
            if stale:
                self._valid_fps = 0.0
            self._sample_count = self._valid_count = 0
            self._window_started = now
            metrics = {
                "status": self._status,
                "teleop_fps": round(self._valid_fps, 2),
                "read_loop_fps": round(self._sample_fps, 2),
                "read_ik_ms": round(self._elapsed_ms, 3),
                "tracker_count": self._tracker_count,
            }
            if self._timed_sample_count:
                metrics.update(
                    sdk_read_avg_ms=round(
                        self._sdk_read_total_ms / self._timed_sample_count, 3),
                    sdk_read_max_ms=round(self._sdk_read_max_ms, 3),
                    ik_avg_ms=round(
                        self._ik_total_ms / self._timed_sample_count, 3),
                    ik_max_ms=round(self._ik_max_ms, 3),
                )
            self._sdk_read_total_ms = self._ik_total_ms = 0.0
            self._sdk_read_max_ms = self._ik_max_ms = 0.0
            self._timed_sample_count = 0
        self._publish()
        if metrics is not None:
            return metrics
        return None

    def close(self) -> None:
        """请求 GUI 进程退出，超时时强制回收而不影响控制进程。"""

        if self._process is None:
            return
        try:
            while True:
                self._updates.get_nowait()
        except queue.Empty:
            pass
        self._updates.put(None)
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join()
        self._process = None

    def _snapshot(self) -> tuple:
        age = ("--" if self._last_valid_at is None else
               f"{time.monotonic() - self._last_valid_at:.2f} s")
        summary = (
            f"Stage           {self._stage}\n"
            f"Status          {self._status}\n"
            f"Teleop FPS      {self._valid_fps:6.1f} / {self.target_rate_hz:g}\n"
            f"Read-loop FPS   {self._sample_fps:6.1f}\n"
            f"Read + IK       {self._elapsed_ms:6.2f} ms\n"
            f"Trackers        {self._tracker_count}\n"
            f"Last valid age  {age}"
        )
        return (summary, self._valid_fps, self._last_valid_at,
                self._robot_feedback_at, self._robot_feedback_warning_seconds,
                self._bridge_heartbeat_at, self._bridge_heartbeat_warning_seconds,
                self._calibration_deadline)

    def _publish(self) -> None:
        if self._process is None or not self._process.is_alive():
            return
        try:
            self._updates.put_nowait(self._snapshot())
        except queue.Full:
            try:
                self._updates.get_nowait()
            except queue.Empty:
                pass
            try:
                self._updates.put_nowait(self._snapshot())
            except queue.Full:
                pass

    @staticmethod
    def _run(updates: Any, target_rate_hz: float, warning_fps: float,
             owner_pid: int, mujoco_window_count: int) -> None:
        try:
            import tkinter as tk

            root = tk.Tk()
            root.title("PICO Teleoperation Monitor")
            root.resizable(False, False)
            root.attributes("-topmost", True)
            root.configure(bg="#171a21")
            # 额外留出窗口边框和标题栏空间，避免被屏幕边缘裁切。
            left, top, width, height = TeleoperationMonitor._desktop_workarea(
                root.winfo_screenwidth(), root.winfo_screenheight())
            x = left + width - TeleoperationMonitor.WIDTH - 2 * TeleoperationMonitor.MARGIN
            y = top + height - TeleoperationMonitor.HEIGHT - 5 * TeleoperationMonitor.MARGIN
            root.geometry(
                f"{TeleoperationMonitor.WIDTH}x{TeleoperationMonitor.HEIGHT}+{x}+{y}")

            summary = tk.Text(
                root, height=6, font=("monospace", 11), borderwidth=0,
                fg="#d9e1ea", bg="#171a21", padx=12, pady=10,
            )
            summary.pack(fill="both", expand=True)
            summary.tag_configure("low_fps", foreground="#ff4d4d")
            summary.tag_configure("countdown", foreground="#ffd166", font=("monospace", 16, "bold"))
            root.protocol("WM_DELETE_WINDOW", root.withdraw)

            arranged = False
            current = (
                f"Status          STARTING\n"
                f"Teleop FPS         0.0 / {target_rate_hz:g}\n"
                f"Read-loop FPS      0.0\n"
                f"Read + IK          0.00 ms\n"
                f"Trackers        0\n"
                f"Last valid age  --",
                0.0,
                None,
                None, .25, None, 1.0, None,
            )

            def refresh() -> None:
                nonlocal arranged, current
                try:
                    while True:
                        update = updates.get_nowait()
                        if update is None:
                            root.destroy()
                            return
                        current = update
                except queue.Empty:
                    pass
                (text, teleop_fps, last_valid_at, feedback_at,
                 feedback_warning, heartbeat_at, heartbeat_warning,
                 calibration_deadline) = current
                # 主进程即使卡在 IK 中，独立窗口也不能一直显示上次的非零帧率。
                age = None if last_valid_at is None else time.monotonic() - last_valid_at
                if age is None or age >= TeleoperationMonitor.STALE_AFTER_SECONDS:
                    teleop_fps = 0.0
                    text = re.sub(r"(?m)^Status +.*$", "Status          NO FRESH PICO DATA", text)
                text = re.sub(
                    r"(?m)^Teleop FPS +.*$",
                    f"Teleop FPS      {teleop_fps:6.1f} / {target_rate_hz:g}", text)
                text = re.sub(
                    r"(?m)^Last valid age +.*$",
                    "Last valid age  " + ("--" if age is None else f"{age:.2f} s"), text)
                feedback_age = None if feedback_at is None else max(0., time.monotonic() - feedback_at)
                text += "\nArm feedback age " + (
                    "--" if feedback_age is None else f"{feedback_age * 1000:.0f} ms")
                heartbeat_age = (None if heartbeat_at is None else
                                 max(0., time.monotonic() - heartbeat_at))
                text += "\nMDU heartbeat age " + (
                    "--" if heartbeat_age is None else f"{heartbeat_age * 1000:.0f} ms")
                if calibration_deadline is not None:
                    remaining = max(0, math.ceil(calibration_deadline - time.monotonic()))
                    text += (f"\nCALIBRATION IN {remaining} s" if remaining
                             else "\nCalibration: waiting for PICO")
                summary.configure(state="normal")
                summary.delete("1.0", "end")
                for line in text.splitlines(keepends=True):
                    if line.startswith("Teleop FPS"):
                        prefix, value = line[:16], line[16:]
                        summary.insert("end", prefix)
                        summary.insert(
                            "end", value,
                            "low_fps" if teleop_fps < warning_fps else (),
                        )
                    elif line.startswith("Arm feedback age"):
                        summary.insert("end", line, "low_fps" if feedback_age is not None
                                       and feedback_age > feedback_warning else ())
                    elif line.startswith("MDU heartbeat age"):
                        summary.insert("end", line, "low_fps" if heartbeat_age is not None
                                       and heartbeat_age > heartbeat_warning else ())
                    elif line.startswith("CALIBRATION"):
                        summary.insert("end", line, "countdown")
                    else:
                        summary.insert("end", line)
                summary.configure(state="disabled")
                if not arranged:
                    arranged = TeleoperationMonitor._arrange_mujoco_window(
                        root.winfo_screenwidth(), root.winfo_screenheight(),
                        owner_pid, mujoco_window_count)
                root.after(100, refresh)

            refresh()
            root.mainloop()
        except KeyboardInterrupt:
            pass
        except Exception as exc:
            print(f"Teleoperation monitor unavailable: {exc}")

    @staticmethod
    def _desktop_workarea(screen_width: int, screen_height: int) -> tuple[int, int, int, int]:
        """读取当前桌面可用区域，避开顶部面板及侧边 Dock。"""

        try:
            properties = subprocess.run(
                ["xprop", "-root", "_NET_CURRENT_DESKTOP", "_NET_WORKAREA"],
                capture_output=True, text=True, check=True, timeout=1.0,
            ).stdout
            desktop = re.search(r"_NET_CURRENT_DESKTOP.*=\s*(\d+)", properties)
            area = re.search(r"_NET_WORKAREA.*=\s*([\d, ]+)", properties)
            if desktop and area:
                values = [int(value) for value in area.group(1).split(",")]
                offset = 4 * int(desktop.group(1))
                rectangle = tuple(values[offset:offset + 4])
                if len(rectangle) == 4 and min(rectangle[2:]) > 0:
                    return rectangle
        except (OSError, subprocess.SubprocessError, ValueError):
            pass
        return 0, 0, screen_width, screen_height

    @staticmethod
    def _arrange_mujoco_window(
        screen_width: int, screen_height: int, owner_pid: int,
        expected_count: int = 1,
    ) -> bool:
        """X11 下将本进程及其子进程的 MuJoCo 窗口排列在监视器左侧。"""

        try:
            tree = subprocess.run(
                ["xwininfo", "-root", "-tree"], capture_output=True,
                text=True, check=True, timeout=1.0,
            ).stdout
            candidates = re.findall(r"^\s*(0x[0-9a-f]+).*mujoco", tree,
                                    flags=re.IGNORECASE | re.MULTILINE)
            process_rows = subprocess.run(
                ["ps", "-eo", "pid=,ppid="], capture_output=True,
                text=True, check=True, timeout=1.0,
            ).stdout.splitlines()
            parents = {int(pid): int(ppid) for pid, ppid in
                       (row.split() for row in process_rows)}
            descendants = {owner_pid}
            while True:
                added = {pid for pid, ppid in parents.items()
                         if ppid in descendants} - descendants
                if not added:
                    break
                descendants.update(added)
            window_ids = []
            window_pids = {}
            for candidate in candidates:
                properties = subprocess.run(
                    ["xprop", "-id", candidate, "_NET_WM_PID"],
                    capture_output=True, text=True, timeout=1.0,
                ).stdout
                match = re.search(r"=\s*(\d+)", properties)
                if match and int(match.group(1)) in descendants:
                    window_ids.append(candidate)
                    window_pids[candidate] = int(match.group(1))
            if len(window_ids) < expected_count:
                return False
            # 两个实例标题相同；按先后启动的进程排列，IK 在左、SONIC 在右。
            window_ids.sort(key=window_pids.__getitem__)
            library = ctypes.cdll.LoadLibrary("libX11.so.6")
            library.XOpenDisplay.restype = ctypes.c_void_p
            display = library.XOpenDisplay(None)
            if not display:
                return False
            try:
                class ClientMessage(ctypes.Structure):
                    _fields_ = [
                        ("type", ctypes.c_int), ("serial", ctypes.c_ulong),
                        ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
                        ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                        ("format", ctypes.c_int), ("data", ctypes.c_long * 5),
                    ]

                library.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
                library.XDefaultRootWindow.restype = ctypes.c_ulong
                library.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
                library.XInternAtom.restype = ctypes.c_ulong
                root_window = library.XDefaultRootWindow(display)

                def send(window_id: str, name: str, values: tuple[int, ...]) -> None:
                    message = ClientMessage(
                        type=33, display=display, window=int(window_id, 16),
                        message_type=library.XInternAtom(display, name.encode(), 0),
                        format=32, data=(ctypes.c_long * 5)(*values))
                    event = ctypes.create_string_buffer(24 * ctypes.sizeof(ctypes.c_long))
                    ctypes.memmove(event, ctypes.byref(message), ctypes.sizeof(message))
                    library.XSendEvent(ctypes.c_void_p(display),
                                       ctypes.c_ulong(root_window), 0,
                                       (1 << 19) | (1 << 20), event)

                left, top, area_width, area_height = TeleoperationMonitor._desktop_workarea(
                    screen_width, screen_height)
                gap = TeleoperationMonitor.MARGIN
                available_width = area_width - TeleoperationMonitor.WIDTH - 5 * gap
                slot_width = available_width // expected_count
                for index, window_id in enumerate(window_ids[:expected_count]):
                    # 请求窗口管理器取消最大化，再按含边框的槽位分配客户区。
                    send(window_id, "_NET_WM_STATE", (
                        0, library.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_VERT", 0),
                        library.XInternAtom(display, b"_NET_WM_STATE_MAXIMIZED_HORZ", 0), 1, 0))
                    extents = subprocess.run(
                        ["xprop", "-id", window_id, "_NET_FRAME_EXTENTS"],
                        capture_output=True, text=True, timeout=1.0).stdout
                    match = re.search(r"=\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)", extents)
                    border_left, border_right, border_top, border_bottom = (
                        map(int, match.groups()) if match else (8, 8, 40, 8))
                    # Mutter 的阴影未包含在 _NET_FRAME_EXTENTS 内，另留 16 像素。
                    shadow = 16
                    border_left += shadow
                    border_right += shadow
                    border_top += shadow
                    border_bottom += shadow
                    x = left + gap + index * slot_width
                    y = top + gap
                    width = max(1, slot_width - gap - border_left - border_right)
                    height = max(1, area_height - 2 * gap - border_top - border_bottom)
                    # StaticGravity + x/y/width/height + 应用请求来源。
                    send(window_id, "_NET_MOVERESIZE_WINDOW", (
                        10 | (15 << 8) | (1 << 12),
                        x + border_left, y + border_top, width, height))
                library.XFlush(ctypes.c_void_p(display))
            finally:
                library.XCloseDisplay(ctypes.c_void_p(display))
            return True
        except (OSError, subprocess.SubprocessError):
            return False


class BaseRobotInterface(ABC):
    """所有机器人遥操作接口共享的生命周期和控制循环。

    这是抽象基类，不能直接实例化。子类至少需要实现：

    - :meth:`solve_inverse_kinematics`：把一帧 PICO 输入变成机器人命令；
    - :meth:`send_to_robot`：把机器人命令送往仿真或真实硬件。

    ``sdk`` 参数主要用于依赖注入。正常运行时保持为 ``None``，类会加载真实
    SDK；离线工具可以传入实现了相同方法的替代对象。
    """

    def __init__(
        self,
        control_rate_hz: float = 50.0,
        tracking_mode: TrackingMode | str = TrackingMode.WHOLE_BODY,
        auto_start_pc_service: bool = True,
        sdk: Any | None = None,
        low_pass_cutoff_hz: float = 10.0,
        show_monitor: bool = True,
        monitor_warning_fps: float = 25.0,
        enable_file_log: bool = True,
        log_directory: str | Path | None = None,
        log_level: str = "INFO",
        log_metrics_interval_seconds: float = 1.0,
        log_max_bytes: int = 40 * 1024 * 1024,
        log_backup_count: int = 4,
        log_data_interval_seconds: float = 1.0,
    ) -> None:
        # 控制频率同时决定循环周期和数字滤波器的采样频率，必须保持一致。
        if control_rate_hz <= 0:
            raise ValueError("control_rate_hz 必须大于 0")
        if monitor_warning_fps <= 0:
            raise ValueError("monitor_warning_fps 必须大于 0")
        if log_metrics_interval_seconds <= 0:
            raise ValueError("log_metrics_interval_seconds 必须大于 0")
        if log_data_interval_seconds <= 0:
            raise ValueError("log_data_interval_seconds 必须大于 0")
        if log_max_bytes <= 0 or log_backup_count < 0:
            raise ValueError("日志大小必须大于 0，备份数量不能为负数")
        numeric_log_level = getattr(logging, log_level.upper(), None)
        if not isinstance(numeric_log_level, int):
            raise ValueError(f"无效日志等级：{log_level}")

        self.control_rate_hz = control_rate_hz
        # 接受字符串是为了方便命令行或配置文件调用，内部统一保存为枚举。
        try:
            self.tracking_mode = TrackingMode(tracking_mode)
        except ValueError as exc:
            choices = ", ".join(mode.value for mode in TrackingMode)
            raise ValueError(f"tracking_mode 必须是以下值之一：{choices}") from exc

        # 每个控制周期允许使用的墙钟时间，例如 50 Hz 对应 20 ms。
        self._period_seconds = 1.0 / control_rate_hz
        # 平移和普通数值使用二阶巴特沃斯滤波；姿态不能直接滤矩阵元素，
        # 因而使用专门的旋转空间滤波器以保证输出仍是合法旋转矩阵。
        self.low_pass_filter = SecondOrderButterworthLowPassFilter(
            sample_rate_hz=control_rate_hz,
            cutoff_hz=low_pass_cutoff_hz,
        )
        self.rotation_low_pass_filter = RotationSpaceLowPassFilter(
            sample_rate_hz=control_rate_hz,
            cutoff_hz=low_pass_cutoff_hz,
        )
        # 保存启动策略和 SDK 实例，但构造阶段不连接服务，避免构造对象产生 I/O。
        self._auto_start_pc_service = auto_start_pc_service
        self._sdk = sdk if sdk is not None else xrobotoolkit_sdk
        self.monitor = TeleoperationMonitor(
            control_rate_hz, monitor_warning_fps, log_metrics_interval_seconds)
        self._show_monitor = show_monitor
        # _initialized 管理 SDK 生命周期；_running 管理控制循环生命周期。
        self._initialized = False
        self._running = False
        self._tracking_available: bool | None = None
        self._log_data_interval_seconds = log_data_interval_seconds
        self._last_data_log_at: float | None = None
        self._session_id = uuid.uuid4().hex[:8]
        self._session_started_at = time.monotonic()
        self._log_listener: QueueListener | None = None
        self._log_handler: RotatingFileHandler | None = None
        self._logger: logging.Logger | None = None
        self.log_path: Path | None = None
        if enable_file_log:
            try:
                state_root = Path(
                    os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
                directory = Path(log_directory).expanduser() if log_directory else (
                    state_root / "pico_teleop")
                directory.mkdir(parents=True, exist_ok=True)
                self.log_path = directory / "teleoperation.log"
                file_handler = RotatingFileHandler(
                    self.log_path, maxBytes=log_max_bytes,
                    backupCount=log_backup_count, encoding="utf-8")
                file_handler.setFormatter(logging.Formatter(
                    "%(asctime)s %(levelname)-8s %(message)s"))
                self._log_handler = file_handler
                log_queue: queue.SimpleQueue = queue.SimpleQueue()
                self._logger = logging.getLogger(
                    f"pico_teleop.{self._session_id}.{id(self)}")
                self._logger.handlers = [QueueHandler(log_queue)]
                self._logger.setLevel(numeric_log_level)
                self._logger.propagate = False
                self._log_listener = QueueListener(
                    log_queue, file_handler, respect_handler_level=True)
                self._log_listener.start()
            except OSError as exc:
                print(f"File logging unavailable: {exc}")
        self.log_event(
            "program_started", robot_class=type(self).__name__,
            tracking_mode=self.tracking_mode.value,
            target_fps=self.control_rate_hz,
        )

    def start_pc_service(self) -> None:
        """调用仓库内的启动脚本启动 XRoboToolkit PC Service。

        路径根据当前源码位置计算，不依赖启动命令所在目录。``check=True`` 会
        在脚本失败时立即抛出异常，避免程序在没有数据服务的情况下静默运行。
        启动脚本自身负责检测并避免重复启动服务。
        """

        launcher = (
            Path(__file__).resolve().parent.parent
            / "third_party"
            / "start_xrobotoolkit_pc_service.sh"
        )
        if not launcher.is_file():
            raise RuntimeError(f"未找到 PC Service 启动脚本：{launcher}")
        self.log_event("pc_service_starting", launcher=str(launcher))
        try:
            subprocess.run([str(launcher)], check=True)
        except BaseException:
            self.log_event(
                "pc_service_failed", level=logging.ERROR, exc_info=True)
            raise
        self.log_event("pc_service_started")

    def initialize(self) -> None:
        """初始化滤波状态和 XRoboToolkit SDK。

        该方法是幂等的：重复调用不会重复执行 ``sdk.init()``。先清空滤波器历史，
        可以避免同一个接口对象重新连接后沿用上一次会话的数据。
        """

        if not self._initialized:
            self.low_pass_filter.reset()
            self.rotation_low_pass_filter.reset()
            self._sdk.init()
            self._initialized = True
            self.log_event("sdk_initialized")
            self._start_monitor()

    def _start_monitor(self) -> None:
        """在启用状态下显示遥操监视窗口。"""

        if self._show_monitor:
            self.monitor.start()

    def apply_low_pass_filter(self, values: Any) -> np.ndarray:
        """滤波一个标量或固定形状数组，并返回 NumPy 数组。

        滤波器具有跨帧状态，因此每个周期只应调用一次，并且后续输入形状必须
        与第一次调用一致。该接口通常用于 Tracker 位置，不适用于旋转矩阵。
        """

        return self.low_pass_filter.update(values)

    def apply_rotation_low_pass_filter(self, rotations: Any) -> np.ndarray:
        """在旋转空间中滤波一个或一组 ``3×3`` 旋转矩阵。

        与逐元素滤波不同，该实现会保持旋转矩阵的正交性和单位行列式。输入中
        含非有限值或非法旋转矩阵时，底层滤波器会抛出 ``ValueError``。
        """

        return self.rotation_low_pass_filter.update(rotations)

    def read_tracking_input(self) -> TeleoperationInput:
        """从 SDK 读取一帧数据并封装成 :class:`TeleoperationInput`。

        全身数据和独立 Tracker 数据只读取当前模式对应的一组。头显、控制器与
        手部信息始终读取，方便其他机器人子类复用。没有数据不视为异常，而是
        用 ``None``、``False`` 和计数 ``0`` 表示，下游应保持上一条安全命令。

        注意：``captured_at_ns`` 是 PC 端封装本帧时记录的本地时间，不能证明
        SDK 返回的是最新网络帧，也不能单独用于检测服务端缓存的陈旧数据。
        """

        if not self._initialized:
            self.initialize()

        # 先设置“无数据”默认值，确保两种追踪模式返回一致、完整的数据结构。
        body_available = False
        body_pose = None
        motion_tracker_pose = None
        motion_tracker_velocity = None
        motion_tracker_acceleration = None
        motion_tracker_serial_numbers = None
        motion_tracker_count = 0
        motion_timestamp_ns = None
        pico_timestamp_ns = int(self._sdk.get_time_stamp_ns())

        if self.tracking_mode is TrackingMode.WHOLE_BODY:
            # 只有 SDK 明确报告可用时才读取人体关节，避免读取无效缓存。
            body_available = bool(self._sdk.is_body_data_available())
            if body_available:
                body_pose = self._sdk.get_body_joints_pose()
        else:
            # Object Tracking 可能返回 0、2 或更多设备；左右绑定由具体子类完成。
            motion_tracker_count = int(self._sdk.num_motion_data_available())
            if motion_tracker_count > 0:
                motion_tracker_pose = self._sdk.get_motion_tracker_pose()
                motion_tracker_velocity = self._sdk.get_motion_tracker_velocity()
                motion_tracker_acceleration = self._sdk.get_motion_tracker_acceleration()
                motion_tracker_serial_numbers = (
                    self._sdk.get_motion_tracker_serial_numbers()
                )
                motion_timestamp_ns = int(self._sdk.get_motion_timestamp_ns())

        # 集中构造字段不可替换的数据快照，避免下游依赖零散 SDK getter。
        return TeleoperationInput(
            captured_at_ns=time.monotonic_ns(),
            tracking_mode=self.tracking_mode,
            headset_pose=self._sdk.get_headset_pose(),
            left_controller_pose=self._sdk.get_left_controller_pose(),
            right_controller_pose=self._sdk.get_right_controller_pose(),
            left_hand_state=self._sdk.get_left_hand_tracking_state(),
            right_hand_state=self._sdk.get_right_hand_tracking_state(),
            body_pose=body_pose,
            body_available=body_available,
            motion_tracker_pose=motion_tracker_pose,
            motion_tracker_velocity=motion_tracker_velocity,
            motion_tracker_acceleration=motion_tracker_acceleration,
            motion_tracker_serial_numbers=motion_tracker_serial_numbers,
            motion_tracker_count=motion_tracker_count,
            motion_timestamp_ns=motion_timestamp_ns,
            left_hand_active=bool(self._sdk.get_left_hand_is_active()),
            right_hand_active=bool(self._sdk.get_right_hand_is_active()),
            pico_timestamp_ns=pico_timestamp_ns,
        )

    def run(self) -> None:
        """启动服务和 SDK，并按 ``control_rate_hz`` 运行控制循环。

        每轮依次读取 PICO 数据、解算机器人命令并在结果有效时下发。解算返回
        ``None`` 表示当前帧不可用，此时保持下层控制器当前状态并立即重新采样，
        不等待正常控制周期。只有产生有效命令的周期才按照目标频率进行休眠。
        每轮使用单调时钟测量实际计算耗时，只休眠剩余周期。如果一次解算超过
        周期预算，则立即开始下一轮，不额外排队或补跑旧帧。按 Ctrl+C 或循环
        内发生异常时都会进入 ``finally``，确保 SDK 被关闭；非键盘异常继续向上
        抛出，避免控制故障被静默忽略。
        """

        # 用 try/finally 包住整个运行阶段，保证任何退出路径都会释放 SDK。
        try:
            # 根据构造参数决定是否由本程序自动启动 PC Service。
            if self._auto_start_pc_service:
                # 执行仓库中的启动脚本；启动失败会直接抛出异常并停止运行。
                self.start_pc_service()
            # 初始化 SDK 并清空位置、旋转滤波器在上一次会话中的历史状态。
            self.initialize()
            # 将循环状态设为运行；外部调用 stop() 后该值会变为 False。
            self._running = True

            # 只要没有收到停止请求，就持续执行固定频率的遥操作周期。
            while self._running:
                # 记录本周期开始的单调时钟，用于计算本轮实际消耗的时间。
                cycle_started = time.monotonic()
                # 从 XRoboToolkit SDK 读取并封装当前一帧 PICO 追踪数据。
                tracking_input = self.read_tracking_input()
                tracking_read_finished = time.monotonic()
                # 交给具体机器人子类，把追踪输入解算成机器人控制命令。
                robot_command = self.solve_inverse_kinematics(tracking_input)
                ik_finished = time.monotonic()
                self.record_teleoperation_frame(
                    tracking_input, robot_command,
                    ik_finished - cycle_started,
                    sdk_read_seconds=tracking_read_finished - cycle_started,
                    ik_seconds=ik_finished - tracking_read_finished,
                )
                # None 表示当前追踪数据不可用，需要尽快重新读取下一份数据。
                if robot_command is None:
                    # 没有数据就一直尝试获取数据，不 time.sleep 
                    continue
                # 由具体机器人子类把有效命令发送给仿真器或真实机器人。
                self.send_to_robot(robot_command)

                # 保证周期性运行
                # 计算读取、解算和发送总共占用了多少墙钟时间。
                elapsed = time.monotonic() - cycle_started
                # 从目标周期中扣除计算时间，得到本周期还可以休眠的时间。
                remaining = self._period_seconds - elapsed
                # 只有计算尚未耗尽周期预算时才休眠；超时后立即进入下一轮。
                if remaining > 0:
                    # 休眠剩余时间，使平均循环频率尽量接近 control_rate_hz。
                    time.sleep(remaining)

        # Ctrl+C 属于操作者主动停止，正常结束即可，不需要输出错误堆栈。
        except KeyboardInterrupt:
            # 不在这里处理资源释放，统一交给下面的 finally 分支。
            self.log_event("control_loop_interrupted")
        except BaseException:
            self.log_event(
                "control_loop_failed", level=logging.ERROR, exc_info=True)
            raise
        # 无论正常停止、Ctrl+C 还是其他异常，退出前都必须执行清理。
        finally:
            # 停止循环并在已经初始化的情况下调用 sdk.close()。
            self.close()

    def stop(self) -> None:
        """请求控制循环在当前周期结束后停止，不直接关闭 SDK。"""

        self._running = False

    def record_teleoperation_frame(
        self, tracking_input: TeleoperationInput, robot_command: Any,
        elapsed_seconds: float, *, sdk_read_seconds: float | None = None,
        ik_seconds: float | None = None,
    ) -> None:
        """统一更新监视器、跟踪状态与周期性文件日志。"""

        self._start_monitor()
        command_available = robot_command is not None
        if command_available != self._tracking_available:
            self.log_event(
                "tracking_acquired" if command_available else "tracking_lost",
                level=logging.INFO if command_available else logging.WARNING,
                tracker_count=tracking_input.motion_tracker_count,
            )
            self._tracking_available = command_available
        metrics = self.monitor.update(
            tracking_input, command_available, elapsed_seconds,
            sdk_read_seconds, ik_seconds)
        if metrics is not None:
            self.log_event("metrics", **metrics)
        now = time.monotonic()
        if (command_available and self._logger is not None
                and (self._last_data_log_at is None
                     or now - self._last_data_log_at
                     >= self._log_data_interval_seconds)):
            # 这里只记录 IK 目标；真实关节反馈必须由机器人通信层提供。
            self._last_data_log_at = now
            try:
                serials = ([] if tracking_input.motion_tracker_serial_numbers is None
                           else np.asarray(
                               tracking_input.motion_tracker_serial_numbers
                           ).reshape(-1).tolist())
                serials = [
                    value.decode(errors="replace") if isinstance(value, bytes)
                    else str(value)
                    for value in serials
                ]
                numeric_fields = {
                    "pico_motion_tracker_pose": tracking_input.motion_tracker_pose,
                    "pico_motion_tracker_velocity": tracking_input.motion_tracker_velocity,
                    "pico_motion_tracker_acceleration": (
                        tracking_input.motion_tracker_acceleration),
                    "robot_joint_target_rad": getattr(
                        robot_command, "joint_positions", None),
                }
                snapshot = {
                    name: (None if value is None else
                           np.asarray(value, dtype=float).round(6).tolist())
                    for name, value in numeric_fields.items()
                }
                snapshot.update(
                    pico_captured_at_ns=tracking_input.captured_at_ns,
                    pico_motion_timestamp_ns=tracking_input.motion_timestamp_ns,
                    pico_timestamp_ns=tracking_input.pico_timestamp_ns,
                    pico_motion_tracker_serials=serials,
                    robot_joint_names=tuple(
                        str(name) for name in
                        getattr(robot_command, "joint_names", ())),
                    robot_ik_status=getattr(robot_command, "status", None),
                    robot_ik_iterations=getattr(
                        robot_command, "iterations", None),
                )
                self.log_event("data_snapshot", **snapshot)
            except (TypeError, ValueError) as exc:
                self.log_event(
                    "data_snapshot_failed", level=logging.WARNING,
                    error=str(exc),
                )

    def log_event(
        self, event: str, *, level: int = logging.INFO,
        exc_info: bool = False, **fields: Any,
    ) -> None:
        """子类共用的唯一日志入口；文件、队列和轮转由基类管理。"""

        if self._logger is None:
            return
        details = " ".join(
            f"{name}={value!r}" for name, value in sorted(fields.items()))
        message = f"session={self._session_id} event={event}"
        if details:
            message = f"{message} {details}"
        self._logger.log(level, message, exc_info=exc_info)

    def close(self) -> None:
        """停止控制循环并释放 SDK 资源；尚未初始化时可安全调用。"""

        self._running = False
        self.monitor.close()
        try:
            if self._initialized:
                self._sdk.close()
                self._initialized = False
                self.log_event("sdk_closed")
        finally:
            if self._log_listener is not None:
                self.log_event(
                    "program_stopped",
                    runtime_seconds=round(time.monotonic() - self._session_started_at, 3),
                )
                self._log_listener.stop()
                self._log_listener = None
                if self._log_handler is not None:
                    self._log_handler.close()
                    self._log_handler = None
                if self._logger is not None:
                    self._logger.handlers.clear()
                    self._logger = None

    def __enter__(self) -> "BaseRobotInterface":
        """进入 ``with`` 语句时初始化 SDK，并返回接口自身。"""

        self.initialize()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        """离开 ``with`` 语句时始终关闭 SDK，异常仍由调用方处理。"""

        self.close()

    @abstractmethod
    def solve_inverse_kinematics(self, tracking_input: TeleoperationInput) -> Any:
        """将 PICO 输入转换为具体机器人的命令。

        数据无效或不足时应返回 ``None``；正常结果的具体类型由子类定义。
        """

    @abstractmethod
    def send_to_robot(self, robot_command: Any) -> None:
        """把解算结果发送给仿真或真实机器人，由子类实现通信细节。"""
