"""A3 Ultra 双臂底层 ROS 2 通信适配器。"""

from __future__ import annotations

from collections.abc import Sequence
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any
import warnings
import numpy as np

if __package__:
    from ..arm_diagnostic_log import ArmDiagnosticLog
    from ..real_robot_interface import RealRobotInterface
    from . import config
    from .dual_arm_ik import ARM_JOINT_NAMES, DEFAULT_ARM_JOINT_POSITIONS
    from .a3_ultra_arm_motion_planner import ArmMotionPlanner, NATURAL_ARM_POSITIONS
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from pico_teleop_interface.robot_interfaces.arm_diagnostic_log import ArmDiagnosticLog
    from pico_teleop_interface.robot_interfaces.real_robot_interface import RealRobotInterface
    from pico_teleop_interface.robot_interfaces.a3_ultra import config
    from pico_teleop_interface.robot_interfaces.a3_ultra.dual_arm_ik import (
        ARM_JOINT_NAMES,
        DEFAULT_ARM_JOINT_POSITIONS,
    )
    from pico_teleop_interface.robot_interfaces.a3_ultra.a3_ultra_arm_motion_planner import (
        ArmMotionPlanner,
        NATURAL_ARM_POSITIONS,
    )

# 当前终端可能预先 source 过系统 ROS 2，不能让 Python 3.12 包覆盖 Conda Python 3.11 包。
sys.path[:] = [path for path in sys.path if "python3.12/site-packages" not in path]

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# AIMDK v3.2 手册中的双臂关节硬限位，单位为 rad。
ARM_JOINT_LIMITS = {
    "left_shoulder_pitch_joint": (-2.87979, 2.87979),
    "left_shoulder_roll_joint": (-0.08727, 2.61799),
    "left_shoulder_yaw_joint": (-2.79253, 2.79253),
    "left_elbow_joint": (-0.95993, 1.74533),
    "left_wrist_roll_joint": (-2.79253, 2.79253),
    "left_wrist_pitch_joint": (-1.62316, 1.62316),
    "left_wrist_yaw_joint": (-1.62316, 1.62316),
    "right_shoulder_pitch_joint": (-2.87979, 2.87979),
    "right_shoulder_roll_joint": (-2.61799, 0.08727),
    "right_shoulder_yaw_joint": (-2.79253, 2.79253),
    "right_elbow_joint": (-0.95993, 1.74533),
    "right_wrist_roll_joint": (-2.79253, 2.79253),
    "right_wrist_pitch_joint": (-1.62316, 1.62316),
    "right_wrist_yaw_joint": (-1.62316, 1.62316),
}
assert tuple(ARM_JOINT_LIMITS) == ARM_JOINT_NAMES


class A3UltraRealRobot(RealRobotInterface):
    """通过 HDU 桥接的底层 ``body_drive`` 接口控制 A3 Ultra 双臂。

    ROS 2 节点直接运行在 ``a3_teleop`` Conda 环境中。独立执行器线程以
    100 Hz 发送最新目标，不依赖 IK 的实际帧率。
    """

    def __init__(
        self,
        *,
        dds_profile: str | Path | None = config.REAL_ROBOT_DDS_PROFILE,
    ) -> None:
        self.command_topic = config.REAL_ROBOT_COMMAND_TOPIC
        self.state_topic = config.REAL_ROBOT_STATE_TOPIC
        self.command_rate_hz = config.REAL_ROBOT_COMMAND_RATE_HZ
        self.max_joint_speed = config.REAL_ROBOT_MAX_JOINT_SPEED_RAD_S
        self.max_joint_acceleration = config.REAL_ROBOT_MAX_JOINT_ACCELERATION_RAD_S2
        self.connect_timeout = config.REAL_ROBOT_CONNECT_TIMEOUT_SECONDS
        self.state_timeout = config.REAL_ROBOT_STATE_TIMEOUT_SECONDS
        self.ros_domain_id = config.REAL_ROBOT_ROS_DOMAIN_ID
        self.dds_profile = Path(dds_profile).expanduser() if dds_profile else None

        self._lock = threading.Lock()
        self._state_ready = threading.Event()
        self._node: Node | None = None
        self._spin_thread: threading.Thread | None = None
        self._publisher: Any = None
        self._ros_error: str | None = None
        self._bridge_ready = False
        self._bridge_ready_at = 0.0
        self._bridge_ready_event = threading.Event()
        self._measured: tuple[float, ...] | None = None
        self._measured_velocity: tuple[float, ...] | None = None
        self._state_received_at = 0.0
        self._commanded: tuple[float, ...] | None = None
        self._target: tuple[float, ...] | None = None
        self._last_publish_at = time.monotonic()
        self._active = False
        self._connected = False
        self._low_level_warning_shown = False
        self._planner = ArmMotionPlanner()
        # 搜索过程中也必须遵守实机硬限位，不能只检查最终目标。
        self._planner.lower = np.maximum(self._planner.lower, [ARM_JOINT_LIMITS[n][0] for n in ARM_JOINT_NAMES])
        self._planner.upper = np.minimum(self._planner.upper, [ARM_JOINT_LIMITS[n][1] for n in ARM_JOINT_NAMES])
        self._planning = False
        self._planner_lock = threading.Lock()
        self._path: np.ndarray | None = None
        self._path_index = 0
        self._home_stable_since: float | None = None
        self._home_stable_reference: np.ndarray | None = None
        self._stream_target: tuple[float, ...] | None = None
        self._stream_updated_at = 0.0
        self._motion_generation = 0
        self._diagnostics: ArmDiagnosticLog | None = None
        self._last_command_sent_at: float | None = None
        self._command_speed = 0.0

    def connect(self) -> None:
        """启动 Conda 环境内的 ROS 2 节点，并等待第一帧双臂反馈。"""

        if self._connected:
            return

        self._diagnostics = ArmDiagnosticLog("pc")
        self._last_command_sent_at = None
        self._record("connection_start", joint_names=ARM_JOINT_NAMES,
                     command_topic=self.command_topic, state_topic=self.state_topic,
                     command_rate_hz=self.command_rate_hz, max_joint_speed=self.max_joint_speed)

        os.environ["ROS_DOMAIN_ID"] = str(self.ros_domain_id)
        os.environ["RMW_IMPLEMENTATION"] = config.REAL_ROBOT_RMW_IMPLEMENTATION
        self._state_ready.clear()
        self._ros_error = None
        self._bridge_ready = False
        self._bridge_ready_at = 0.0
        self._bridge_ready_event.clear()
        self._measured = None
        self._measured_velocity = None
        self._commanded = None
        self._target = None
        self._last_publish_at = time.monotonic()
        self._active = False
        self._command_speed = 0.0
        self._low_level_warning_shown = False
        self._path = None
        self._planning = False
        self._stream_target = None
        if self.dds_profile:
            if not self.dds_profile.is_file():
                raise FileNotFoundError(f"DDS 配置文件不存在: {self.dds_profile}")
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(self.dds_profile.resolve())
        elif "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
            warnings.warn(
                "未设置 DDS 配置文件；第三方 PC 直连机器人时通常无法发现 ROS 2 话题。",
                RuntimeWarning,
                stacklevel=2,
            )

        try:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            # shortcut: 进程只创建一个真机 ROS 节点；多机器人时改为独立 Context。
            rclpy.init(domain_id=self.ros_domain_id)
            self._node = Node("a3_ultra_real_robot")
            self._publisher = self._node.create_publisher(
                JointState, self.command_topic, qos
            )
            self._node.create_subscription(
                JointState, self.state_topic, self._on_joint_state, qos
            )
            self._node.create_subscription(
                Bool, config.REAL_ROBOT_BRIDGE_READY_TOPIC, self._on_bridge_ready, qos
            )
            self._node.create_timer(1.0 / self.command_rate_hz, self._publish_command)
            self._spin_thread = threading.Thread(
                target=self._spin_ros, name="a3-ultra-ros", daemon=True
            )
            self._spin_thread.start()
            if not self._state_ready.wait(self.connect_timeout):
                raise TimeoutError(
                    f"{self.connect_timeout:.1f} 秒内未收到 {self.state_topic}；"
                    "请检查 body-drive 控制器、HDU 底层桥接进程和 DDS 网络"
                )
            if self._ros_error:
                raise RuntimeError(self._ros_error)
            self._connected = True
            self._record("connected")
        except Exception as exc:
            self._record("connection_failed", reason=str(exc))
            self.close()
            raise

    def read_joint_positions(self, joint_names: Sequence[str]) -> tuple[float, ...]:
        """读取 A3 Ultra 关节反馈，并按请求的关节顺序返回弧度值。"""

        self._require_connected()
        requested = self._validate_joint_names(joint_names)
        with self._lock:
            # PC 反馈超时仅在监视窗告警；仍禁止使用尚未收到过的反馈。
            if self._measured is None:
                raise TimeoutError("尚未收到双臂反馈")
            positions = dict(zip(ARM_JOINT_NAMES, self._measured, strict=True))
        return tuple(positions[name] for name in requested)

    def read_all_arm_joint_positions(self) -> dict[str, float]:
        """读取双臂全部 14 个关节的名称和角度，角度单位为 rad。"""

        return dict(
            zip(
                ARM_JOINT_NAMES,
                self.read_joint_positions(ARM_JOINT_NAMES),
                strict=True,
            )
        )

    def read_arm_feedback(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """为 SONIC 原子读取同一帧双臂位置、速度，均按标准 14 关节顺序。"""
        self._require_connected()
        with self._lock:
            if self._measured is None or self._measured_velocity is None:
                raise RuntimeError("SONIC requires arm position and velocity feedback")
            return self._measured, self._measured_velocity

    def move_to_default_pose(self) -> None:
        """从实时反馈规划避障路径，再异步归位并持续保持；规划失败不会直线归位。"""

        self.send_joint_positions(ARM_JOINT_NAMES, DEFAULT_ARM_JOINT_POSITIONS)

    def move_to_natural_pose(self) -> None:
        """避障移动到双臂自然下垂姿态，最后允许无穿透地接近身体。"""

        self.send_joint_positions(
            ARM_JOINT_NAMES, NATURAL_ARM_POSITIONS, allow_close_approach=True,
        )

    @property
    def is_moving(self) -> bool:
        """规划或路径执行期间为 True；后台反馈异常会直接抛出。"""

        self._require_connected()
        with self._lock:
            return self._planning or self._path is not None

    def send_joint_positions(
        self, joint_names: Sequence[str], positions: Sequence[float],
        *, allow_close_approach: bool = False,
    ) -> None:
        """规划到指定角度并交给 100 Hz 线程执行；规划阻塞，执行异步。

        适用于 GUI 的离散目标，不应在每帧遥操作循环中反复调用。
        ``allow_close_approach`` 仅允许最后无穿透地进入 10 cm 安全距离内。
        """

        self._require_connected()
        if not self._control_bridge_ready():
            # DDS 首帧状态与心跳可能到达顺序不同，给心跳发现留出连接超时窗口。
            self._bridge_ready_event.wait(self.connect_timeout)
            self._require_connected()
            if not self._control_bridge_ready():
                raise ConnectionError("MDU control bridge not ready (missing heartbeat or dry-run mode)")
        if not self._low_level_warning_shown:
            warnings.warn(
                "底层桥接要求官方 motion_control 已停止，并且 body-drive 控制器已就绪。",
                RuntimeWarning,
                stacklevel=2,
            )
            self._low_level_warning_shown = True
        names = self._validate_joint_names(joint_names)
        values = tuple(float(value) for value in positions)
        if len(values) != len(names) or not all(math.isfinite(value) for value in values):
            raise ValueError("关节名称与角度数量必须一致，且角度必须为有限值")

        by_name = dict(zip(names, values, strict=True))
        for name, value in by_name.items():
            lower, upper = ARM_JOINT_LIMITS[name]
            if not lower <= value <= upper:
                raise ValueError(f"{name}={value:.5f} rad 超出限位 [{lower}, {upper}]")
        with self._lock:
            if self._planning or self._path is not None or self._stream_target is not None:
                raise RuntimeError("已有规划或运动正在执行，请等待完成")
            if self._measured is None:
                raise TimeoutError("尚未收到双臂反馈，不能开始路径规划")
            start = np.array(self._measured)
            self._motion_generation += 1
            generation = self._motion_generation
            self._planning = True
            # 已经取得控制时保持当前位置；首次归位等路径完成后才开启指令流。
            self._commanded = tuple(start)
            self._target = tuple(start)
            self._record("planning_started", start=start.tolist(), goal=by_name)
        try:
            goal = np.array([by_name[name] for name in ARM_JOINT_NAMES])
            # 取消只禁止提交结果；尚未返回的优化器仍需独占其模型。
            with self._planner_lock:
                planner = (self._planner.plan_close_approach
                           if allow_close_approach else self._planner.plan)
                path = planner(
                    start, goal,
                    timeout_seconds=config.REAL_ROBOT_PLANNING_TIMEOUT_SECONDS,
                )
            measured = np.array(self.read_joint_positions(ARM_JOINT_NAMES))
            if np.max(np.abs(measured - start)) > config.REAL_ROBOT_PLANNING_START_TOLERANCE_RAD:
                raise RuntimeError("规划期间机器人位置发生变化，请重新规划")
            with self._lock:
                if generation != self._motion_generation or not self._connected:
                    raise RuntimeError("规划已取消，路径不会执行")
                if not self._control_bridge_ready():
                    raise ConnectionError("MDU bridge became not ready during planning; check MDU BRIDGE FAULT output")
                self._path = path
                self._path_index = 1
                self._home_stable_since = None
                self._home_stable_reference = None
                self._target = tuple(goal)
                self._last_publish_at = time.monotonic()
                self._command_speed = 0.0
                self._active = True
                self._record("path_committed", waypoints=len(path), target=self._target)
        except Exception as exc:
            self._record("planning_failed", reason=str(exc))
            raise
        finally:
            with self._lock:
                if generation == self._motion_generation:
                    self._planning = False

    def stream_joint_positions(
        self, joint_names: Sequence[str], positions: Sequence[float],
    ) -> None:
        """覆盖实时 IK 目标；100 Hz 线程限速发送，不规划、不排队等待旧帧。"""

        self._require_connected()
        names = self._validate_joint_names(joint_names)
        values = tuple(float(value) for value in positions)
        if len(values) != len(names) or not all(math.isfinite(v) for v in values):
            raise ValueError("实时目标必须包含 14 个有限关节角")
        by_name = dict(zip(names, values, strict=True))
        for name, value in by_name.items():
            if not ARM_JOINT_LIMITS[name][0] <= value <= ARM_JOINT_LIMITS[name][1]:
                raise ValueError(f"{name}={value:.5f} rad 超出实机关节限位")
        with self._lock:
            self._require_connected()
            if self._planning or self._path is not None:
                raise RuntimeError("离散路径尚未完成，不能接收实时目标")
            if not self._control_bridge_ready():
                raise ConnectionError("MDU control bridge not ready")
            if self._measured is None:
                raise TimeoutError("No arm feedback received; streaming rejected")
            self._stream_target = tuple(by_name[name] for name in ARM_JOINT_NAMES)
            self._target = self._stream_target
            self._stream_updated_at = time.monotonic()
            if not self._active:
                self._commanded = self._measured
                self._last_publish_at = self._stream_updated_at
                self._command_speed = 0.0
            self._active = True
            self._record("stream_target", position=self._stream_target)

    def stop(self) -> None:
        """中止规划和后续路径，持续保持最后已发送的角度，不能直连原终点。"""

        with self._lock:
            self._record("stop_hold_requested", commanded=self._commanded, measured=self._measured)
            self._motion_generation += 1
            self._planning = False
            self._path = None
            self._stream_target = None
            self._target = self._commanded
            self._command_speed = 0.0

    def close(self) -> None:
        """释放 A3 Ultra 控制权和通信资源。"""

        self._connected = False
        with self._lock:
            self._motion_generation += 1
            self._planning = False
            self._path = None
            self._stream_target = None
            self._active = False
            self._command_speed = 0.0
        if self._node is None:
            if self._diagnostics is not None:
                self._diagnostics.close()
                self._diagnostics = None
            return

        # 进程退出后无法继续保持；先停止本进程的定时发布，再释放 ROS 节点。
        time.sleep(min(0.05, 2.0 / self.command_rate_hz))
        node = self._node
        rclpy.try_shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        node.destroy_node()
        self._node = None
        self._publisher = None
        self._spin_thread = None
        if self._diagnostics is not None:
            self._diagnostics.close()
            self._diagnostics = None

    def _record(self, event, **fields) -> None:
        if self._diagnostics is not None:
            self._diagnostics.write(event, **fields)

    def _spin_ros(self) -> None:
        """在独立线程处理 ROS 反馈和 100 Hz 发送定时器。"""

        try:
            assert self._node is not None
            rclpy.spin(self._node)
        except Exception as exc:
            with self._lock:
                if rclpy.ok():
                    self._ros_error = f"A3 ROS 2 执行器异常: {exc}"
                    self._record("ros_executor_failed", reason=str(exc))
            self._state_ready.set()

    def _on_joint_state(self, message: JointState) -> None:
        """校验并缓存机器人最新的 14 维双臂反馈。"""

        names = tuple(message.name)
        if (len(names) != len(ARM_JOINT_NAMES)
                or len(message.position) != len(ARM_JOINT_NAMES)
                or set(names) != set(ARM_JOINT_NAMES)):
            return
        by_name = dict(zip(names, message.position, strict=True))
        positions = tuple(float(by_name[name]) for name in ARM_JOINT_NAMES)
        if not all(math.isfinite(value) for value in positions):
            return
        velocities = None
        if len(message.velocity) == len(names):
            by_name_velocity = dict(zip(names, message.velocity, strict=True))
            values = tuple(float(by_name_velocity[name]) for name in ARM_JOINT_NAMES)
            if all(math.isfinite(value) for value in values):
                velocities = values
        with self._lock:
            self._record("feedback", stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                         names=list(message.name), position=list(message.position),
                         velocity=list(message.velocity), effort=list(message.effort),
                         receive_gap_ms=(time.monotonic() - self._state_received_at) * 1000
                         if self._state_received_at else None)
            self._measured = positions
            self._measured_velocity = velocities
            if self._commanded is None:
                self._commanded = positions
            self._state_received_at = time.monotonic()
        self._state_ready.set()

    def _on_bridge_ready(self, message: Bool) -> None:
        with self._lock:
            self._record("bridge_ready", ready=bool(message.data))
            self._bridge_ready = bool(message.data)
            self._bridge_ready_at = time.monotonic()
            if self._bridge_ready:
                self._bridge_ready_event.set()
            else:
                self._bridge_ready_event.clear()

    def _control_bridge_ready(self) -> bool:
        # 心跳年龄只用于监视窗告警；MDU 明确发布 False 时才停止控制。
        return self._bridge_ready

    def _publish_command(self) -> None:
        """逐段执行规划路径；使用共同插值系数，保持已检查过的关节空间线段。"""

        now = time.monotonic()
        with self._lock:
            elapsed = min(max(0.0, now - self._last_publish_at), 2.0 / self.command_rate_hz)
            self._last_publish_at = now
            if (not self._active or self._ros_error or self._measured is None
                    or self._commanded is None or self._publisher is None):
                return
            state_age = now - self._state_received_at
            reasons = []
            # 暂停 PC 反馈超时中止；state_timeout 仅作为监视窗红色告警阈值。
            # if state_age > self.state_timeout:
            #     reasons.append(f"arm feedback age {state_age:.3f}s > {self.state_timeout:.3f}s")
            if not self._bridge_ready:
                reasons.append("MDU reports NOT READY; check its BRIDGE FAULT output")
            if reasons:
                self._ros_error = "Motion aborted: " + "; ".join(reasons)
                print(self._ros_error, file=sys.stderr, flush=True)
                self._record("motion_aborted", reasons=reasons, commanded=self._commanded,
                             measured=self._measured, feedback_age_ms=state_age * 1000)
                self._motion_generation += 1
                self._planning = False
                self._path = None
                self._stream_target = None
                self._target = self._commanded
                self._command_speed = 0.0
                # 故障锁存后不再发送；防止重启 MDU 控制桥时旧 GUI 自动恢复下发。
                return
            if self._path is not None:
                if self._path_index < len(self._path):
                    waypoint = self._path[self._path_index]
                    delta = waypoint - np.asarray(self._commanded)
                    maximum = float(np.max(np.abs(delta)))
                    self._ramp_command_speed(elapsed)
                    alpha = min(1., self._command_speed * elapsed / maximum) if maximum else 1.
                    candidate = np.asarray(self._commanded) + alpha * delta
                    self._commanded = tuple(candidate)
                    if alpha == 1.:
                        self._path_index += 1
                if self._path_index >= len(self._path):
                    error = max(abs(a - b) for a, b in zip(self._measured, self._target))
                    if self._target == DEFAULT_ARM_JOINT_POSITIONS:
                        # 只能用持续到达的新反馈判断稳定，不能把断流的旧缓存当作静止。
                        if (error >= config.REAL_ROBOT_HOME_TOLERANCE_RAD
                                or state_age > self.state_timeout):
                            self._home_stable_since = None
                            self._home_stable_reference = None
                        else:
                            if (self._home_stable_since is None
                                    or np.max(np.abs(np.asarray(self._measured)
                                        - self._home_stable_reference))
                                    > config.REAL_ROBOT_HOME_STABLE_RANGE_RAD):
                                self._home_stable_since = now
                                self._home_stable_reference = np.array(self._measured)
                            if now - self._home_stable_since >= config.REAL_ROBOT_HOME_SETTLE_SECONDS:
                                self._path = None
                                self._command_speed = 0.0
                                self._record("home_pose_settled", max_error_rad=error,
                                             stable_seconds=now - self._home_stable_since)
                    elif error <= config.REAL_ROBOT_PLANNING_START_TOLERANCE_RAD:
                        self._path = None
                        self._command_speed = 0.0
            elif self._stream_target is not None:
                if now - self._stream_updated_at > config.REAL_ROBOT_STREAM_TIMEOUT_SECONDS:
                    # PICO/IK 停止更新时冻结在最后发出的角度，不继续走向旧目标。
                    self._stream_target = None
                    self._target = self._commanded
                    self._command_speed = 0.0
                    self._record("stream_stalled_hold", commanded=self._commanded)
                else:
                    delta = np.asarray(self._stream_target) - self._commanded
                    maximum = float(np.max(np.abs(delta)))
                    self._ramp_command_speed(elapsed)
                    alpha = min(1., self._command_speed * elapsed / maximum) if maximum else 1.
                    self._commanded = tuple(np.asarray(self._commanded) + alpha * delta)
            positions = self._commanded

        self._publish_positions(positions)

    def _ramp_command_speed(self, elapsed: float) -> None:
        """从静止按加速度上限平滑增加指令速度。"""

        self._command_speed = min(
            self.max_joint_speed,
            self._command_speed + self.max_joint_acceleration * elapsed,
        )

    def _publish_positions(self, positions: tuple[float, ...]) -> None:
        """发布一帧固定顺序的 14 维关节位置命令。"""

        node = self._node
        publisher = self._publisher
        if node is None or publisher is None:
            return
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = list(ARM_JOINT_NAMES)
        message.position = list(positions)
        message.velocity = [0.0] * len(ARM_JOINT_NAMES)
        message.effort = [0.0] * len(ARM_JOINT_NAMES)
        publish_started_at = time.monotonic()
        publisher.publish(message)
        sent_at = time.monotonic()
        self._record("command_sent", stamp_ns=message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                     position=list(positions), measured=self._measured,
                     send_gap_ms=(sent_at - self._last_command_sent_at) * 1000
                     if self._last_command_sent_at is not None else None,
                     publish_ms=(sent_at - publish_started_at) * 1000,
                     path_index=self._path_index)
        self._last_command_sent_at = sent_at

    def _require_connected(self) -> None:
        if not self._connected:
            raise ConnectionError("尚未连接 A3 Ultra")
        if self._ros_error:
            raise ConnectionError(self._ros_error)

    @staticmethod
    def _validate_joint_names(joint_names: Sequence[str]) -> tuple[str, ...]:
        names = tuple(joint_names)
        if len(names) != len(ARM_JOINT_NAMES) or set(names) != set(ARM_JOINT_NAMES):
            raise ValueError("A3 Ultra 双臂命令必须恰好包含官方的 14 个关节")
        return names


def main() -> None:
    """连接真机，限速移动到 MuJoCo 默认姿态，并持续保持到程序退出。"""

    dds_profile = Path(__file__).resolve().parents[3] / "usage/ros_dds_configuration.xml"
    robot = A3UltraRealRobot(dds_profile=dds_profile)
    try:
        robot.connect()
        print("A3 Ultra 双臂关节反馈：")
        for name, angle in robot.read_all_arm_joint_positions().items():
            print(f"  {name}: {angle:.6f} rad")
        robot.move_to_default_pose()
        print("正在移动并保持 MuJoCo 双臂默认姿态；按 Ctrl+C 退出。")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()


if __name__ == "__main__":
    main()
