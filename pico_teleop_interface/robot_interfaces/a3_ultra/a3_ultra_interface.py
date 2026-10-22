"""把 PICO Motion Tracker 数据转换为 A3 Ultra 双臂关节目标。

处理链路如下：

``PICO 原始位姿 → 坐标系转换 → 左右设备绑定 → 低通滤波 → 相对位姿映射
→ 双手掌目标 → 双臂 IK → 关节目标``。

本文件只负责追踪数据到关节角的转换。MuJoCo 动力学和 SONIC 策略分别放在
``sim.py`` 与 ``sonic_sim.py``，真机 ROS 2 通信由 ``a3_ultra_real_robot.py`` 负责。
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pinocchio as pin

if __package__:
    # 通过 ``python -m`` 或作为包导入时，使用正常的相对导入。
    from . import config
    from ..base_interface import BaseRobotInterface, TeleoperationInput, TrackingMode
    from .dual_arm_ik import (
        ARM_JOINT_NAMES,
        DEFAULT_ARM_JOINT_POSITIONS,
        A3UltraDualArmIK,
        A3UltraIKResult,
    )
    from .a3_ultra_real_robot import A3UltraRealRobot
else:
    # 直接运行本文件时没有父包，需要先把项目根目录加入模块搜索路径。
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from pico_teleop_interface.robot_interfaces.a3_ultra import config
    from pico_teleop_interface.robot_interfaces.base_interface import BaseRobotInterface, TeleoperationInput, TrackingMode
    from pico_teleop_interface.robot_interfaces.a3_ultra.dual_arm_ik import (
        ARM_JOINT_NAMES,
        DEFAULT_ARM_JOINT_POSITIONS,
        A3UltraDualArmIK,
        A3UltraIKResult,
    )
    from pico_teleop_interface.robot_interfaces.a3_ultra.a3_ultra_real_robot import A3UltraRealRobot

# XRoboToolkit 实际输出 X 右、Y 上、-Z 前；A3 使用 X 前、Y 左、Z 上。
PICO_TO_ROBOT_BASIS = np.array(((0., 0., -1.), (-1., 0., 0.), (0., 1., 0.)))

class A3UltraInterface(BaseRobotInterface):
    """PICO 双 Tracker 到 A3 Ultra 双臂关节角的机器人适配器。

    首帧有效头显及 Tracker 数据固定水平朝向，同时记录人和机器人的双掌参考位姿。
    之后每帧都使用“当前 Tracker − 首帧 Tracker”构造双掌目标，并不是在上一帧
    结果上累计增量。IK 以上一帧关节角为初值，结合肘部偏好、关节限位和
    自碰撞代价选择连续、安全的解。
    """

    def __init__(
        self,
        model_path: str | Path | None = None,
        position_scale: float = config.POSITION_SCALE,
        max_iterations: int = config.IK_MAX_ITERATIONS,
        max_solve_time_ms: float = config.IK_MAX_SOLVE_TIME_MS,
        position_tolerance: float = config.IK_POSITION_TOLERANCE_M,
        rotation_tolerance_deg: float = config.IK_ROTATION_TOLERANCE_DEG,
        damping: float = config.IK_DAMPING,
        integration_step: float = config.IK_INTEGRATION_STEP,
        max_joint_step: float = config.IK_MAX_JOINT_STEP_RAD,
        elbow_preference_weight: float = config.IK_ELBOW_PREFERENCE_WEIGHT,
        self_collision_weight: float = config.IK_SELF_COLLISION_WEIGHT,
        joint_limit_weight: float = config.IK_JOINT_LIMIT_WEIGHT,
        collision_activation_distance: float = config.COLLISION_ACTIVATION_DISTANCE_M,
        collision_min_distance: float = config.COLLISION_MIN_DISTANCE_M,
        collision_danger_distance: float = config.COLLISION_DANGER_DISTANCE_M,
        collision_active_enter_distance: float = config.COLLISION_ACTIVE_ENTER_DISTANCE_M,
        collision_active_exit_distance: float = config.COLLISION_ACTIVE_EXIT_DISTANCE_M,
        collision_gradient_refresh_interval: int = config.COLLISION_GRADIENT_REFRESH_INTERVAL,
        collision_warning_weight_scale: float = config.COLLISION_WARNING_WEIGHT_SCALE,
        collision_broadphase_margin: float = config.COLLISION_BROADPHASE_MARGIN_M,
        workspace_min_radius: float = config.IK_WORKSPACE_MIN_RADIUS_M,
        workspace_max_radius: float = config.IK_WORKSPACE_MAX_RADIUS_M,
        reconnect_jump_after_seconds: float = config.RECONNECT_JUMP_AFTER_SECONDS,
        left_tracker_serial: str | None = None,
        right_tracker_serial: str | None = None,
        **base_kwargs: Any,
    ) -> None:
        """加载机器人模型并配置 IK。

        ``position_scale`` 缩放人的平移幅度；位置、旋转阈值和
        ``max_iterations`` 和 ``max_solve_time_ms`` 分别限制迭代次数与单帧耗时；
        ``damping`` 抑制奇异点附近的数值放大；两个碰撞距离分别表示软代价开始
        生效的位置和不允许继续靠近的硬边界。``reconnect_jump_after_seconds``
        指定 Tracker 数据不可用或设备时间戳停止推进多久后将恢复帧视为重连。
        左右序列号可以同时省略，此时第一帧按头显水平朝向校正后的左右位置自动绑定。
        ``base_kwargs`` 继续传给 :class:`BaseRobotInterface`。
        """

        # A3 双臂必须读取独立 Motion Tracker，而不是 PICO 全身骨骼数据。
        base_kwargs.setdefault("tracking_mode", TrackingMode.OBJECT)
        base_kwargs.setdefault("control_rate_hz", config.CONTROL_RATE_HZ)
        base_kwargs.setdefault("low_pass_cutoff_hz", config.LOW_PASS_CUTOFF_HZ)
        base_kwargs.setdefault("show_monitor", config.SHOW_MONITOR)
        base_kwargs.setdefault("monitor_warning_fps", config.MONITOR_WARNING_FPS)
        base_kwargs.setdefault("enable_file_log", config.ENABLE_FILE_LOG)
        base_kwargs.setdefault("log_directory", config.LOG_DIRECTORY)
        base_kwargs.setdefault("log_level", config.LOG_LEVEL)
        base_kwargs.setdefault(
            "log_metrics_interval_seconds", config.LOG_METRICS_INTERVAL_SECONDS)
        base_kwargs.setdefault(
            "log_data_interval_seconds", config.LOG_DATA_INTERVAL_SECONDS)
        base_kwargs.setdefault("log_max_bytes", config.LOG_MAX_BYTES)
        base_kwargs.setdefault("log_backup_count", config.LOG_BACKUP_COUNT)
        super().__init__(**base_kwargs)

        # 只在 run_real_robot() 中连接；默认仿真入口不会发送实机指令。
        self.real_robot = A3UltraRealRobot()

        # 构造阶段集中拒绝会导致错误绑定或缩放方向反转的接口配置。
        if self.tracking_mode is not TrackingMode.OBJECT:
            raise ValueError("A3 Ultra 双臂接口需要 tracking_mode='object'")
        if ((left_tracker_serial is None) != (right_tracker_serial is None)
                or left_tracker_serial is not None
                and left_tracker_serial == right_tracker_serial):
            raise ValueError("左右 Tracker 序列号必须同时提供且不能相同")
        if position_scale <= 0:
            raise ValueError("position_scale 必须大于 0")
        if reconnect_jump_after_seconds <= 0:
            raise ValueError("reconnect_jump_after_seconds 必须大于 0")

        # PICO 层只保留平移比例；机器人模型和全部 IK 状态由独立求解器维护。
        self.position_scale = position_scale
        self.reconnect_jump_after_ns = int(reconnect_jump_after_seconds * 1e9)
        self._ik = A3UltraDualArmIK(
            model_path=model_path,
            max_iterations=max_iterations,
            max_solve_time_ms=max_solve_time_ms,
            position_tolerance=position_tolerance,
            rotation_tolerance_deg=rotation_tolerance_deg,
            damping=damping,
            integration_step=integration_step,
            max_joint_step=max_joint_step,
            elbow_preference_weight=elbow_preference_weight,
            self_collision_weight=self_collision_weight,
            joint_limit_weight=joint_limit_weight,
            collision_activation_distance=collision_activation_distance,
            collision_min_distance=collision_min_distance,
            collision_danger_distance=collision_danger_distance,
            collision_active_enter_distance=collision_active_enter_distance,
            collision_active_exit_distance=collision_active_exit_distance,
            collision_gradient_refresh_interval=collision_gradient_refresh_interval,
            collision_warning_weight_scale=collision_warning_weight_scale,
            collision_broadphase_margin=collision_broadphase_margin,
            workspace_min_radius=workspace_min_radius,
            workspace_max_radius=workspace_max_radius,
        )
        # 显式序列号立即生效；None 表示等待首帧按左右位置自动绑定。
        self._tracker_serials = (
            (left_tracker_serial, right_tracker_serial)
            if left_tracker_serial is not None
            else None
        )
        # 三组运行状态将在首帧标定时建立，reset_ik_reference() 会统一清空。
        self._pico_reference = self._palm_reference = self._accepted_targets = None
        self._heading_alignment = None
        self._heading_calibration_warned = False
        self._tracking_unavailable_since_ns = None
        self._last_motion_timestamp_ns = None
        self._motion_timestamp_changed_at_ns = None
        self._motion_stream_stalled = False

    @staticmethod
    def _parse_pico_pose(pose: Any) -> tuple[np.ndarray, np.ndarray] | None:
        """校验一个 PICO 位姿并转换到 A3 坐标轴定义。

        XRoboToolkit 位姿格式为 ``[x, y, z, qx, qy, qz, qw]``。四元数先归一化，
        再转成旋转矩阵；位置用基变换左乘，旋转则必须使用 ``C R Cᵀ`` 同时变换
        输入轴和输出轴。非法数据返回 ``None``，让控制循环等待下一帧。
        """

        # 网络中断或 SDK 尚未准备好时可能返回 None、错误形状或非有限值。
        try:
            values = np.asarray(pose, dtype=float)
        except (TypeError, ValueError):
            return None
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            return None

        # 零四元数没有合法旋转含义，不能交给 Pinocchio。
        quaternion = values[3:]
        quaternion_norm = np.linalg.norm(quaternion)
        if quaternion_norm < 1e-8:
            return None
        x, y, z, w = quaternion / quaternion_norm
        rotation_pico = pin.Quaternion(w, x, y, z).matrix()
        # PICO：X 右、Y 上、-Z 前；A3：X 前、Y 左、Z 上。
        rotation_robot = PICO_TO_ROBOT_BASIS @ rotation_pico @ PICO_TO_ROBOT_BASIS.T
        position_robot = PICO_TO_ROBOT_BASIS @ values[:3]
        return position_robot, rotation_robot

    def reset_ik_reference(self) -> None:
        """清除朝向标定和滤波历史，让下一帧有效头显及 Tracker 数据重新标定。

        该方法不会把机器人关节角恢复到默认值，因此重新标定时会以机器人当前
        双掌位姿作为新的输出锚点，避免无故跳回初始姿态。
        """

        self._pico_reference = self._palm_reference = self._accepted_targets = None
        self._heading_alignment = None
        self._heading_calibration_warned = False
        self._tracking_unavailable_since_ns = None
        self._last_motion_timestamp_ns = None
        self._motion_timestamp_changed_at_ns = None
        self._motion_stream_stalled = False
        self.low_pass_filter.reset()
        self.rotation_low_pass_filter.reset()

    def _select_tracker_poses(
        self, tracking_input: TeleoperationInput, heading_alignment: np.ndarray
    ) -> tuple[Any, Any] | None:
        """根据序列号稳定返回 ``(左 Tracker, 右 Tracker)``。

        SDK 数组顺序可能变化，因此只有第一次自动绑定会参考校正后的左右位置；绑定后只
        根据序列号查找。任一目标设备缺失或输入结构非法时返回 ``None``。
        """

        # 同时解析位姿与序列号，后面的字典用于消除 SDK 返回顺序的影响。
        try:
            poses = np.asarray(tracking_input.motion_tracker_pose, dtype=float)
            serials = tuple(
                value.decode(errors="replace") if isinstance(value, bytes) else str(value)
                for value in np.asarray(tracking_input.motion_tracker_serial_numbers).reshape(-1)
            )
        except (TypeError, ValueError):
            return None
        if (
            poses.ndim != 2
            or poses.shape[1] != 7
            or poses.shape[0] != len(serials)
            or poses.shape[0] < 2
        ):
            return None

        # 重复序列号无法唯一识别设备，直接丢弃该帧。
        poses_by_serial = dict(zip(serials, poses))
        if len(poses_by_serial) != len(serials):
            return None

        # 未显式配置时，按标定后的机器人 Y（左正右负）绑定，避免转身导致绑反。
        if self._tracker_serials is None:
            if len(serials) != 2:
                raise ValueError(
                    "自动左右绑定只支持恰好两个 Tracker；请显式指定左右序列号"
                )
            if not np.isfinite(poses).all():
                return None
            aligned_positions = (heading_alignment @ PICO_TO_ROBOT_BASIS @ poses[:, :3].T).T
            left, right = np.argsort(aligned_positions[:, 1])[[-1, 0]]
            self._tracker_serials = serials[int(left)], serials[int(right)]
            print(f"Motion Tracker mapping: left={self._tracker_serials[0]}, "
                  f"right={self._tracker_serials[1]}")
            self.log_event(
                "trackers_mapped", left=self._tracker_serials[0],
                right=self._tracker_serials[1])

        left_serial, right_serial = self._tracker_serials
        if left_serial not in poses_by_serial or right_serial not in poses_by_serial:
            return None
        return poses_by_serial[left_serial], poses_by_serial[right_serial]

    def _tracker_targets(self, tracker_poses: tuple) -> tuple[pin.SE3, pin.SE3]:
        """把当前 Tracker 位姿转换为机器人基座坐标系中的双掌目标。

        位置公式为 ``机器人参考掌位 + scale × A × (当前 Tracker - Tracker 参考位)``；
        A 是首帧头显水平朝向的逆旋转。姿态变化使用 ``A ΔR Aᵀ`` 校正后，
        再作用于机器人参考掌姿态。标定后 A 固定，不跟随后续转头变化。
        因此每帧都直接相对固定参考计算，不会逐帧积分产生累计误差。
        """

        targets: list[pin.SE3] = []
        for current, pico_reference, palm_reference in zip(
            tracker_poses, self._pico_reference, self._palm_reference
        ):
            current_position, current_rotation = current
            reference_position, reference_rotation = pico_reference
            alignment = self._heading_alignment
            position = palm_reference.translation + self.position_scale * (
                alignment @ (current_position - reference_position))
            rotation = (alignment @ current_rotation @ reference_rotation.T
                        @ alignment.T @ palm_reference.rotation)
            targets.append(pin.SE3(rotation, position))
        # 小于 2 mm / 0.5° 的变化视为噪声，继续保持上一次被接受的目标。
        if self._accepted_targets is not None:
            for target, previous in zip(targets, self._accepted_targets):
                if (np.linalg.norm(target.translation - previous.translation)
                        < config.POSITION_DEADBAND_M):
                    target.translation = previous.translation.copy()
                angle = np.linalg.norm(pin.log3(previous.rotation.T @ target.rotation))
                if angle < np.deg2rad(config.ROTATION_DEADBAND_DEG):
                    target.rotation = previous.rotation.copy()
        # 显式复制 SE3，防止后续修改 targets 时污染死区比较基准。
        self._accepted_targets = (pin.SE3(targets[0]), pin.SE3(targets[1]))
        return targets[0], targets[1]

    def solve_inverse_kinematics(
        self, tracking_input: TeleoperationInput
    ) -> A3UltraIKResult | None:
        """根据一帧左右 Motion Tracker 数据求解 14 个双臂关节角。

        返回 ``None`` 只表示当前帧无法形成有效输入；调用方此时应保持上一命令并
        尽快读取下一帧。成功返回并不代表误差严格降到 ``tolerance``，因为达到
        迭代上限、碰撞边界或回溯失败时会返回当前最安全的有限关节配置。
        """

        # 只在标定时读取头显方向，取水平投影，忽略俯仰和侧倾。
        heading_alignment = self._heading_alignment
        if heading_alignment is None:
            head_pose = self._parse_pico_pose(tracking_input.headset_pose)
            # 经固定轴转换后，头显局部 +X 就是其看向的前方。
            forward = None if head_pose is None else head_pose[1][:2, 0]
            if forward is None or np.linalg.norm(forward) < 0.1:
                if not self._heading_calibration_warned:
                    print("Waiting for heading calibration: enable Head tracking and look straight ahead.")
                    self.log_event("heading_calibration_waiting", level=logging.WARNING)
                    self._heading_calibration_warned = True
                return None
            x, y = forward / np.linalg.norm(forward)
            heading_alignment = np.array(((x, y, 0.), (-y, x, 0.), (0., 0., 1.)))

        # 先稳定左右设备顺序，再分别校验和转换两个 7 维位姿。
        selected_poses = self._select_tracker_poses(tracking_input, heading_alignment)
        if selected_poses is None:
            if self._tracking_unavailable_since_ns is None:
                self._tracking_unavailable_since_ns = tracking_input.captured_at_ns
            return None
        left_pose = self._parse_pico_pose(selected_poses[0])
        right_pose = self._parse_pico_pose(selected_poses[1])
        if left_pose is None or right_pose is None:
            if self._tracking_unavailable_since_ns is None:
                self._tracking_unavailable_since_ns = tracking_input.captured_at_ns
            return None

        # 头显和双 Tracker 均有效才提交标定；短时断连恢复不会重设朝向或零点。
        if self._heading_alignment is None:
            self._heading_alignment = heading_alignment
            yaw_deg = np.rad2deg(np.arctan2(heading_alignment[0, 1], heading_alignment[0, 0]))
            print(f"Heading calibrated: headset yaw {yaw_deg:.1f} deg; mapping is now fixed.")
            self.log_event("heading_calibrated", headset_yaw_deg=round(float(yaw_deg), 3))

        # 只根据数据不可用或设备时间戳停滞判断重连；本机 IK 耗时不会触发。
        reconnect_reason = None
        reconnect_gap_ns = 0
        if self._tracking_unavailable_since_ns is not None:
            reconnect_gap_ns = (
                tracking_input.captured_at_ns - self._tracking_unavailable_since_ns)
            if reconnect_gap_ns >= self.reconnect_jump_after_ns:
                reconnect_reason = "tracking_unavailable"
        self._tracking_unavailable_since_ns = None

        # 部分 APK 的 Motion 时间戳恒为 0，此时使用整包时间戳判断旧缓存。
        motion_timestamp_ns = tracking_input.motion_timestamp_ns or tracking_input.pico_timestamp_ns
        if motion_timestamp_ns is not None:
            if self._last_motion_timestamp_ns is None:
                self._motion_timestamp_changed_at_ns = tracking_input.captured_at_ns
            elif motion_timestamp_ns == self._last_motion_timestamp_ns:
                stalled_ns = (
                    tracking_input.captured_at_ns
                    - self._motion_timestamp_changed_at_ns)
                if (stalled_ns >= self.reconnect_jump_after_ns
                        and not self._motion_stream_stalled):
                    self._motion_stream_stalled = True
                    self.log_event(
                        "tracking_data_stalled", level=logging.WARNING,
                        stale_seconds=round(stalled_ns / 1e9, 3),
                        motion_timestamp_ns=motion_timestamp_ns,
                    )
            else:
                if self._motion_stream_stalled:
                    reconnect_reason = "motion_timestamp_resumed"
                    reconnect_gap_ns = (
                        tracking_input.captured_at_ns
                        - self._motion_timestamp_changed_at_ns)
                elif motion_timestamp_ns < self._last_motion_timestamp_ns:
                    reconnect_reason = "motion_timestamp_reset"
                self._motion_timestamp_changed_at_ns = tracking_input.captured_at_ns
                self._motion_stream_stalled = False
            self._last_motion_timestamp_ns = motion_timestamp_ns

        if reconnect_reason is not None:
            self.low_pass_filter.reset()
            self.rotation_low_pass_filter.reset()
            print("Motion Tracker reconnected; allowing an immediate IK update.")
            self.log_event(
                "tracking_reconnected", filter_state_reset=True,
                reason=reconnect_reason,
                gap_seconds=round(reconnect_gap_ns / 1e9, 3),
            )
        # 两只手成组使用同一滤波器更新，保证滤波状态的形状和时间一致。
        positions = self.apply_low_pass_filter(np.stack((left_pose[0], right_pose[0])))
        rotations = self.apply_rotation_low_pass_filter(
            np.stack((left_pose[1], right_pose[1]))
        )
        tracker_poses = (
            (positions[0], rotations[0]), (positions[1], rotations[1])
        )

        # 第一帧同时记录人侧 Tracker 和机器人侧双掌，建立一一对应的相对零点。
        if self._pico_reference is None:
            self._pico_reference = tuple(
                (p.copy(), r.copy()) for p, r in tracker_poses
            )
            self._palm_reference = self._ik.palm_poses()

        # PICO 层只构造双掌目标，机器人模型、碰撞和数值迭代均由求解器负责。
        return self._ik.solve(self._tracker_targets(tracker_poses))

    def send_to_robot(self, robot_command: Any) -> None:
        """把最新双臂目标交给实机 100 Hz 线程，不逐帧重新规划。"""

        if self._motion_stream_stalled:
            self.real_robot.stop()
            return
        self.real_robot.stream_joint_positions(
            robot_command.joint_names, robot_command.joint_positions)

    def run_real_robot(self) -> None:
        """规划归位后标定 PICO，以 IK 为参考经 SONIC 控制真机并显示反馈。"""

        from pico_teleop_interface.robot_interfaces.a3_ultra.sim import run_simulation

        self.real_robot.dds_profile = (
            self.real_robot.dds_profile
            or Path(__file__).resolve().parents[3] / "usage/ros_dds_configuration.xml")
        self.real_robot.max_joint_speed = config.REAL_ROBOT_TELEOP_SPEED_RAD_S
        # IK 输出遵守实机限位，避免每帧目标在通信边界被拒绝。
        indices = self._ik._arm_q_indices
        self._ik.model.lowerPositionLimit[indices] = self.real_robot._planner.lower
        self._ik.model.upperPositionLimit[indices] = self.real_robot._planner.upper
        run_simulation(self, real_robot=True)

    def close(self) -> None:
        """退出仿真或遥操作时同时释放 ROS 和 PICO SDK。"""

        try:
            self.real_robot.close()
        finally:
            super().close()

    def run_sim(self, model_path: str | Path | None = None) -> None:
        """分别启动 IK 和 SONIC 两个 MuJoCo 进程并发送同一 IK 目标。"""

        from pico_teleop_interface.robot_interfaces.a3_ultra.sim import run_comparison_simulation

        run_comparison_simulation(self, model_path=model_path)

    def run_sonic_sim(self) -> None:
        """运行固定身体、仅执行双臂 SONIC 输出的实验。"""
        from pico_teleop_interface.robot_interfaces.a3_ultra.sonic_sim import run_sonic_simulation

        run_sonic_simulation(self)


if __name__ == "__main__":
    robot = A3UltraInterface()
    robot.run_real_robot()
