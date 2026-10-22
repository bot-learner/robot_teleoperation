"""A3 Ultra 的 PICO 双臂 MuJoCo PD 动力学预览。"""

from __future__ import annotations

import argparse
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import logging
import multiprocessing as mp
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from queue import Empty, Full
from typing import TYPE_CHECKING, Sequence

import mujoco
import mujoco.viewer
import numpy as np

from . import config
from .dual_arm_ik import (
    ARM_JOINT_NAMES,
    DEFAULT_ARM_JOINT_POSITIONS,
    A3UltraIKResult,
)
from .a3_ultra_interface import A3UltraInterface

if TYPE_CHECKING:
    from mujoco.viewer import Handle


DEFAULT_MODEL_PATH = (
    Path(__file__).resolve().with_name("sim")
    / "model"
    / "mjcf"
    / "a3_ultra_t3d0.xml"
)


def _furthest_collision_free_target(ik, start, target):
    """返回从上一安全目标出发、首次碰撞边界之前的最远关节目标。"""

    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    if start.shape != target.shape or not np.isfinite((start, target)).all():
        raise ValueError("Invalid SONIC collision projection target")

    indices = ik._arm_q_indices

    def clearance(positions):
        configuration = ik._q.copy()
        configuration[indices] = positions
        return ik._minimum_collision_distance(configuration)[0]

    start_distance = clearance(start)
    if start_distance < ik.collision_min_distance:
        return start.copy(), start_distance, 0.0

    delta = target - start
    checks = max(1, int(np.ceil(np.max(np.abs(delta)) / 0.02)))
    safe_fraction, safe_distance = 0.0, start_distance
    for fraction in np.linspace(1.0 / checks, 1.0, checks):
        distance = clearance(start + fraction * delta)
        if distance >= ik.collision_min_distance:
            safe_fraction, safe_distance = fraction, distance
            continue

        unsafe_fraction = fraction
        for _ in range(8):
            middle = 0.5 * (safe_fraction + unsafe_fraction)
            middle_distance = clearance(start + middle * delta)
            if middle_distance >= ik.collision_min_distance:
                safe_fraction, safe_distance = middle, middle_distance
            else:
                unsafe_fraction = middle
        return start + safe_fraction * delta, safe_distance, safe_fraction
    return target.copy(), safe_distance, 1.0


class A3UltraSimulator:
    """加载官方 MJCF，固定非双臂关节并通过 PD 力矩控制双臂。"""

    def __init__(self, model_path: str | Path | None = None) -> None:
        self.model_path = Path(model_path or DEFAULT_MODEL_PATH).resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"未找到 A3 Ultra MuJoCo 模型：{self.model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)

        self._arm_qpos_addresses = {
            name: self._require_scalar_joint_qpos_address(name)
            for name in ARM_JOINT_NAMES
        }
        self.apply_joint_positions(ARM_JOINT_NAMES, DEFAULT_ARM_JOINT_POSITIONS)
        self._place_feet_on_ground()
        mujoco.mj_forward(self.model, self.data)
        self._fix_non_arm_joints()
        self._arm_q = np.array([self._arm_qpos_addresses[n] for n in ARM_JOINT_NAMES])
        self._arm_v = np.array([
            self.model.jnt_dofadr[self.model.joint(n).id] for n in ARM_JOINT_NAMES
        ])
        self._arm_actuators = np.array([
            self.model.actuator(n).id for n in ARM_JOINT_NAMES
        ])
        # low_level_motion_control.zip: a3_policy_parameters.hpp a3_kps/a3_kds.
        self.arm_kp = np.tile([40., 40., 30., 30., 30., 20., 20.], 2)
        self.arm_kd = np.tile([3., 3., 2., 2., 2., 2., 2.], 2)
        # Effort limits from the archive's a3_ultra_t3d0/urdf/model.urdf.
        self.arm_torque_limits = np.tile([60., 60., 24., 24., 24., 6., 6.], 2)
        self._arm_targets = self.data.qpos[self._arm_q].copy()
        self._step_remainder = 0.0

    def _fix_non_arm_joints(self) -> None:
        """固定支撑身体；双臂使用真实动力学，不依赖全身平衡控制。"""
        root = ET.parse(self.model_path).getroot()
        compiler = root.find('compiler')
        for key in ('meshdir', 'texturedir'):
            if compiler is not None and compiler.get(key):
                compiler.set(key, str((self.model_path.parent / compiler.get(key)).resolve()))
        for include in root.iter('include'):
            include.set('file', str((self.model_path.parent / include.get('file')).resolve()))
        equality = root.find('equality')
        if equality is None:
            equality = ET.SubElement(root, 'equality')
        for joint_id in range(self.model.njnt):
            joint = self.model.joint(joint_id)
            if joint.name in ARM_JOINT_NAMES:
                continue
            address = int(self.model.jnt_qposadr[joint_id])
            if self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
                body_name = self.model.body(self.model.jnt_bodyid[joint_id]).name
                pose = self.data.qpos[address:address + 7]
                ET.SubElement(equality, 'weld', body1='world', body2=body_name,
                              relpose=' '.join(map(str, pose)))
            elif int(self.model.jnt_type[joint_id]) in (
                int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
            ):
                value = self.data.qpos[address]
                ET.SubElement(equality, 'joint', joint1=joint.name,
                              polycoef=f'{value} 0 0 0 0')
            # Passive ball joints in closed-chain rods retain their own constraints.
        initial_q = self.data.qpos.copy()
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = initial_q
        mujoco.mj_forward(self.model, self.data)

    def _require_scalar_joint_qpos_address(self, name: str) -> int:
        joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
            raise ValueError(f"MuJoCo 模型中缺少关节：{name}")
        if int(self.model.jnt_type[joint_id]) not in (
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        ):
            raise ValueError(f"MuJoCo 关节 {name} 不是单自由度关节")
        return int(self.model.jnt_qposadr[joint_id])

    def _place_feet_on_ground(self) -> None:
        foot_geoms = [
            geom_id
            for geom_id in range(self.model.ngeom)
            if (name := mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            ))
            and name.startswith(("left_foot", "right_foot"))
            and name.endswith("_collision")
        ]
        free_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "free_joint"
        )
        if not foot_geoms or free_joint_id < 0:
            raise ValueError("MuJoCo 模型缺少脚底碰撞体或 free_joint")

        mujoco.mj_forward(self.model, self.data)
        foot_bottom = min(
            self.data.geom_xpos[geom_id, 2] - self.model.geom_size[geom_id, 0]
            for geom_id in foot_geoms
        )
        root_qpos_address = int(self.model.jnt_qposadr[free_joint_id])
        self.data.qpos[root_qpos_address + 2] -= foot_bottom
        mujoco.mj_forward(self.model, self.data)

    def apply_joint_positions(
        self, joint_names: Sequence[str], joint_positions: Sequence[float]
    ) -> None:
        """设置双臂 PD 位置目标；初始化阶段才直接写入初始位置。"""

        positions = np.asarray(joint_positions, dtype=float)
        if positions.shape != (len(joint_names),):
            raise ValueError("关节名称和位置数量不一致")
        if not np.all(np.isfinite(positions)):
            raise ValueError("关节位置包含非有限值")

        unknown = set(joint_names) - self._arm_qpos_addresses.keys()
        if unknown:
            raise ValueError(f"不支持的仿真关节：{', '.join(sorted(unknown))}")

        for name, position in zip(joint_names, positions):
            if hasattr(self, '_arm_targets'):
                index = ARM_JOINT_NAMES.index(name)
                joint = self.model.joint(name)
                self._arm_targets[index] = np.clip(position, *joint.range)
            else:
                self.data.qpos[self._arm_qpos_addresses[name]] = position
        if not hasattr(self, '_arm_targets'):
            mujoco.mj_forward(self.model, self.data)

    def step(self, duration: float) -> None:
        """每个物理步执行 PD、重力/科氏补偿和力矩限幅。"""
        if not np.isfinite(duration) or duration < 0:
            raise ValueError('仿真步进时间必须是有限非负数')
        self._step_remainder += duration
        dt = self.model.opt.timestep
        steps = int((self._step_remainder + 1e-12) / dt)
        self._step_remainder -= steps * dt
        for _ in range(steps):
            mujoco.mj_forward(self.model, self.data)
            torque = (
                self.arm_kp * (self._arm_targets - self.data.qpos[self._arm_q])
                - self.arm_kd * self.data.qvel[self._arm_v]
                + self.data.qfrc_bias[self._arm_v]
            )
            self.data.ctrl[self._arm_actuators] = np.clip(
                torque, -self.arm_torque_limits, self.arm_torque_limits
            )
            mujoco.mj_step(self.model, self.data)

    def apply_ik_result(self, result: A3UltraIKResult) -> None:
        """把一次 A3 Ultra IK 结果应用到 MuJoCo。"""

        self.apply_joint_positions(result.joint_names, result.joint_positions)


def _configure_camera(viewer: Handle, simulator: A3UltraSimulator) -> None:
    torso_id = mujoco.mj_name2id(
        simulator.model, mujoco.mjtObj.mjOBJ_BODY, "torso_Link"
    )
    if torso_id >= 0:
        viewer.cam.lookat[:] = simulator.data.xpos[torso_id]
    viewer.cam.distance = 3.0
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -15.0


def _calibration_countdown(seconds: int = 6, *, robot=None):
    print("Enable Head and Object Tracking; stand upright and look straight ahead.")
    print("Hold both arms forward with elbows slightly bent; do not cross them.")
    if robot is not None:
        robot.monitor.set_calibration_countdown(seconds)
    deadline = time.monotonic() + seconds
    last_remaining = None
    calibration_input = None
    while time.monotonic() < deadline:
        remaining = min(seconds, int(deadline - time.monotonic()) + 1)
        if remaining != last_remaining:
            print(f"Calibration starts in {remaining}...")
            last_remaining = remaining
        if robot is not None:
            sample_started = time.monotonic()
            tracking_input = robot.read_tracking_input()
            calibration_input = tracking_input
            robot.monitor.update(
                tracking_input, False, time.monotonic() - sample_started)
        if robot is not None and robot.real_robot._connected:
            robot.monitor.set_robot_feedback(
                robot.real_robot._state_received_at, robot.real_robot.state_timeout,
                robot.real_robot._bridge_ready_at,
                config.REAL_ROBOT_BRIDGE_READY_TIMEOUT_SECONDS)
        time.sleep(.05)
    print("Calibration captured the headset direction and Tracker zero points at countdown end.")
    return calibration_input


def _take_latest(command_queue):
    """丢弃过期命令，只返回队列中最新的 IK 结果。"""

    latest = None
    while True:
        try:
            latest = command_queue.get_nowait()
        except Empty:
            return latest


def _publish_latest(command_queue, result: A3UltraIKResult) -> None:
    """队列满时替换旧命令，避免仿真落后于操作者。"""

    try:
        command_queue.put_nowait(result)
        return
    except Full:
        pass
    try:
        command_queue.get_nowait()
    except Empty:
        pass
    try:
        command_queue.put_nowait(result)
    except Full:
        pass


def run_ik_viewer(command_queue, stop_event, ready_event,
                  model_path: str | Path | None = None) -> None:
    """使用与 SONIC 窗口相同的 Ultra T3D0 模型，直接由 IK + PD 驱动。"""

    from .sonic_sim import SonicUpperBodySimulator

    simulator = None
    try:
        simulator = SonicUpperBodySimulator(use_policy=False, model_path=model_path)
        with mujoco.viewer.launch_passive(
                simulator.model, simulator.data,
                show_left_ui=False, show_right_ui=False) as viewer:
            _configure_camera(viewer, simulator)
            viewer.set_texts((None, None, "IK + PD", "Direct joint targets"))
            viewer.sync()
            ready_event.set()
            while viewer.is_running() and not stop_event.is_set():
                started = time.monotonic()
                if (result := _take_latest(command_queue)) is not None:
                    with viewer.lock():
                        simulator.apply_ik_result(result)
                with viewer.lock():
                    simulator.step(1.0 / config.CONTROL_RATE_HZ)
                viewer.sync()
                time.sleep(max(0.0, 1.0 / config.CONTROL_RATE_HZ
                               - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        ready_event.set()
        stop_event.set()
        if simulator is not None:
            simulator.close()


def run_comparison_simulation(robot: A3UltraInterface,
                              model_path: str | Path | None = None) -> None:
    """读取一次 PICO/IK，并用两个独立进程对比 IK 与 SONIC。"""

    from .sonic_sim import run_sonic_viewer

    context = mp.get_context("spawn")
    stop_event = context.Event()
    ready_events = (context.Event(), context.Event())
    command_queues = (context.Queue(maxsize=1), context.Queue(maxsize=1))
    processes = (
        context.Process(
            name="a3-ik-mujoco", target=run_ik_viewer,
            args=(command_queues[0], stop_event, ready_events[0], model_path)),
        context.Process(
            name="a3-sonic-mujoco", target=run_sonic_viewer,
            args=(command_queues[1], stop_event, ready_events[1], model_path)),
    )
    try:
        if robot._auto_start_pc_service:
            robot.start_pc_service()
        robot.monitor.mujoco_window_count = 2
        robot.initialize()
        for process in processes:
            process.start()

        deadline = time.monotonic() + 60.0
        while not all(event.is_set() for event in ready_events):
            failed = next((p for p in processes if p.exitcode is not None), None)
            if failed is not None:
                raise RuntimeError(
                    f"{failed.name} failed during startup (exit code {failed.exitcode})")
            if time.monotonic() >= deadline:
                raise TimeoutError("MuJoCo viewers did not start within 60 seconds")
            time.sleep(0.05)
        failed = next((p for p in processes if not p.is_alive()), None)
        if failed is not None:
            raise RuntimeError(
                f"{failed.name} failed during startup (exit code {failed.exitcode})")

        calibration_input = _calibration_countdown(robot=robot)
        robot.reset_ik_reference()
        if calibration_input is not None:
            robot.solve_inverse_kinematics(calibration_input)
        robot.log_event(
            "comparison_simulation_started",
            model_path=str(Path(model_path or DEFAULT_MODEL_PATH).resolve()))
        print("Two MuJoCo processes started: IK + PD and SONIC.")
        while (not stop_event.is_set()
               and all(process.is_alive() for process in processes)):
            cycle_started = time.monotonic()
            tracking_input = robot.read_tracking_input()
            tracking_read_finished = time.monotonic()
            result = robot.solve_inverse_kinematics(tracking_input)
            ik_finished = time.monotonic()
            robot.record_teleoperation_frame(
                tracking_input, result, ik_finished - cycle_started,
                sdk_read_seconds=tracking_read_finished - cycle_started,
                ik_seconds=ik_finished - tracking_read_finished)
            if result is None:
                continue
            for command_queue in command_queues:
                _publish_latest(command_queue, result)
            time.sleep(max(0.0, robot._period_seconds
                           - (time.monotonic() - cycle_started)))
        failed = next((p for p in processes if p.exitcode not in (None, 0)), None)
        if failed is not None:
            raise RuntimeError(
                f"{failed.name} stopped with exit code {failed.exitcode}")
    except KeyboardInterrupt:
        robot.log_event("comparison_simulation_interrupted")
    except BaseException:
        robot.log_event(
            "comparison_simulation_failed", level=logging.ERROR, exc_info=True)
        raise
    finally:
        stop_event.set()
        for process in processes:
            if process.pid is not None:
                process.join(timeout=5.0)
                if process.is_alive():
                    process.terminate()
                    process.join()
        for command_queue in command_queues:
            command_queue.close()
        robot.log_event("comparison_simulation_stopped")
        robot.close()


def run_simulation(
    robot: A3UltraInterface, model_path: str | Path | None = None,
    *, real_robot: bool = False,
) -> None:
    """读取 PICO 并求解 IK；实机模式发送目标，同时用反馈角度刷新 MuJoCo。"""

    simulator = None
    try:
        if real_robot:
            from .sonic_sim import SonicUpperBodySimulator
            if not np.isclose(robot._period_seconds, .02):
                raise ValueError("Real SONIC teleoperation requires a 50 Hz loop")
            simulator = SonicUpperBodySimulator(model_path=model_path)
        else:
            simulator = A3UltraSimulator(model_path)
        if robot._auto_start_pc_service:
            robot.start_pc_service()
        robot.initialize()
        if real_robot:
            robot.real_robot.connect()
        robot.reset_ik_reference()
        robot.log_event(
            "mujoco_simulation_started", model_path=str(simulator.model_path))
        print(
            "MuJoCo simulation started. "
            "Move both Motion Trackers; close the window to stop."
        )
        with mujoco.viewer.launch_passive(
                simulator.model, simulator.data,
                show_left_ui=False, show_right_ui=False) as viewer:
            _configure_camera(viewer, simulator)
            viewer.sync()
            if real_robot:
                print("Planning collision-aware return to the default arm pose...")
                robot.monitor.set_stage("PLANNING HOME")
                # 规划在后台进行，归位全程显示真实角度，窗口关闭时取消尚未提交的路径。
                with ThreadPoolExecutor(max_workers=1) as executor:
                    home = executor.submit(robot.real_robot.move_to_default_pose)
                    try:
                        while viewer.is_running():
                            # 归位期间也观察 PICO，但不求 IK、不用它控制真机。
                            sample_started = time.monotonic()
                            tracking_input = robot.read_tracking_input()
                            robot.monitor.update(
                                tracking_input, False, time.monotonic() - sample_started)
                            robot.monitor.set_robot_feedback(
                                robot.real_robot._state_received_at, robot.real_robot.state_timeout,
                                robot.real_robot._bridge_ready_at,
                                config.REAL_ROBOT_BRIDGE_READY_TIMEOUT_SECONDS)
                            if home.done():
                                home.result()
                                if not robot.real_robot.is_moving:
                                    break
                            measured = robot.real_robot.read_joint_positions(ARM_JOINT_NAMES)
                            if home.done():
                                error_deg = np.rad2deg(np.max(np.abs(
                                    np.asarray(measured) - DEFAULT_ARM_JOINT_POSITIONS)))
                                robot.monitor.set_stage(f"HOMING: error {error_deg:.1f} deg")
                            with viewer.lock():
                                simulator.data.qpos[simulator._arm_q] = measured
                                mujoco.mj_forward(simulator.model, simulator.data)
                            viewer.sync()
                            time.sleep(robot._period_seconds)
                        else:
                            return
                    finally:
                        # 异常或关窗时先撤销归位，再等待后台规划结束，防止迟到的路径启动。
                        robot.real_robot.stop()
                print("Default arm pose reached. Prepare both Motion Trackers for calibration.")
            calibration_input = _calibration_countdown(robot=robot)
            if real_robot:
                # 归位后以实际反馈作为人机零点，避免残余跟踪误差造成接管跳变。
                initial = robot.real_robot.read_joint_positions(ARM_JOINT_NAMES)
                robot._ik._q[robot._ik._arm_q_indices] = initial
                simulator.data.qpos[simulator._arm_q] = initial
                mujoco.mj_forward(simulator.model, simulator.data)
                positions, velocities = robot.real_robot.read_arm_feedback()
                simulator.reset_real_feedback(positions, velocities)
                robot.reset_ik_reference()
                if calibration_input is not None:
                    robot.solve_inverse_kinematics(calibration_input)
                robot.log_event("real_teleoperation_started", initial_joint_positions=initial)
                print("Real robot SONIC teleoperation enabled: PICO -> IK -> SONIC -> real arms.")
            else:
                robot.reset_ik_reference()
                if calibration_input is not None:
                    robot.solve_inverse_kinematics(calibration_input)
            last_waiting_message = 0.0
            received_first_frame = False
            last_safe_sonic_target = None
            robot.monitor.set_stage("WAITING FOR HEAD + 2 TRACKERS")
            while viewer.is_running():
                if real_robot:
                    robot.monitor.set_robot_feedback(
                        robot.real_robot._state_received_at, robot.real_robot.state_timeout,
                        robot.real_robot._bridge_ready_at,
                        config.REAL_ROBOT_BRIDGE_READY_TIMEOUT_SECONDS)
                cycle_started = time.monotonic()
                tracking_input = robot.read_tracking_input()
                tracking_read_finished = time.monotonic()
                result = robot.solve_inverse_kinematics(tracking_input)
                ik_finished = time.monotonic()
                if real_robot and result is not None and not robot._motion_stream_stalled:
                    positions, velocities = robot.real_robot.read_arm_feedback()
                    sonic_started = time.monotonic()
                    targets = simulator.infer_real_arm_targets(result, positions, velocities)
                    indices = robot._ik._arm_q_indices
                    targets = np.clip(targets,
                        robot._ik.model.lowerPositionLimit[indices],
                        robot._ik.model.upperPositionLimit[indices])
                    if last_safe_sonic_target is None:
                        # IK 结果已通过硬碰撞边界，用它初始化 SONIC 的安全连续路径。
                        last_safe_sonic_target = np.asarray(result.joint_positions)
                    safe_targets, distance, safe_fraction = (
                        _furthest_collision_free_target(
                            robot._ik, last_safe_sonic_target, targets))
                    robot.real_robot._record(
                        "sonic_target", ik_reference=result.joint_positions,
                        position=targets, accepted_position=safe_targets,
                        safe_fraction=safe_fraction,
                        inference_ms=(time.monotonic() - sonic_started) * 1000,
                        collision_distance_m=distance)
                    if safe_fraction < 1.0:
                        robot.monitor.set_stage("SONIC COLLISION: CLIPPED")
                        robot.real_robot._record(
                            "sonic_collision_clipped", distance_m=distance,
                            safe_fraction=safe_fraction)
                    else:
                        robot.monitor.set_stage("TELEOPERATION: SONIC")
                    last_safe_sonic_target = safe_targets
                    result = replace(result, joint_positions=safe_targets)
                robot.record_teleoperation_frame(
                    tracking_input, result, time.monotonic() - cycle_started,
                    sdk_read_seconds=tracking_read_finished - cycle_started,
                    ik_seconds=ik_finished - tracking_read_finished)
                if result is not None:
                    if not received_first_frame:
                        print(
                            "Motion Tracker data received; "
                            + ("real arms now follow IK through SONIC."
                               if real_robot else "simulation is now following IK.")
                        )
                        received_first_frame = True
                        robot.monitor.set_stage("TELEOPERATION: SONIC" if real_robot else "TELEOPERATION")
                    if real_robot:
                        robot.send_to_robot(result)
                    else:
                        with viewer.lock():
                            simulator.apply_ik_result(result)
                elif real_robot:
                    robot.real_robot.stop()
                elif cycle_started - last_waiting_message >= 2.0:
                    print("Waiting for two valid Motion Trackers in Object Tracking mode...")
                    last_waiting_message = cycle_started
                with viewer.lock():
                    if real_robot:
                        simulator.data.qpos[simulator._arm_q] = robot.real_robot.read_joint_positions(ARM_JOINT_NAMES)
                        mujoco.mj_forward(simulator.model, simulator.data)
                    else:
                        simulator.step(robot._period_seconds)
                viewer.sync()

                remaining = robot._period_seconds - (
                    time.monotonic() - cycle_started
                )
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        robot.log_event("mujoco_simulation_interrupted")
    except BaseException:
        robot.log_event(
            "mujoco_simulation_failed", level=logging.ERROR, exc_info=True)
        raise
    finally:
        robot.log_event("mujoco_simulation_stopped")
        if real_robot and simulator is not None:
            simulator.close()
        robot.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Preview PICO-driven A3 Ultra dual-arm IK in MuJoCo."
    )
    parser.add_argument("--rate", type=float, default=config.CONTROL_RATE_HZ,
                        help="Control rate in Hz")
    parser.add_argument(
        "--position-scale",
        type=float,
        default=config.POSITION_SCALE,
        help="Scale applied to Motion Tracker translation",
    )
    parser.add_argument("--left-tracker-serial", help="Left Motion Tracker serial")
    parser.add_argument("--right-tracker-serial", help="Right Motion Tracker serial")
    args = parser.parse_args()

    robot = A3UltraInterface(
        control_rate_hz=args.rate,
        position_scale=args.position_scale,
        left_tracker_serial=args.left_tracker_serial,
        right_tracker_serial=args.right_tracker_serial,
    )
    robot.run_sim()


if __name__ == "__main__":
    main()
