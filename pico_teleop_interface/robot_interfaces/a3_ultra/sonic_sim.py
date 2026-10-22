"""PICO → 双臂 IK → SONIC → 双臂 PD 的固定身体仿真实验。"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import mujoco
import mujoco.viewer
import numpy as np

from . import config
from .a3_ultra_interface import A3UltraInterface
from .dual_arm_ik import ARM_JOINT_NAMES
from ..base_interface import TeleoperationInput, TrackingMode
from .sim import (
    A3UltraSimulator,
    _calibration_countdown,
    _configure_camera,
    _take_latest,
)


SONIC_ROOT = Path(__file__).resolve().parents[2] / 'third_party' / 'sonic_for_a3'


class SonicUpperBodySimulator(A3UltraSimulator):
    """共用 Ultra T3D0 模型和 PD；由 SONIC 或直接 IK 提供双臂目标。"""

    def __init__(self, *, use_policy: bool = True,
                 model_path: str | Path | None = None) -> None:
        self.use_policy = use_policy
        checkpoint = SONIC_ROOT / 'checkpoints/035_step200000/model_step_200000.pt'
        if use_policy and not checkpoint.is_file():
            raise FileNotFoundError(f'SONIC checkpoint not found: {checkpoint}')
        sys.path.insert(0, str(SONIC_ROOT))
        try:
            import torch
            from gear_sonic.scripts import sim2sim_a3_mujoco as sonic
        except ImportError as exc:
            raise RuntimeError(
                'Missing SONIC dependencies. Activate a3_teleop and install the '
                'CPU dependencies listed in usage/sonic_for_a3.md. '
                f'Import failed: {exc}'
            ) from exc
        self.sonic = sonic
        # CPU inference: avoid excessive thread-launch overhead for single samples.
        torch.set_num_threads(1)
        self.device = torch.device('cpu')
        self.policy = (sonic.A3Policy(checkpoint, self.device, encoder='a3_fast')
                       if use_policy else None)
        super().__init__(model_path=model_path)
        # 上游 URDF 只定义 checkpoint 的输入排列；实际关节状态来自 Ultra MJCF。
        il_names, _ = sonic.load_urdf_actuated_joints(sonic.DEFAULT_URDF)
        if len(il_names) != sonic.NUM_POLICY_DOFS or len(set(il_names)) != len(il_names):
            raise ValueError('SONIC requires 29 unique policy joint names')
        self._policy_q = np.array([
            self._require_scalar_joint_qpos_address(name) for name in il_names])
        self._policy_v = np.array([
            self.model.jnt_dofadr[self.model.joint(name).id] for name in il_names])
        self._arm_il = np.array([il_names.index(name) for name in ARM_JOINT_NAMES])
        self._default_q_il = np.array([sonic.DEFAULT_Q_BY_NAME[name] for name in il_names])
        self._action_scale_il = np.array([sonic.get_a3_action_scale(name) for name in il_names])
        self._reference_il = self.data.qpos[self._policy_q].copy()
        self._anchor_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis_link')
        if self._anchor_body_id < 0:
            raise ValueError('Ultra model is missing pelvis_link')
        self._anchor_reference = sonic.body_quat_wxyz(
            self.data, self._anchor_body_id).copy()
        self.last_action = np.zeros(sonic.NUM_POLICY_DOFS)
        self.history = sonic.prime_history(self._observation()) if use_policy else None

    def _observation(self):
        """从 Ultra 的实际关节直接构造策略观测，不进行 A3 闭链电机换算。"""

        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
            self._anchor_body_id, velocity, 1)
        anchor = self.sonic.body_quat_wxyz(self.data, self._anchor_body_id)
        terms = {
            'base_ang_vel': velocity[:3],
            'joint_pos': self.data.qpos[self._policy_q] - self._default_q_il,
            'joint_vel': self.data.qvel[self._policy_v],
            'actions': self.last_action.copy(),
            'gravity_dir': self.sonic.quat_rotate_inverse_wxyz(
                anchor, np.array([0., 0., -1.])),
        }
        for name, values in terms.items():
            if values.shape != (self.sonic.OBS_TERM_DIMS[name],) or not np.isfinite(values).all():
                raise ValueError(f'Invalid Ultra policy observation: {name}')
        return {name: values.astype(np.float32) for name, values in terms.items()}

    def encoder_input(self) -> np.ndarray:
        # Hold-last future window, not a prediction: q constant, dq zero.
        reference = SimpleNamespace(
            num_frames=10,
            dof_il=np.tile(self._reference_il, (10, 1)),
            dof_vel_il=np.zeros((10, 29)),
            anchor_quat_wxyz=np.tile(self._anchor_reference, (10, 1)),
        )
        anchor = self.sonic.body_quat_wxyz(self.data, self._anchor_body_id)
        return self.sonic.build_encoder_input(reference, 0, anchor, frame_skip=1)

    def apply_ik_result(self, result) -> None:
        if tuple(result.joint_names) != ARM_JOINT_NAMES:
            raise ValueError('SONIC requires the standard A3 dual-arm joint order')
        q = np.asarray(result.joint_positions, dtype=float)
        if q.shape != (14,) or not np.all(np.isfinite(q)):
            raise ValueError('Invalid IK reference for SONIC')
        self._reference_il[self._arm_il] = q

    def step(self, duration: float = 0.02) -> None:
        if not np.isclose(duration, self.sonic.POLICY_DT):
            raise ValueError('SONIC upper-body mode requires a 50 Hz policy loop')
        if self.use_policy:
            raw = self.policy.act(
                self.encoder_input(), self.sonic.flatten_history(self.history), self.device
            )
            raw = np.clip(raw, -self.sonic.ACTION_CLIP, self.sonic.ACTION_CLIP)
            target_il = self._default_q_il + raw * self._action_scale_il
            arm_targets = target_il[self._arm_il]
        else:
            arm_targets = self._reference_il[self._arm_il]
        # 两种控制链路共用完全相同的 PD、力矩限幅和物理步进。
        self.apply_joint_positions(ARM_JOINT_NAMES, arm_targets)
        substeps = round(duration / self.model.opt.timestep)
        for _ in range(substeps):
            mujoco.mj_forward(self.model, self.data)
            torque = (self.arm_kp * (self._arm_targets - self.data.qpos[self._arm_q])
                      - self.arm_kd * self.data.qvel[self._arm_v])
            self.data.ctrl[:] = 0.0
            self.data.ctrl[self._arm_actuators] = np.clip(
                torque, -self.arm_torque_limits, self.arm_torque_limits
            )
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        # History contains the actions actually applied; discarded body actions are zero.
        if self.use_policy:
            self.last_action[:] = 0.0
            self.last_action[self._arm_il] = raw[self._arm_il]
            self.sonic.append_history(self.history, self._observation())
        if not np.all(np.isfinite(self.data.qpos)) or np.any(self.data.warning.number):
            raise RuntimeError('MuJoCo instability detected; stopping SONIC experiment')

    def reset_real_feedback(self, positions, velocities) -> None:
        """以真机归位后的反馈初始化策略历史，不推进 MuJoCo 动力学。"""
        feedback = np.asarray((positions, velocities), dtype=float)
        if feedback.shape != (2, 14) or not np.isfinite(feedback).all():
            raise ValueError('Invalid real arm feedback for SONIC')
        self.data.qpos[self._arm_q] = feedback[0]
        self.data.qvel[self._arm_v] = feedback[1]
        mujoco.mj_forward(self.model, self.data)
        self._reference_il = self.data.qpos[self._policy_q].copy()
        self.last_action[:] = 0.
        self.history = self.sonic.prime_history(self._observation())

    def infer_real_arm_targets(self, result, positions, velocities) -> np.ndarray:
        """IK 为参考，真机角度/速度为观测，仅返回双臂策略目标，不发指令。"""
        feedback = np.asarray((positions, velocities), dtype=float)
        if feedback.shape != (2, 14) or not np.isfinite(feedback).all():
            raise ValueError('Invalid real arm feedback for SONIC')
        self.data.qpos[self._arm_q] = feedback[0]
        self.data.qvel[self._arm_v] = feedback[1]
        mujoco.mj_forward(self.model, self.data)
        self.apply_ik_result(result)
        self.sonic.append_history(self.history, self._observation())
        raw = np.asarray(self.policy.act(
            self.encoder_input(), self.sonic.flatten_history(self.history), self.device))
        if raw.shape != (self.sonic.NUM_POLICY_DOFS,) or not np.isfinite(raw).all():
            raise ValueError('Invalid SONIC action; real command rejected')
        raw = np.clip(raw, -self.sonic.ACTION_CLIP, self.sonic.ACTION_CLIP)
        targets = (self._default_q_il + raw * self._action_scale_il)[self._arm_il]
        self.last_action[:] = 0.
        self.last_action[self._arm_il] = raw[self._arm_il]
        return targets

    def close(self) -> None:
        self.policy = None
        self.history = None


def mock_tracking_input(frame: int) -> TeleoperationInput:
    """小幅前伸/收回的双 Tracker 输入，第一帧作为相对零点。"""
    forward = 0.015 * np.sin(frame * 0.02 * 2 * np.pi * 0.2)
    poses = np.array([[-.25, 1.2, .4 - forward, 0, 0, 0, 1],
                      [.25, 1.2, .4 - forward, 0, 0, 0, 1]])
    return TeleoperationInput(
        captured_at_ns=time.monotonic_ns(), tracking_mode=TrackingMode.OBJECT,
        headset_pose=[0., 1.6, 0., 0., 0., 0., 1.],
        left_controller_pose=None, right_controller_pose=None,
        left_hand_state=None, right_hand_state=None, body_pose=None, body_available=False,
        motion_tracker_pose=poses, motion_tracker_velocity=None,
        motion_tracker_acceleration=None, motion_tracker_serial_numbers=('LEFT', 'RIGHT'),
        motion_tracker_count=2, motion_timestamp_ns=frame * 20_000_000,
        left_hand_active=False, right_hand_active=False,
    )


def run_sonic_viewer(command_queue, stop_event, ready_event,
                     model_path: str | Path | None = None) -> None:
    """在独立进程中显示由同一 IK 参考驱动的 SONIC MuJoCo。"""

    simulator = None
    try:
        simulator = SonicUpperBodySimulator(model_path=model_path)
        with mujoco.viewer.launch_passive(
                simulator.model, simulator.data,
                show_left_ui=False, show_right_ui=False) as viewer:
            _configure_camera(viewer, simulator)
            viewer.set_texts((None, None, "SONIC", "Learned joint targets"))
            viewer.sync()
            ready_event.set()
            while viewer.is_running() and not stop_event.is_set():
                started = time.monotonic()
                if (result := _take_latest(command_queue)) is not None:
                    simulator.apply_ik_result(result)
                with viewer.lock():
                    simulator.step()
                viewer.sync()
                time.sleep(max(0.0, simulator.sonic.POLICY_DT
                               - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        ready_event.set()
        stop_event.set()
        if simulator is not None:
            simulator.close()


def run_sonic_simulation(robot: A3UltraInterface, *, mock: bool = False,
                         headless: bool = False, steps: int | None = None,
                         metrics_path: Path | None = None) -> dict:
    simulator = None
    viewer = None
    errors = []
    try:
        if not np.isclose(robot.control_rate_hz, config.CONTROL_RATE_HZ):
            raise ValueError(
                f'SONIC requires control_rate_hz={config.CONTROL_RATE_HZ:g}')
        if steps is not None and steps <= 0:
            raise ValueError('steps must be positive')
        if headless and steps is None:
            raise ValueError('Headless runs require a finite --steps value')
        simulator = SonicUpperBodySimulator()
        if not headless:
            viewer = mujoco.viewer.launch_passive(
                simulator.model, simulator.data,
                show_left_ui=False, show_right_ui=False)
            _configure_camera(viewer, simulator)
            viewer.sync()
        if not mock:
            if robot._auto_start_pc_service:
                robot.start_pc_service()
            robot.initialize()
            calibration_input = _calibration_countdown(robot=robot)
        else:
            calibration_input = None
        robot.reset_ik_reference()
        if calibration_input is not None:
            robot.solve_inverse_kinematics(calibration_input)
        robot.log_event(
            'sonic_simulation_started', headless=headless, mock=mock,
            model_path=str(simulator.model_path))
        print('Experimental SONIC upper-body mode: fixed body, arm policy outputs only.')
        frame = 0
        waiting_since = -float('inf')
        while (steps is None or frame < steps) and (viewer is None or viewer.is_running()):
            started = time.monotonic()
            tracking = mock_tracking_input(frame) if mock else robot.read_tracking_input()
            tracking_read_finished = time.monotonic()
            result = robot.solve_inverse_kinematics(tracking)
            ik_finished = time.monotonic()
            if not headless:
                robot.record_teleoperation_frame(
                    tracking, result, ik_finished - started,
                    sdk_read_seconds=tracking_read_finished - started,
                    ik_seconds=ik_finished - tracking_read_finished)
            if result is not None:
                simulator.apply_ik_result(result)
            elif started - waiting_since > 2.0:
                print('Waiting for two valid Motion Trackers; holding the last reference.')
                waiting_since = started
            if viewer is not None:
                with viewer.lock():
                    simulator.step()
                viewer.sync()
            else:
                simulator.step()
            errors.append(simulator.data.qpos[simulator._arm_q].copy()
                          - simulator._reference_il[simulator._arm_il])
            frame += 1
            if not headless:
                time.sleep(max(0.0, 0.02 - (time.monotonic() - started)))
        report = {
            'mode': 'sonic_fixed_body_upper_arms', 'mock': mock, 'steps': frame,
            'simulation_seconds': float(simulator.data.time),
            'arm_reference_rmse_rad': float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            'arm_reference_max_abs_error_rad': float(np.max(np.abs(errors))) if errors else None,
            'warning_count': int(np.sum(simulator.data.warning.number)),
        }
        if metrics_path is not None:
            metrics_path.parent.mkdir(parents=True, exist_ok=True)
            metrics_path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        return report
    except KeyboardInterrupt:
        robot.log_event('sonic_simulation_interrupted')
        return {}
    except BaseException:
        robot.log_event(
            'sonic_simulation_failed', level=logging.ERROR, exc_info=True)
        raise
    finally:
        robot.log_event('sonic_simulation_stopped')
        if viewer is not None:
            viewer.close()
        if simulator is not None:
            simulator.close()
        robot.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--steps', type=int)
    parser.add_argument('--metrics', type=Path)
    parser.add_argument('--left-tracker-serial')
    parser.add_argument('--right-tracker-serial')
    args = parser.parse_args()
    robot = A3UltraInterface(
        sdk=object() if args.mock else None, auto_start_pc_service=not args.mock,
        left_tracker_serial=args.left_tracker_serial,
        right_tracker_serial=args.right_tracker_serial,
    )
    run_sonic_simulation(robot, mock=args.mock, headless=args.headless,
                         steps=args.steps, metrics_path=args.metrics)


if __name__ == '__main__':
    main()
