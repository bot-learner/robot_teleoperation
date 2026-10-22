"""双臂关节空间防自碰撞规划，直接运行显示交叉、手在身后等姿态的归位演示。

运行：conda activate a3_teleop
      python pico_teleop_interface/robot_interfaces/a3_ultra/a3_ultra_arm_motion_planner.py
仅做运动学路径播放，不连接真机，不模拟 PD 跟踪误差。
返回的路径角度单位为 rad，顺序为左臂 7 关节、右臂 7 关节。
"""

from contextlib import ExitStack
from pathlib import Path
import sys
import threading
import time

import numpy as np
from ompl import base as ob, geometric as og
from scipy.optimize import Bounds, minimize

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from pico_teleop_interface.robot_interfaces.a3_ultra.dual_arm_ik import (
    A3UltraDualArmIK, ARM_JOINT_NAMES, DEFAULT_ARM_JOINT_POSITIONS,
)


# 经模型验证的困难起点；交叉姿态的两只手上下错开，保留 10 cm 安全间距。
# 每个起点直接插值归位均会进入安全边界内，必须搜索绕行路径。
DEMO_START_POSES = {
    "Crossed arms": (
        -1.504873, -0.012344, -1.678680, 0.668692, 0., 0., 0.,
        -1.960164, -0.096399, 1.238098, 0.394635, 0., 0., 0.,
    ),
    "Left hand behind body": (
        2.424163, 0.798447, -2.364081, 1.271452, 0., 0., 0.,
        -2.363021, -0.621254, -2.636611, 0.351073, 0., 0., 0.,
    ),
    "Right hand behind body": (
        0.911066, 2.341957, 1.484633, 0.810593, 0., 0., 0.,
        2.471861, -0.712146, -0.187230, 0.874416, 0., 0., 0.,
    ),
    "Both hands behind body": (
        2.424163, 0.798447, -2.364081, 1.271452, 0., 0., 0.,
        2.471861, -0.712146, -0.187230, 0.874416, 0., 0., 0.,
    ),
}


# 固定的贴身起点，不加随机扰动，以免把近接触变成初始穿透。
# 这些起点先规划逐步脱离身体的路径，达到 10 cm 间距后再正常归位。
DEMO_NEAR_BODY_POSES = {
    "Arms close to torso": (
        0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0.,
    ),
    "Hands close to thighs": (
        0.5, 0.05, 0., 0.8, 0., 0., 0.,
        0.5, -0.05, 0., 0.8, 0., 0., 0.,
    ),
    # 约 0.5 mm 间隙模拟手掌触腿；避免精确零距离的浮点不确定性。
    "Arms hanging, hands almost touching thighs": (
        0.5, 0.04484757, 0., 0.8, 0., 0., 0.,
        0.5, -0.04484757, 0., 0.8, 0., 0., 0.,
    ),
}

# 双臂近似竖直向下，肘关节 1.418 rad 使上臂与前臂共线。
NATURAL_ARM_POSITIONS = (
    0.15, 0.065, 0., 1.41810278, 0., 0., 0.,
    0.15, -0.065, 0., 1.41810278, 0., 0., 0.,
)


class ArmMotionPlanner:
    """贴身起点先脱离，再用 RRT-Connect 在 10 cm 安全间距下归位。

    躯干、头和腿固定在模型初始姿态；实例内部持有独立模型，不与实时 IK 共享状态。
    shortcut: 碰撞按关节空间小步长离散检查，实机接入前需验证模型精度和采样分辨率。
    """

    def __init__(self) -> None:
        self._ik = A3UltraDualArmIK()
        # 普通归位阶段使用 10 cm 安全间距；脱离阶段从初始间距逐步增加到该值。
        self.safety_distance_m = 0.10
        self.lower = self._ik.model.lowerPositionLimit[self._ik._arm_q_indices].copy()
        self.upper = self._ik.model.upperPositionLimit[self._ik._arm_q_indices].copy()
        self.goal = np.array(DEFAULT_ARM_JOINT_POSITIONS)
        # 相邻检查点的 14 维欧氏距离，保证每个关节的变化也不超过该值。
        self.check_step_rad = 0.02

    def _within_limits(self, positions: np.ndarray) -> bool:
        q = np.asarray(positions, dtype=float)
        return (q.shape == (len(ARM_JOINT_NAMES),) and np.isfinite(q).all()
                and np.all(q >= self.lower) and np.all(q <= self.upper))

    def _distances(self, positions: np.ndarray) -> np.ndarray:
        """额外扫描 1 cm，以便在脱离阶段提前发现新的近碰撞对。"""

        configuration = self._ik._q.copy()
        configuration[self._ik._arm_q_indices] = positions
        return self._ik._collision_distances(configuration, self.safety_distance_m + 0.01)

    def is_valid(self, positions: np.ndarray) -> bool:
        """检查普通归位阶段的限位和 10 cm 安全间距。"""

        return (self._within_limits(positions)
                and bool(np.min(self._distances(positions)) >= self.safety_distance_m))

    def _escape(self, start: np.ndarray, deadline: float) -> np.ndarray:
        """允许从模型穿透姿态开始，以总碰撞欠距持续下降的方式逐步脱离。"""

        path = [start.copy()]
        q = start.copy()
        # 单个碰撞对可暂时变差，但最深穿透不能超过真实起点，避免逃逸过程继续钻入身体。
        minimum_start_distance = float(np.min(self._distances(start)))
        for _ in range(200):
            distances = self._distances(q)
            if np.min(distances) >= self.safety_distance_m:
                return np.array(path)
            if time.perf_counter() >= deadline:
                raise RuntimeError("Escape planning timed out; no path returned")
            configuration = self._ik._q.copy()
            configuration[self._ik._arm_q_indices] = q
            active = np.flatnonzero(distances < self.safety_distance_m + 0.01)
            gradients = np.array([
                self._ik._collision_distance_gradient(configuration, int(i))
                for i in active
            ])
            # 每步争取增加最多 4 mm 间距；小幅越过 10 cm，避免无限逼近边界。
            desired = np.clip(self.safety_distance_m + 0.002 - distances[active], 0., 0.004)
            result = minimize(
                lambda v: 0.5 * np.sum((gradients @ v - desired) ** 2)
                + 1e-5 * np.dot(v, v),
                np.zeros(len(ARM_JOINT_NAMES)),
                jac=lambda v: gradients.T @ (gradients @ v - desired) + 2e-5 * v,
                method="SLSQP",
                bounds=Bounds(np.maximum(self.lower - q, -0.04),
                              np.minimum(self.upper - q, 0.04)),
                options={"ftol": 1e-12, "maxiter": 100},
            )
            # SLSQP 在接触边界可能报告收敛精度不足；仅把输出当方向，后面逐点验真。
            if not np.isfinite(result.x).all():
                raise RuntimeError("Escape direction search failed; no path returned")
            # 梯度只用于找方向；必须密集复检真实几何距离，不能直接接受线性预测。
            for scale in (1., 0.5, 0.25, 0.125, 0.0625):
                candidate = q + scale * result.x
                if np.linalg.norm(candidate - q) < 1e-7:
                    continue
                count = max(1, int(np.ceil(np.linalg.norm(candidate - q) / 0.005)))
                segment = np.linspace(q, candidate, count + 1)[1:]
                violation = np.maximum(self.safety_distance_m - distances, 0.)
                previous_debt = float(violation @ violation)
                for positions in segment:
                    actual = self._distances(positions)
                    violation = np.maximum(self.safety_distance_m - actual, 0.)
                    debt = float(violation @ violation)
                    if (not self._within_limits(positions)
                            or np.min(actual) < minimum_start_distance - 1e-8
                            or debt > previous_debt + 1e-10):
                        break
                    previous_debt = debt
                else:
                    current_violation = np.maximum(
                        self.safety_distance_m - distances, 0.,
                    )
                    current_debt = float(current_violation @ current_violation)
                    if previous_debt < current_debt - 1e-10:
                        path.extend(segment)
                        q = candidate
                        break
            else:
                raise RuntimeError("No collision-reducing escape step found; no path returned")
        raise RuntimeError("Escape iteration limit reached; no path returned")

    def plan(
        self, start: np.ndarray, goal: np.ndarray | None = None,
        timeout_seconds: float = 5.0,
    ) -> np.ndarray:
        """贴身或模型穿透起点先降低总体碰撞并脱离至 10 cm，再 RRT 归位。

        路径只包含关节角，没有速度和时间信息，不能当作 100 Hz 轨迹直接发送。
        """

        start = np.asarray(start, dtype=float)
        goal = self.goal.copy() if goal is None else np.asarray(goal, dtype=float)
        if not np.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("Planning timeout must be positive and finite")
        deadline = time.perf_counter() + timeout_seconds
        if not self._within_limits(start):
            raise ValueError("Start violates joint limits or contains invalid values")
        if not self.is_valid(goal):
            raise ValueError("Goal violates joint limits or collision clearance")
        escape = self._escape(start, deadline)
        start = escape[-1]

        space = ob.RealVectorStateSpace(len(ARM_JOINT_NAMES))
        bounds = ob.RealVectorBounds(len(ARM_JOINT_NAMES))
        for i, (lower, upper) in enumerate(zip(self.lower, self.upper)):
            bounds.setLow(i, float(lower))
            bounds.setHigh(i, float(upper))
        space.setBounds(bounds)
        setup = og.SimpleSetup(space)
        setup.setStateValidityChecker(
            lambda state: self.is_valid(np.array([
                state[i] for i in range(len(ARM_JOINT_NAMES))
            ]))
        )
        setup.getSpaceInformation().setStateValidityCheckingResolution(
            self.check_step_rad / space.getMaximumExtent()
        )
        start_state, goal_state = space.allocState(), space.allocState()
        for i in range(len(ARM_JOINT_NAMES)):
            start_state[i], goal_state[i] = float(start[i]), float(goal[i])
        setup.setStartAndGoalStates(start_state, goal_state, 1e-9)
        planner = og.RRTConnect(setup.getSpaceInformation())
        planner.setRange(0.4)
        setup.setPlanner(planner)
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            raise RuntimeError("Planning timed out after escape; no path returned")
        setup.solve(remaining)
        if not setup.haveExactSolutionPath():
            raise RuntimeError("No exact collision-free path found within the time limit")

        # 简化也通过 OMPL 的路径段碰撞检查，不能只连接安全的端点。
        setup.simplifySolution(0.1)
        waypoints = np.array([
            [state[i] for i in range(len(ARM_JOINT_NAMES))]
            for state in setup.getSolutionPath().getStates()
        ])
        waypoints[0], waypoints[-1] = start, goal
        dense = [start.copy()]
        for first, second in zip(waypoints[:-1], waypoints[1:]):
            count = max(1, int(np.ceil(np.linalg.norm(second - first)
                                       / self.check_step_rad)))
            dense.extend(np.linspace(first, second, count + 1)[1:])
        path = np.array(dense)
        if not all(self.is_valid(q) for q in path):
            raise RuntimeError("Final path collision check failed; path rejected")
        # 脱离段不能参与普通路径简化，否则可能切掉已验证的避障动作。
        return np.vstack((escape[:-1], path))

    def plan_close_approach(
        self, start: np.ndarray, goal: np.ndarray, timeout_seconds: float = 5.0,
    ) -> np.ndarray:
        """先在 10 cm 安全区规划，再沿无穿透路径接近身体。"""

        start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        if not self._within_limits(goal) or np.min(self._distances(goal)) <= 0.:
            raise ValueError("Close-approach goal violates joint limits or penetrates geometry")
        deadline = time.perf_counter() + timeout_seconds
        # 从近身终点向外脱离，倒放该路径就是最后的安全接近段。
        approach_reversed = self._escape(goal, deadline)
        safe_path = self.plan(
            start, approach_reversed[-1],
            timeout_seconds=max(0.1, deadline - time.perf_counter()),
        )
        path = np.vstack((safe_path, approach_reversed[-2::-1]))
        if not np.allclose(path[0], start) or not np.allclose(path[-1], goal):
            raise RuntimeError("Close-approach path endpoints are incorrect")
        if min(float(np.min(self._distances(q))) for q in path) <= 0.:
            raise RuntimeError("Close-approach path penetrates geometry")
        return path


def _sample_demo_start(
    planner: ArmMotionPlanner, nominal: tuple[float, ...], rng: np.random.Generator,
) -> np.ndarray:
    """在指定困难姿态附近随机化，并确保直接归位存在障碍，避免退化为简单演示。"""

    for _ in range(100):
        start = np.asarray(nominal) + rng.uniform(-0.01, 0.01, len(ARM_JOINT_NAMES))
        if (planner.is_valid(start)
                and not all(planner.is_valid(q)
                            for q in np.linspace(start, planner.goal, 80))):
            return start
    raise RuntimeError("Could not sample a safe challenging start; check the collision model")


def main() -> None:
    """演示贴身/触腿脱离，以及交叉、身后姿态的避障归位。"""

    import mujoco
    import mujoco.viewer
    from pico_teleop_interface.robot_interfaces.a3_ultra.sim import (
        A3UltraSimulator, _configure_camera,
    )

    planner = ArmMotionPlanner()
    rng = np.random.default_rng()
    if not planner.is_valid(planner.goal):
        raise RuntimeError("Default arm pose is not collision-free in the current model")
    simulator = A3UltraSimulator()
    # 显示模型的腰、头、腿等关节与规划模型保持一致，避免 keyframe 造成姿态差异。
    for name in planner._ik.model.names[1:]:
        pin_joint = planner._ik.model.joints[planner._ik.model.getJointId(name)]
        mj_joint_id = mujoco.mj_name2id(simulator.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if pin_joint.nq == 1 and mj_joint_id >= 0:
            address = simulator.model.jnt_qposadr[mj_joint_id]
            simulator.data.qpos[address] = planner._ik._q[pin_joint.idx_q]
    simulator._place_feet_on_ground()
    print("MuJoCo kinematic preview only; no robot commands will be sent.")
    print("Goal (rad):", planner.goal)
    successes = 0
    rejected = 0
    threads_before = set(threading.enumerate())
    with ExitStack() as stack:
        viewer = mujoco.viewer.launch_passive(
            simulator.model, simulator.data, show_left_ui=False, show_right_ui=False,
        )
        # MuJoCo 的 close 只发退出请求；等渲染线程释放资源，避免解释器退出时崩溃。
        for thread in set(threading.enumerate()) - threads_before:
            stack.callback(thread.join, 5.0)
        stack.enter_context(viewer)
        _configure_camera(viewer, simulator)

        def show(positions: np.ndarray) -> bool:
            if not viewer.is_running():
                return False
            with viewer.lock():
                simulator.data.qpos[simulator._arm_q] = positions
                mujoco.mj_forward(simulator.model, simulator.data)
            viewer.sync()
            return True

        cases = {**DEMO_NEAR_BODY_POSES, **DEMO_START_POSES}
        total = len(cases)
        for trial, (label, nominal) in enumerate(cases.items(), 1):
            if not viewer.is_running():
                return
            near_body = label in DEMO_NEAR_BODY_POSES
            start = (np.array(nominal) if near_body
                     else _sample_demo_start(planner, nominal, rng))
            print(f"\n[{trial}/{total}] {label}: {np.round(start, 4)}", flush=True)
            if near_body:
                ik = planner._ik
                configuration = ik._q.copy()
                configuration[ik._arm_q_indices] = start
                distances = ik._collision_distances(configuration, planner.safety_distance_m)
                index = int(np.argmin(distances))
                pair = ik._collision_model.collisionPairs[index]
                names = [ik._collision_model.geometryObjects[i].name
                         for i in (pair.first, pair.second)]
                print(f"Clearance: {distances[index] * 1000:.2f} mm; "
                      f"target: {planner.safety_distance_m * 1000:.0f} mm; "
                      f"pair: {names[0]} / {names[1]}", flush=True)
                print("Planning gradual escape to 100 mm, then collision-free return.", flush=True)
            else:
                print("Direct interpolation is blocked; searching for a detour.", flush=True)
            # 贴身起点停留五秒，其他起点停留两秒，持续处理窗口事件。
            for _ in range(250 if near_body else 100):
                if not show(start):
                    return
                time.sleep(0.02)
            started = time.perf_counter()
            try:
                path = planner.plan(start, timeout_seconds=10.)
            except ValueError as exc:
                rejected += 1
                print(f"REJECTED: {exc}; no motion will be played", flush=True)
                continue
            except RuntimeError as exc:
                print(f"FAIL: {exc}; no motion will be played", flush=True)
                continue
            assert np.allclose(path[0], start) and np.allclose(path[-1], planner.goal)
            print(f"PASS: {len(path)} checked points, "
                  f"{time.perf_counter() - started:.3f} s; playing path", flush=True)
            # 50 Hz 运动学播放，关节速度不超过 0.5 rad/s；保持原路径的线性插值。
            for first, second in zip(path[:-1], path[1:]):
                frames = max(1, int(np.ceil(np.max(np.abs(second - first)) / (0.5 * 0.02))))
                for positions in np.linspace(first, second, frames + 1)[1:]:
                    tick = time.perf_counter()
                    if not show(positions):
                        return
                    time.sleep(max(0., 0.02 - (time.perf_counter() - tick)))
            successes += 1
            print(f"[{trial}/{total}] Reached default pose", flush=True)
            # 目标处停留一秒，并持续处理窗口事件。
            for _ in range(50):
                if not show(planner.goal):
                    return
                time.sleep(0.02)
        print(f"\nResult: {successes}/{total} reached the goal; "
              f"{rejected} invalid starts rejected. Close the window to exit.", flush=True)
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
