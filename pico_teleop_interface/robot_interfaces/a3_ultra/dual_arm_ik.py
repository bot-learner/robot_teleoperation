"""A3 Ultra 双臂 Pinocchio 逆运动学模型。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
import pinocchio as pin

from . import config


# 顺序固定为“左臂 7 关节 + 右臂 7 关节”，必须与 MuJoCo 和 SONIC 一致。
ARM_JOINT_NAMES = tuple(
    f"{side}_{joint}_joint" for side in ("left", "right") for joint in (
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
        "wrist_roll", "wrist_pitch", "wrist_yaw",
    )
)

# 双臂向前、肘部微弯，为 IK 保留活动余量并确定肘部弯曲分支。
DEFAULT_ARM_JOINT_POSITIONS = (
    -1., .1, 0., .5, 0., 0., 0., -1., -.1, 0., .5, 0., 0., 0.,
)

PALM_FRAME_NAMES = ("left_palm_Link", "right_palm_Link")
# 手臂可达的固定身体区域；膝和踝以下不进入上半身遥操作碰撞集合。
ARM_COLLISION_BODY_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "waist_pitch_joint", "head_yaw_joint", "ultra_head_pitch_joint",
)
DEFAULT_MODEL_PATH = (Path(__file__).resolve().parent / "models"
                      / "A3-A3U-robot-model" / "a3_ultra_t3d0"
                      / "urdf" / "model_planning.urdf")


@dataclass(frozen=True)
class A3UltraIKResult:
    """一次 IK 输出；关节名称与关节角数组按同一索引对应。"""

    joint_names: tuple[str, ...]
    joint_positions: np.ndarray
    status: str = "unknown"
    iterations: int = 0


class A3UltraDualArmIK:
    """维护 A3 Ultra 模型状态并求解左右手掌的联合 IK。"""

    def __init__(
        self,
        model_path: str | Path | None = None,
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
    ) -> None:
        """加载 URDF，并配置收敛、姿态偏好、关节限位与碰撞参数。"""

        if min(max_iterations, max_solve_time_ms, position_tolerance,
               rotation_tolerance_deg, damping, integration_step,
               max_joint_step) <= 0:
            raise ValueError("IK 数值参数必须大于 0")
        if min(elbow_preference_weight, self_collision_weight,
               joint_limit_weight) < 0:
            raise ValueError("次要 IK 代价权重不能为负数")
        if not 0 < collision_min_distance < collision_activation_distance:
            raise ValueError("自碰撞硬边界必须大于 0 且小于激活距离")
        if not (collision_min_distance < collision_danger_distance
                <= collision_active_enter_distance
                < collision_activation_distance
                <= collision_active_exit_distance):
            raise ValueError("碰撞距离阈值顺序不正确")
        if collision_gradient_refresh_interval < 1:
            raise ValueError("碰撞梯度刷新间隔必须大于 0")
        if not 0 <= collision_warning_weight_scale <= 1:
            raise ValueError("碰撞预警区权重比例必须在 0 到 1 之间")
        if collision_broadphase_margin < 0:
            raise ValueError("碰撞预检余量不能小于 0")
        if not 0 < workspace_min_radius < workspace_max_radius:
            raise ValueError("IK 工作空间半径顺序不正确")

        self.model_path = Path(model_path or DEFAULT_MODEL_PATH).resolve()
        self.model = pin.buildModelFromUrdf(str(self.model_path))
        self.data = self.model.createData()
        self.max_iterations = max_iterations
        self.max_solve_time_seconds = max_solve_time_ms / 1000.
        self.position_tolerance = position_tolerance
        self.rotation_tolerance = np.deg2rad(rotation_tolerance_deg)
        self.damping = damping
        self.integration_step = integration_step
        self.max_joint_step = max_joint_step
        self.elbow_preference_weight = elbow_preference_weight
        self.self_collision_weight = self_collision_weight
        self.joint_limit_weight = joint_limit_weight
        self.collision_activation_distance = collision_activation_distance
        self.collision_min_distance = collision_min_distance
        self.collision_danger_distance = collision_danger_distance
        self.collision_active_enter_distance = collision_active_enter_distance
        self.collision_active_exit_distance = collision_active_exit_distance
        self.collision_gradient_refresh_interval = collision_gradient_refresh_interval
        self.collision_warning_weight_scale = collision_warning_weight_scale
        self.collision_broadphase_margin = collision_broadphase_margin
        self.workspace_min_radius = workspace_min_radius
        self.workspace_max_radius = workspace_max_radius

        self._arm_q_indices, self._arm_v_indices = self._resolve_arm_indices()
        self._shoulder_joint_ids = tuple(
            self.model.getJointId(f"{side}_shoulder_pitch_joint")
            for side in ("left", "right"))
        self._palm_frame_ids = tuple(self.model.getFrameId(n) for n in PALM_FRAME_NAMES)
        if any(i >= len(self.model.frames) for i in self._palm_frame_ids):
            raise ValueError("URDF 中缺少左右手掌 Frame")
        self._q = pin.neutral(self.model)
        self._q[self._arm_q_indices] = DEFAULT_ARM_JOINT_POSITIONS
        self._nominal_arm_q = np.asarray(DEFAULT_ARM_JOINT_POSITIONS)
        self._collision_model = self._build_collision_model()
        self._collision_data = pin.GeometryData(self._collision_model)
        self._build_collision_broadphase()
        self._active_collision_pairs = np.zeros(
            len(self._collision_model.collisionPairs), dtype=bool)
        self._cached_collision_gradient = np.zeros(len(self._arm_q_indices))
        self._cached_collision_distance_gradient = np.zeros(
            len(self._arm_q_indices))
        self._cached_collision_distance = float("inf")
        # 软避碰和硬边界使用不同粗筛距离，缓存必须分别保存。
        self._collision_distances_cache: dict[
            float, tuple[np.ndarray, np.ndarray]
        ] = {}
        self._collision_update_count = 0
        self._fallback_error_reference: tuple[np.ndarray, np.ndarray] | None = None
        self._fallback_target_reference: tuple[pin.SE3, pin.SE3] | None = None

    def _build_collision_broadphase(self) -> None:
        """为每个精确碰撞体建立能完整覆盖它的局部包围球。"""

        centers, radii = [], []
        for geometry_object in self._collision_model.geometryObjects:
            geometry_object.geometry.computeLocalAABB()
            lower = np.asarray(geometry_object.geometry.aabb_local.min_)
            upper = np.asarray(geometry_object.geometry.aabb_local.max_)
            center = (lower + upper) * .5
            centers.append(center)
            radii.append(np.linalg.norm(upper - center))
        self._broadphase_centers = np.asarray(centers)
        self._broadphase_radii = np.asarray(radii)
        self._collision_pair_first = np.fromiter(
            (pair.first for pair in self._collision_model.collisionPairs), int)
        self._collision_pair_second = np.fromiter(
            (pair.second for pair in self._collision_model.collisionPairs), int)

    def palm_poses(self) -> tuple[pin.SE3, pin.SE3]:
        """返回当前关节配置下左右掌根在机器人基座坐标系中的位姿。"""

        pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateFramePlacements(self.model, self.data)
        return tuple(pin.SE3(self.data.oMf[i]) for i in self._palm_frame_ids)

    def _build_collision_model(self) -> pin.GeometryModel:
        """只保留涉及手臂且不是相邻连杆的碰撞对。"""

        collision_model = pin.buildGeomFromUrdf(
            self.model, str(self.model_path), pin.GeometryType.COLLISION)
        collision_model.addAllCollisionPairs()
        srdf_path = self.model_path.with_suffix(".srdf")
        if srdf_path.is_file():
            pin.removeCollisionPairs(self.model, collision_model, str(srdf_path), False)

        arm_joint_ids = {self.model.getJointId(name) for name in ARM_JOINT_NAMES}
        # 0 是固定在 world 上的 pelvis；其余是躯干、头部和大腿相关关节。
        body_joint_ids = {0} | {
            self.model.getJointId(name) for name in ARM_COLLISION_BODY_JOINTS
        }
        relevant_joint_ids = arm_joint_ids | body_joint_ids

        def ancestors(joint_id: int) -> list[int]:
            result = []
            while joint_id:
                result.append(joint_id)
                joint_id = int(self.model.parents[joint_id])
            return result

        joint_ancestors = [ancestors(i) for i in range(len(self.model.joints))]

        def tree_distance(first: int, second: int) -> int:
            first_steps = {joint: step for step, joint in enumerate(joint_ancestors[first])}
            return min(
                (first_steps[joint] + step
                 for step, joint in enumerate(joint_ancestors[second])
                 if joint in first_steps),
                default=len(self.model.joints),
            )

        collision_pairs = []
        for pair in collision_model.collisionPairs:
            first = collision_model.geometryObjects[pair.first].parentJoint
            second = collision_model.geometryObjects[pair.second].parentJoint
            # 相邻连杆天然贴近，不能当作需要排斥的自碰撞。
            if (first in relevant_joint_ids and second in relevant_joint_ids
                    and (first in arm_joint_ids or second in arm_joint_ids)
                    and tree_distance(first, second) > 2):
                collision_pairs.append(pair)

        collision_model.removeAllCollisionPairs()
        for pair in collision_pairs:
            collision_model.addCollisionPair(pair)
        if not collision_model.collisionPairs:
            raise ValueError("URDF 中未找到双臂自碰撞对")
        return collision_model

    def _resolve_arm_indices(self) -> tuple[np.ndarray, np.ndarray]:
        """将关节名称转换为 Pinocchio 配置和速度向量索引。"""

        q_indices, v_indices = [], []
        for name in ARM_JOINT_NAMES:
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                raise ValueError(f"URDF 中缺少关节：{name}")
            joint = self.model.joints[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f"关节 {name} 不是单自由度关节")
            q_indices.append(joint.idx_q)
            v_indices.append(joint.idx_v)
        return np.asarray(q_indices), np.asarray(v_indices)

    def _stacked_task(
        self, targets: tuple[pin.SE3, pin.SE3], *, with_jacobian: bool = True,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """构造双掌误差；仅在迭代更新需要时计算共享关节雅可比。"""

        if with_jacobian:
            pin.computeJointJacobians(self.model, self.data, self._q)
        else:
            pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateFramePlacements(self.model, self.data)
        errors, jacobians = [], []
        for frame_id, target in zip(self._palm_frame_ids, targets):
            current_to_target = self.data.oMf[frame_id].actInv(target)
            error = pin.log6(current_to_target).vector
            errors.append(error)
            if with_jacobian:
                jacobian = pin.getFrameJacobian(
                    self.model, self.data, frame_id, pin.ReferenceFrame.LOCAL)
                jacobian = -pin.Jlog6(current_to_target.inverse()) @ jacobian
                jacobians.append(jacobian[:, self._arm_v_indices])
        return (np.concatenate(errors),
                np.vstack(jacobians) if with_jacobian else None)

    def _project_targets(
        self, targets: tuple[pin.SE3, pin.SE3],
    ) -> tuple[pin.SE3, pin.SE3]:
        """把掌根目标投影到对应肩关节的可达球壳内。"""

        pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateFramePlacements(self.model, self.data)
        projected = []
        for target, shoulder_id, palm_id in zip(
                targets, self._shoulder_joint_ids, self._palm_frame_ids):
            shoulder = self.data.oMi[shoulder_id].translation
            offset = target.translation - shoulder
            distance = np.linalg.norm(offset)
            if distance < 1e-12:
                direction = self.data.oMf[palm_id].translation - shoulder
                direction /= np.linalg.norm(direction)
                radius = self.workspace_min_radius
            else:
                direction = offset / distance
                radius = np.clip(
                    distance, self.workspace_min_radius,
                    self.workspace_max_radius)
            projected.append(pin.SE3(
                target.rotation, shoulder + direction * radius))
        return tuple(projected)

    def _collision_distances(
        self, configuration: np.ndarray, distance_threshold: float,
    ) -> np.ndarray:
        """包围球预检后，只计算可能进入指定距离内的精确距离。"""

        cached = self._collision_distances_cache.get(distance_threshold)
        if cached is not None and np.array_equal(configuration, cached[0]):
            return cached[1]
        pin.updateGeometryPlacements(
            self.model, self.data, self._collision_model,
            self._collision_data, configuration)
        world_centers = np.asarray([
            placement.act(center) for placement, center in zip(
                self._collision_data.oMg, self._broadphase_centers)])
        first, second = self._collision_pair_first, self._collision_pair_second
        lower_bounds = (
            np.linalg.norm(world_centers[first] - world_centers[second], axis=1)
            - self._broadphase_radii[first] - self._broadphase_radii[second])
        candidates = np.flatnonzero(
            lower_bounds < distance_threshold + self.collision_broadphase_margin)
        distances = np.full(len(first), np.inf)
        for index in candidates:
            distances[index] = pin.computeDistance(
                self._collision_model, self._collision_data,
                int(index)).min_distance
        self._collision_distances_cache[distance_threshold] = (
            configuration.copy(), distances)
        return distances

    def _minimum_collision_distance(
        self, configuration: np.ndarray,
    ) -> tuple[float, int]:
        """用硬边界粗筛返回指定配置下可能危及安全的最近碰撞对。"""

        distances = self._collision_distances(
            configuration, self.collision_min_distance)
        pair_index = int(np.argmin(distances))
        return float(distances[pair_index]), pair_index

    def _active_collision_distance(
        self, configuration: np.ndarray
    ) -> tuple[float, int]:
        """只更新活跃集合，并返回其中最近的碰撞对。"""

        indices = np.flatnonzero(self._active_collision_pairs)
        pin.updateGeometryPlacements(
            self.model, self.data, self._collision_model,
            self._collision_data, configuration)
        distances = np.fromiter(
            (pin.computeDistance(
                self._collision_model, self._collision_data, int(index)).min_distance
             for index in indices), dtype=float, count=len(indices))
        self._active_collision_pairs[indices[
            distances >= self.collision_active_exit_distance]] = False
        local_index = int(np.argmin(distances))
        return float(distances[local_index]), int(indices[local_index])

    def _collision_distance_gradient(
        self, configuration: np.ndarray, pair_index: int,
    ) -> np.ndarray:
        """由最近点法向量和连杆雅可比计算指定碰撞对的距离梯度。"""

        # 缓存只保存距离值；最近点必须与当前关节配置保持一致。
        pin.updateGeometryPlacements(
            self.model, self.data, self._collision_model,
            self._collision_data, configuration)
        pin.computeDistance(
            self._collision_model, self._collision_data, pair_index)
        pair = self._collision_model.collisionPairs[pair_index]
        first = self._collision_model.geometryObjects[pair.first]
        second = self._collision_model.geometryObjects[pair.second]
        result = self._collision_data.distanceResults[pair_index]
        first_point = np.asarray(result.getNearestPoint1())
        second_point = np.asarray(result.getNearestPoint2())
        normal = np.asarray(result.normal)
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-12:
            return np.zeros(len(self._arm_q_indices))
        normal /= normal_norm

        def point_jacobian(joint_id: int, point: np.ndarray) -> np.ndarray:
            if joint_id == 0:
                return np.zeros((3, self.model.nv))
            jacobian = pin.getJointJacobian(
                self.model, self.data, joint_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            offset = point - self.data.oMi[joint_id].translation
            return jacobian[:3] - pin.skew(offset) @ jacobian[3:]

        pin.computeJointJacobians(self.model, self.data, configuration)
        first_jacobian = point_jacobian(first.parentJoint, first_point)
        second_jacobian = point_jacobian(second.parentJoint, second_point)
        return (normal @ (second_jacobian - first_jacobian))[self._arm_v_indices]

    def _collision_gradient(
        self, configuration: np.ndarray,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """按距离分级更新最近活跃碰撞对的解析梯度。"""

        refresh = (self._collision_update_count
                   % self.collision_gradient_refresh_interval == 0)
        self._collision_update_count += 1
        if refresh:
            distances = self._collision_distances(
                configuration, self.collision_activation_distance)
            pair_index = int(np.argmin(distances))
            distance = float(distances[pair_index])
            self._active_collision_pairs = (
                (distances < self.collision_active_enter_distance)
                | (self._active_collision_pairs
                   & (distances < self.collision_active_exit_distance)))
        elif self._active_collision_pairs.any():
            distance, pair_index = self._active_collision_distance(configuration)
        else:
            return (self._cached_collision_gradient,
                    self._cached_collision_distance,
                    self._cached_collision_distance_gradient)

        self._cached_collision_distance = distance

        if (not self.self_collision_weight
                or distance >= self.collision_active_exit_distance):
            self._cached_collision_gradient.fill(0.)
            self._cached_collision_distance_gradient.fill(0.)
            return (self._cached_collision_gradient, distance,
                    self._cached_collision_distance_gradient)

        if distance >= self.collision_danger_distance and not refresh:
            return (self._cached_collision_gradient, distance,
                    self._cached_collision_distance_gradient)
        if distance >= self.collision_activation_distance:
            self._cached_collision_gradient.fill(0.)
            self._cached_collision_distance_gradient.fill(0.)
            return (self._cached_collision_gradient, distance,
                    self._cached_collision_distance_gradient)

        distance_gradient = self._collision_distance_gradient(
            configuration, pair_index)
        self._cached_collision_distance_gradient = distance_gradient
        warning_offset = (self.collision_warning_weight_scale
                          * (self.collision_activation_distance
                             - self.collision_danger_distance))
        clearance_error = (
            self.collision_danger_distance - distance + warning_offset
            if distance < self.collision_danger_distance
            else self.collision_warning_weight_scale
            * (self.collision_activation_distance - distance))
        self._cached_collision_gradient = (
            -self.self_collision_weight * clearance_error * distance_gradient)
        return (self._cached_collision_gradient, distance,
                self._cached_collision_distance_gradient)

    def _secondary_gradient(
        self, collision_gradient: np.ndarray, previous_arm_q: np.ndarray,
    ) -> np.ndarray:
        """组合默认姿态、关节限位、自碰撞和上一姿态四类次要代价。"""

        arm_q = self._q[self._arm_q_indices]
        lower = self.model.lowerPositionLimit[self._arm_q_indices]
        upper = self.model.upperPositionLimit[self._arm_q_indices]
        margin = .05 * (upper - lower)
        limit_gradient = self.joint_limit_weight * (
            np.maximum(margin - (upper - arm_q), 0.)
            - np.maximum(margin - (arm_q - lower), 0.)
        ) / np.square(margin)
        return (self.elbow_preference_weight * (arm_q - self._nominal_arm_q)
                + limit_gradient + collision_gradient
                + config.IK_CONTINUITY_WEIGHT * (arm_q - previous_arm_q))

    def solve(self, targets: tuple[pin.SE3, pin.SE3]) -> A3UltraIKResult:
        """从两个机器人基座坐标系手掌目标求解 14 个双臂关节角。"""

        # 为当前 Pinocchio 原子计算和结果收尾预留 1 ms，避免墙钟时间越过配置上限。
        deadline = (time.perf_counter()
                    + max(0., self.max_solve_time_seconds - .001))
        targets = self._project_targets(targets)
        if (self._fallback_target_reference is not None
                and any(
                    np.linalg.norm(target.translation - reference.translation)
                    > self.position_tolerance
                    or np.linalg.norm(pin.log3(
                        reference.rotation.T @ target.rotation))
                    > self.rotation_tolerance
                    for target, reference in zip(
                        targets, self._fallback_target_reference))):
            self._fallback_error_reference = None
            self._fallback_target_reference = None
        frame_start_q = self._q.copy()
        initial_distance, initial_pair = self._minimum_collision_distance(
            frame_start_q)
        if initial_distance < self.collision_active_enter_distance:
            self._active_collision_pairs[initial_pair] = True
        best_q = (frame_start_q.copy()
                  if initial_distance >= self.collision_min_distance else None)
        best_error = float("inf")
        iterations = 0
        converged = used_backtrack = used_tangent_fallback = held = timed_out = False

        for iteration in range(self.max_iterations):
            if time.perf_counter() >= deadline:
                timed_out = True
                break
            iterations = iteration + 1
            error, jacobian = self._stacked_task(targets)
            residual_norm = float(np.linalg.norm(error))
            if (initial_distance >= self.collision_min_distance
                    and residual_norm < best_error):
                best_q, best_error = self._q.copy(), residual_norm
            palm_errors = error.reshape(2, 6)
            if (np.max(np.linalg.norm(palm_errors[:, :3], axis=1))
                    <= self.position_tolerance
                    and np.max(np.linalg.norm(palm_errors[:, 3:], axis=1))
                    <= self.rotation_tolerance):
                converged = True
                self._fallback_error_reference = None
                self._fallback_target_reference = None
                break

            smallest = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
            damping = self.damping + 1e-3 * max(0., 1. - smallest / .05) ** 2
            pseudoinverse = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(12), np.eye(12))
            velocity = -pseudoinverse @ error
            nullspace = np.eye(len(self._arm_v_indices)) - pseudoinverse @ jacobian
            (collision_gradient, collision_distance,
             distance_gradient) = self._collision_gradient(self._q)
            secondary = -nullspace @ self._secondary_gradient(
                collision_gradient, frame_start_q[self._arm_q_indices])
            secondary_norm = np.linalg.norm(secondary)
            if secondary_norm > .5:
                secondary *= .5 / secondary_norm
            joint_delta = np.clip(
                (velocity + secondary) * self.integration_step,
                -self.max_joint_step, self.max_joint_step)
            full_delta = np.zeros(self.model.nv)
            full_delta[self._arm_v_indices] = joint_delta
            iteration_start_q = self._q.copy()

            # 逐次减半，拒绝误差增大或继续侵入碰撞边界的候选步长。
            accepted = False
            for backtrack in range(6):
                if time.perf_counter() >= deadline:
                    timed_out = True
                    break
                self._q = pin.integrate(
                    self.model, iteration_start_q, full_delta * (.5 ** backtrack))
                self._q[self._arm_q_indices] = np.clip(
                    self._q[self._arm_q_indices],
                    self.model.lowerPositionLimit[self._arm_q_indices],
                    self.model.upperPositionLimit[self._arm_q_indices])
                candidate_error, _ = self._stacked_task(
                    targets, with_jacobian=False)
                candidate_norm = float(np.linalg.norm(candidate_error))
                if candidate_norm >= residual_norm:
                    continue
                candidate_distance, candidate_pair = (
                    self._minimum_collision_distance(self._q))
                distance_floor = min(self.collision_min_distance, collision_distance)
                if candidate_distance >= distance_floor - 1e-9:
                    if candidate_distance < self.collision_active_enter_distance:
                        self._active_collision_pairs[candidate_pair] = True
                    if (candidate_distance >= self.collision_min_distance
                            and candidate_norm < best_error):
                        best_q, best_error = self._q.copy(), candidate_norm
                    used_backtrack |= backtrack > 0
                    self._fallback_error_reference = None
                    self._fallback_target_reference = None
                    accepted = True
                    break
            if accepted:
                continue
            if timed_out:
                self._q = iteration_start_q
                break

            # 正常方向被碰撞面挡住时，沿该碰撞面的切向寻找受限的安全绕行步。
            gradient_norm_squared = float(distance_gradient @ distance_gradient)
            fallback_best = None
            if (collision_distance <= self.collision_danger_distance
                    and gradient_norm_squared > 1e-12):
                tangent_delta = joint_delta.copy()
                inward = float(distance_gradient @ tangent_delta)
                if inward < 0.:
                    tangent_delta -= (
                        inward / gradient_norm_squared * distance_gradient)
                tangent_projector = (
                    np.eye(len(self._arm_v_indices))
                    - np.outer(distance_gradient, distance_gradient)
                    / gradient_norm_squared)
                # 在碰撞切平面内补回原步长的双掌旋转效果，避免绕行耗尽姿态误差预算。
                rotation_jacobian = np.vstack((jacobian[3:6], jacobian[9:12]))
                tangent_rotation_jacobian = rotation_jacobian @ tangent_projector
                tangent_delta += tangent_projector @ (
                    np.linalg.pinv(tangent_rotation_jacobian, rcond=1e-4)
                    @ (rotation_jacobian @ (joint_delta - tangent_delta)))
                clearance_needed = max(
                    0., self.collision_min_distance - collision_distance)
                tangent_delta += (
                    clearance_needed / gradient_norm_squared * distance_gradient)
                tangent_delta = np.clip(
                    tangent_delta, -self.max_joint_step, self.max_joint_step)
                tangent_full_delta = np.zeros(self.model.nv)
                tangent_full_delta[self._arm_v_indices] = tangent_delta
                current_position_errors = np.linalg.norm(
                    palm_errors[:, :3], axis=1)
                current_rotation_errors = np.linalg.norm(
                    palm_errors[:, 3:], axis=1)
                if self._fallback_error_reference is None:
                    self._fallback_error_reference = (
                        current_position_errors.copy(),
                        current_rotation_errors.copy())
                    self._fallback_target_reference = tuple(
                        pin.SE3(target) for target in targets)
                reference_position_errors, reference_rotation_errors = (
                    self._fallback_error_reference)

                for backtrack in range(10):
                    if time.perf_counter() >= deadline:
                        timed_out = True
                        break
                    candidate_q = pin.integrate(
                        self.model, iteration_start_q,
                        tangent_full_delta * (.5 ** backtrack))
                    candidate_q[self._arm_q_indices] = np.clip(
                        candidate_q[self._arm_q_indices],
                        self.model.lowerPositionLimit[self._arm_q_indices],
                        self.model.upperPositionLimit[self._arm_q_indices])
                    if np.array_equal(candidate_q, iteration_start_q):
                        continue
                    self._q = candidate_q
                    candidate_error, _ = self._stacked_task(
                        targets, with_jacobian=False)
                    candidate_palm_errors = candidate_error.reshape(2, 6)
                    if (np.any(np.linalg.norm(
                            candidate_palm_errors[:, :3], axis=1)
                            > reference_position_errors
                            + self.position_tolerance)
                            or np.any(np.linalg.norm(
                                candidate_palm_errors[:, 3:], axis=1)
                                > reference_rotation_errors
                                + self.rotation_tolerance)):
                        continue
                    candidate_norm = float(np.linalg.norm(candidate_error))
                    candidate_distance, candidate_pair = (
                        self._minimum_collision_distance(candidate_q))
                    distance_floor = min(
                        self.collision_min_distance, collision_distance)
                    if candidate_distance < distance_floor - 1e-9:
                        continue
                    score = (candidate_distance, -candidate_norm)
                    if fallback_best is None or score > fallback_best[0]:
                        fallback_best = (
                            score, candidate_q.copy(),
                            candidate_distance, candidate_pair)

            if fallback_best is not None:
                _, self._q, candidate_distance, candidate_pair = fallback_best
                best_q = self._q.copy()
                if candidate_distance < self.collision_active_enter_distance:
                    self._active_collision_pairs[candidate_pair] = True
                used_tangent_fallback = True
                break
            if timed_out:
                self._q = iteration_start_q
                break

            self._q = iteration_start_q
            held = True
            break

        if best_q is not None:
            self._q = best_q
        status = ("held" if held else
                  "tangent_fallback" if used_tangent_fallback else
                  "time_limit" if timed_out else
                  "normal_backtrack" if used_backtrack else
                  "converged" if converged else "max_iterations")
        return A3UltraIKResult(
            ARM_JOINT_NAMES, self._q[self._arm_q_indices].copy(),
            status, iterations)
