"""遥操作实时信号滤波器。"""

from __future__ import annotations

from typing import Any

import numpy as np


class SecondOrderButterworthLowPassFilter:
    """可处理标量或固定形状数组的二阶巴特沃斯低通滤波器。"""

    def __init__(self, sample_rate_hz: float, cutoff_hz: float) -> None:
        if sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz 必须大于 0")
        if not 0 < cutoff_hz < sample_rate_hz / 2:
            raise ValueError("cutoff_hz 必须大于 0 且小于奈奎斯特频率")

        k = np.tan(np.pi * cutoff_hz / sample_rate_hz)
        norm = 1.0 / (1.0 + np.sqrt(2.0) * k + k * k)
        self._b0 = k * k * norm
        self._b1 = 2.0 * self._b0
        self._b2 = self._b0
        self._a1 = 2.0 * (k * k - 1.0) * norm
        self._a2 = (1.0 - np.sqrt(2.0) * k + k * k) * norm
        self.reset()

    def reset(self) -> None:
        """清除历史状态；下一次输入将直接作为滤波初值。"""

        self._x1: np.ndarray | None = None
        self._x2: np.ndarray | None = None
        self._y1: np.ndarray | None = None
        self._y2: np.ndarray | None = None

    def update(self, values: Any) -> np.ndarray:
        """输入一个采样点并返回对应的低通滤波结果。"""

        current = np.asarray(values, dtype=float)
        if not np.all(np.isfinite(current)):
            raise ValueError("低通滤波器输入包含非有限值")

        if self._x1 is None:
            self._x1 = current.copy()
            self._x2 = current.copy()
            self._y1 = current.copy()
            self._y2 = current.copy()
            return current.copy()
        if current.shape != self._x1.shape:
            raise ValueError(
                "低通滤波器输入形状不能在运行过程中改变"
            )

        filtered = (
            self._b0 * current
            + self._b1 * self._x1
            + self._b2 * self._x2
            - self._a1 * self._y1
            - self._a2 * self._y2
        )
        self._x2, self._x1 = self._x1, current.copy()
        self._y2, self._y1 = self._y1, filtered.copy()
        return filtered


def _rotation_matrix_to_vector(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    quaternion = np.empty(4)
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion[:] = (
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        first, second = ((1, 2), (2, 0), (0, 1))[axis]
        scale = 2.0 * np.sqrt(
            max(1.0 + rotation[axis, axis] - rotation[first, first]
                - rotation[second, second], 0.0)
        )
        quaternion[axis + 1] = 0.25 * scale
        quaternion[0] = (rotation[second, first] - rotation[first, second]) / scale
        quaternion[first + 1] = (
            rotation[first, axis] + rotation[axis, first]
        ) / scale
        quaternion[second + 1] = (
            rotation[second, axis] + rotation[axis, second]
        ) / scale

    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    vector_norm = np.linalg.norm(quaternion[1:])
    if vector_norm < 1e-12:
        return 2.0 * quaternion[1:]
    angle = 2.0 * np.arctan2(vector_norm, quaternion[0])
    return quaternion[1:] * (angle / vector_norm)


def _rotation_vector_to_matrix(vector: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(vector)
    if angle < 1e-12:
        skew = np.array(
            ((0.0, -vector[2], vector[1]),
             (vector[2], 0.0, -vector[0]),
             (-vector[1], vector[0], 0.0))
        )
        return np.eye(3) + skew + 0.5 * skew @ skew

    x, y, z = vector / angle
    skew = np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * skew @ skew


class RotationSpaceLowPassFilter:
    """在初始姿态的 SO(3) 切空间中进行二阶巴特沃斯滤波。"""

    def __init__(self, sample_rate_hz: float, cutoff_hz: float) -> None:
        self._vector_filter = SecondOrderButterworthLowPassFilter(
            sample_rate_hz, cutoff_hz
        )
        self.reset()

    def reset(self) -> None:
        self._reference: np.ndarray | None = None
        self._vector_filter.reset()

    def update(self, rotations: Any) -> np.ndarray:
        current = np.asarray(rotations, dtype=float)
        if current.shape[-2:] != (3, 3) or not np.all(np.isfinite(current)):
            raise ValueError("旋转滤波器输入必须是有限的 3x3 旋转矩阵")

        flattened = current.reshape(-1, 3, 3)
        orthogonality = np.swapaxes(flattened, 1, 2) @ flattened
        if not np.allclose(orthogonality, np.eye(3), atol=1e-5) or not np.allclose(
            np.linalg.det(flattened), 1.0, atol=1e-5
        ):
            raise ValueError("旋转滤波器输入不是合法的旋转矩阵")

        if self._reference is None:
            self._reference = current.copy()
            self._vector_filter.update(np.zeros(current.shape[:-1]))
            return current.copy()
        if current.shape != self._reference.shape:
            raise ValueError(
                "旋转滤波器输入形状不能在运行过程中改变"
            )

        relative = np.swapaxes(self._reference, -1, -2) @ current
        rotation_vectors = np.stack(
            [_rotation_matrix_to_vector(value) for value in relative.reshape(-1, 3, 3)]
        ).reshape(current.shape[:-1])
        filtered_vectors = self._vector_filter.update(rotation_vectors)
        filtered_relative = np.stack(
            [
                _rotation_vector_to_matrix(value)
                for value in filtered_vectors.reshape(-1, 3)
            ]
        ).reshape(current.shape)
        # shortcut: 固定初始切空间适合双臂活动范围；跨越 180° 时改用增量 Lie 群滤波。
        return self._reference @ filtered_relative
