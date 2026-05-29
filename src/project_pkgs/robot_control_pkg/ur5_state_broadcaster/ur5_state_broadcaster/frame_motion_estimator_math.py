from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class FrameMotionEstimate:
    position: np.ndarray
    orientation_quat: tuple[float, float, float, float]
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    angular_acceleration: np.ndarray


def _limit_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0.0:
        return vector
    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm <= 1e-12:
        return vector
    return vector * (max_norm / norm)


def _low_pass(previous: np.ndarray | None, current: np.ndarray, dt: float, tau: float) -> np.ndarray:
    if previous is None or tau <= 0.0:
        return current
    alpha = dt / (tau + dt)
    alpha = max(0.0, min(1.0, alpha))
    return alpha * current + (1.0 - alpha) * previous


class FrameMotionEstimator:
    def __init__(
        self,
        *,
        min_dt: float = 1e-4,
        max_dt: float = 0.2,
        velocity_filter_tau: float = 0.02,
        accel_filter_tau: float = 0.05,
        max_linear_speed: float = 5.0,
        max_angular_speed: float = 20.0,
        max_linear_accel: float = 30.0,
        max_angular_accel: float = 80.0,
        express_in_target_frame: bool = True,
    ) -> None:
        self.min_dt = float(min_dt)
        self.max_dt = float(max_dt)
        self.velocity_filter_tau = float(velocity_filter_tau)
        self.accel_filter_tau = float(accel_filter_tau)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.max_linear_accel = float(max_linear_accel)
        self.max_angular_accel = float(max_angular_accel)
        self.express_in_target_frame = bool(express_in_target_frame)

        self._prev_time: float | None = None
        self._prev_position: np.ndarray | None = None
        self._prev_rotation: Rotation | None = None
        self._prev_linear_velocity_base: np.ndarray | None = None
        self._prev_angular_velocity_base: np.ndarray | None = None
        self._linear_velocity_base: np.ndarray | None = None
        self._angular_velocity_base: np.ndarray | None = None
        self._linear_acceleration_base: np.ndarray | None = None
        self._angular_acceleration_base: np.ndarray | None = None

    def update(
        self,
        stamp_sec: float,
        position: tuple[float, float, float],
        orientation_quat_xyzw: tuple[float, float, float, float],
    ) -> FrameMotionEstimate | None:
        current_time = float(stamp_sec)
        current_position = np.asarray(position, dtype=float)
        current_rotation = Rotation.from_quat(orientation_quat_xyzw)

        if self._prev_time is None:
            self._store_pose(current_time, current_position, current_rotation)
            return None

        dt = current_time - self._prev_time
        if dt < self.min_dt:
            return None
        if self.max_dt > 0.0 and dt > self.max_dt:
            self.reset()
            self._store_pose(current_time, current_position, current_rotation)
            return None

        raw_linear_velocity_base = (current_position - self._prev_position) / dt
        delta_rotation_base = current_rotation * self._prev_rotation.inv()
        raw_angular_velocity_base = delta_rotation_base.as_rotvec() / dt

        raw_linear_velocity_base = _limit_norm(raw_linear_velocity_base, self.max_linear_speed)
        raw_angular_velocity_base = _limit_norm(raw_angular_velocity_base, self.max_angular_speed)

        linear_velocity_base = _low_pass(
            self._linear_velocity_base,
            raw_linear_velocity_base,
            dt,
            self.velocity_filter_tau,
        )
        angular_velocity_base = _low_pass(
            self._angular_velocity_base,
            raw_angular_velocity_base,
            dt,
            self.velocity_filter_tau,
        )

        if self._prev_linear_velocity_base is None:
            raw_linear_acceleration_base = np.zeros(3, dtype=float)
            raw_angular_acceleration_base = np.zeros(3, dtype=float)
        else:
            raw_linear_acceleration_base = (
                linear_velocity_base - self._prev_linear_velocity_base
            ) / dt
            raw_angular_acceleration_base = (
                angular_velocity_base - self._prev_angular_velocity_base
            ) / dt

        raw_linear_acceleration_base = _limit_norm(
            raw_linear_acceleration_base,
            self.max_linear_accel,
        )
        raw_angular_acceleration_base = _limit_norm(
            raw_angular_acceleration_base,
            self.max_angular_accel,
        )

        linear_acceleration_base = _low_pass(
            self._linear_acceleration_base,
            raw_linear_acceleration_base,
            dt,
            self.accel_filter_tau,
        )
        angular_acceleration_base = _low_pass(
            self._angular_acceleration_base,
            raw_angular_acceleration_base,
            dt,
            self.accel_filter_tau,
        )

        estimate = self._make_estimate(
            current_position,
            current_rotation,
            linear_velocity_base,
            angular_velocity_base,
            linear_acceleration_base,
            angular_acceleration_base,
        )

        self._prev_time = current_time
        self._prev_position = current_position
        self._prev_rotation = current_rotation
        self._prev_linear_velocity_base = linear_velocity_base
        self._prev_angular_velocity_base = angular_velocity_base
        self._linear_velocity_base = linear_velocity_base
        self._angular_velocity_base = angular_velocity_base
        self._linear_acceleration_base = linear_acceleration_base
        self._angular_acceleration_base = angular_acceleration_base
        return estimate

    def reset(self) -> None:
        self._prev_time = None
        self._prev_position = None
        self._prev_rotation = None
        self._prev_linear_velocity_base = None
        self._prev_angular_velocity_base = None
        self._linear_velocity_base = None
        self._angular_velocity_base = None
        self._linear_acceleration_base = None
        self._angular_acceleration_base = None

    def _store_pose(
        self,
        stamp_sec: float,
        position: np.ndarray,
        rotation: Rotation,
    ) -> None:
        self._prev_time = stamp_sec
        self._prev_position = position
        self._prev_rotation = rotation

    def _make_estimate(
        self,
        position: np.ndarray,
        rotation: Rotation,
        linear_velocity_base: np.ndarray,
        angular_velocity_base: np.ndarray,
        linear_acceleration_base: np.ndarray,
        angular_acceleration_base: np.ndarray,
    ) -> FrameMotionEstimate:
        if self.express_in_target_frame:
            linear_velocity = rotation.inv().apply(linear_velocity_base)
            angular_velocity = rotation.inv().apply(angular_velocity_base)
            linear_acceleration = rotation.inv().apply(linear_acceleration_base)
            angular_acceleration = rotation.inv().apply(angular_acceleration_base)
        else:
            linear_velocity = linear_velocity_base
            angular_velocity = angular_velocity_base
            linear_acceleration = linear_acceleration_base
            angular_acceleration = angular_acceleration_base

        return FrameMotionEstimate(
            position=position,
            orientation_quat=tuple(float(value) for value in rotation.as_quat()),
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
            angular_acceleration=angular_acceleration,
        )
