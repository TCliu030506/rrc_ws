from dataclasses import dataclass
from typing import Optional, Sequence

import mujoco
import numpy as np


@dataclass
class IkResult:
    qpos: np.ndarray
    converged: bool
    iterations: int
    position_error_norm: float
    orientation_error_norm: float


class DampedLeastSquaresIkSolver:
    """Small MuJoCo-based IK solver for actuator-controlled joints."""

    def __init__(
        self,
        model: mujoco.MjModel,
        site_id: int,
        controlled_joint_ids: Sequence[int],
        controlled_qpos_adrs: Sequence[int],
        controlled_dof_adrs: Sequence[int],
        *,
        max_iters: int = 50,
        position_tolerance: float = 1e-4,
        orientation_tolerance: float = 1e-3,
        damping: float = 1e-2,
        step_size: float = 0.5,
        max_position_step: float = 0.05,
        max_orientation_step: float = 0.25,
    ) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        self.site_id = int(site_id)
        self.controlled_joint_ids = np.asarray(controlled_joint_ids, dtype=np.int32)
        self.controlled_qpos_adrs = np.asarray(controlled_qpos_adrs, dtype=np.int32)
        self.controlled_dof_adrs = np.asarray(controlled_dof_adrs, dtype=np.int32)
        self.max_iters = max(1, int(max_iters))
        self.position_tolerance = max(0.0, float(position_tolerance))
        self.orientation_tolerance = max(0.0, float(orientation_tolerance))
        self.damping = max(1e-9, float(damping))
        self.step_size = max(1e-6, float(step_size))
        self.max_position_step = max(1e-9, float(max_position_step))
        self.max_orientation_step = max(1e-9, float(max_orientation_step))

    @staticmethod
    def _mat_to_quat(mat: np.ndarray) -> np.ndarray:
        quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quat, np.asarray(mat, dtype=np.float64))
        return quat

    @staticmethod
    def _quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
        result = np.zeros(4, dtype=np.float64)
        mujoco.mju_mulQuat(
            result,
            np.asarray(left, dtype=np.float64),
            np.asarray(right, dtype=np.float64),
        )
        return result

    def _quat_error(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=np.float64)
        current = np.asarray(current, dtype=np.float64)
        if np.dot(target, current) < 0.0:
            target = -target

        current_conj = np.array(
            [current[0], -current[1], -current[2], -current[3]],
            dtype=np.float64,
        )
        quat_error = self._quat_multiply(target, current_conj)
        vector = quat_error[1:]
        vector_norm = float(np.linalg.norm(vector))
        if vector_norm < 1e-12:
            return np.zeros(3, dtype=np.float64)
        scalar = float(np.clip(quat_error[0], -1.0, 1.0))
        angle = 2.0 * np.arctan2(vector_norm, scalar)
        return (vector / vector_norm) * angle

    @staticmethod
    def _limit_vector(vec: np.ndarray, max_norm: float) -> np.ndarray:
        norm = float(np.linalg.norm(vec))
        if norm <= max_norm or norm < 1e-12:
            return vec
        return vec * (max_norm / norm)

    def _apply_joint_limits(self, qpos: np.ndarray) -> None:
        for joint_id in self.controlled_joint_ids:
            if joint_id < 0 or not bool(self.model.jnt_limited[joint_id]):
                continue
            joint_type = self.model.jnt_type[joint_id]
            if joint_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                continue
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            low, high = self.model.jnt_range[joint_id]
            qpos[qpos_adr] = float(np.clip(qpos[qpos_adr], low, high))

    def solve(
        self,
        target_position: np.ndarray,
        target_quaternion: np.ndarray,
        q_init: np.ndarray,
    ) -> Optional[IkResult]:
        if self.site_id == -1:
            return None
        if self.controlled_qpos_adrs.size == 0 or self.controlled_dof_adrs.size == 0:
            return None

        target_position = np.asarray(target_position, dtype=np.float64)
        target_quaternion = np.asarray(target_quaternion, dtype=np.float64)
        quat_norm = float(np.linalg.norm(target_quaternion))
        if (
            not np.all(np.isfinite(target_position))
            or not np.all(np.isfinite(target_quaternion))
            or quat_norm < 1e-9
        ):
            return None
        target_quaternion = target_quaternion / quat_norm

        q = np.array(q_init, dtype=np.float64, copy=True)
        best_q = np.array(q, copy=True)
        best_pos_err = float("inf")
        best_ori_err = float("inf")
        jac_pos = np.zeros((3, self.model.nv), dtype=np.float64)
        jac_rot = np.zeros((3, self.model.nv), dtype=np.float64)
        identity = np.eye(6, dtype=np.float64)

        for iteration in range(1, self.max_iters + 1):
            self.data.qpos[:] = q
            mujoco.mj_forward(self.model, self.data)

            site = self.data.site(self.site_id)
            current_position = np.asarray(site.xpos, dtype=np.float64)
            current_quaternion = self._mat_to_quat(np.asarray(site.xmat, dtype=np.float64))

            position_error = target_position - current_position
            orientation_error = self._quat_error(target_quaternion, current_quaternion)
            position_norm = float(np.linalg.norm(position_error))
            orientation_norm = float(np.linalg.norm(orientation_error))

            if position_norm + orientation_norm < best_pos_err + best_ori_err:
                best_q[:] = q
                best_pos_err = position_norm
                best_ori_err = orientation_norm

            converged = (
                position_norm <= self.position_tolerance
                and orientation_norm <= self.orientation_tolerance
            )
            if converged:
                return IkResult(q, True, iteration, position_norm, orientation_norm)

            limited_position_error = self._limit_vector(
                position_error, self.max_position_step
            )
            limited_orientation_error = self._limit_vector(
                orientation_error, self.max_orientation_step
            )
            error = np.concatenate([limited_position_error, limited_orientation_error])

            mujoco.mj_jacSite(
                self.model, self.data, jac_pos, jac_rot, self.site_id
            )
            jacobian = np.vstack((jac_pos, jac_rot))
            jacobian_reduced = jacobian[:, self.controlled_dof_adrs]
            lhs = jacobian_reduced @ jacobian_reduced.T + (self.damping**2) * identity
            try:
                delta_q_reduced = jacobian_reduced.T @ np.linalg.solve(lhs, error)
            except np.linalg.LinAlgError:
                return IkResult(best_q, False, iteration, best_pos_err, best_ori_err)

            if not np.all(np.isfinite(delta_q_reduced)):
                return IkResult(best_q, False, iteration, best_pos_err, best_ori_err)

            dq = np.zeros(self.model.nv, dtype=np.float64)
            dq[self.controlled_dof_adrs] = self.step_size * delta_q_reduced
            mujoco.mj_integratePos(self.model, q, dq, 1.0)
            self._apply_joint_limits(q)

        return IkResult(best_q, False, self.max_iters, best_pos_err, best_ori_err)
