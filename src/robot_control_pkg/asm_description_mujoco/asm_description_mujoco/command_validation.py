from copy import deepcopy
from typing import Dict, Optional, Sequence

import numpy as np
from geometry_msgs.msg import PoseStamped


class CommandValidator:
    """Validate and limit ROS command messages before they reach MuJoCo."""

    def __init__(
        self,
        actuator_count: int,
        ctrl_limited: np.ndarray,
        ctrl_range: np.ndarray,
        joint_name_to_ctrl_index: Dict[str, int],
        logger: object,
        *,
        enable_clipping: bool = True,
        allow_partial_raw_commands: bool = False,
        world_frame: str = "world",
    ) -> None:
        self.actuator_count = int(actuator_count)
        self.ctrl_limited = np.asarray(ctrl_limited, dtype=bool)
        self.ctrl_range = np.asarray(ctrl_range, dtype=np.float64)
        self.joint_name_to_ctrl_index = joint_name_to_ctrl_index
        self.logger = logger
        self.enable_clipping = bool(enable_clipping)
        self.allow_partial_raw_commands = bool(allow_partial_raw_commands)
        self.world_frame = world_frame
        self._warned_keys = set()

    def _warn_once(self, key: str, message: str) -> None:
        if key in self._warned_keys:
            return
        self._warned_keys.add(key)
        self.logger.warn(message)

    @staticmethod
    def _is_finite_sequence(values: Sequence[float]) -> bool:
        return bool(np.all(np.isfinite(np.asarray(values, dtype=np.float64))))

    def _clip_value(self, ctrl_index: int, value: float) -> float:
        if (
            not self.enable_clipping
            or ctrl_index >= self.ctrl_limited.size
            or not self.ctrl_limited[ctrl_index]
        ):
            return float(value)

        low, high = self.ctrl_range[ctrl_index]
        clipped = float(np.clip(value, low, high))
        if clipped != float(value):
            self._warn_once(
                f"clip_ctrl_{ctrl_index}",
                (
                    f"Control command for actuator {ctrl_index} exceeded "
                    f"ctrlrange [{low:.4g}, {high:.4g}] and was clipped."
                ),
            )
        return clipped

    def clip_full_ctrl(self, ctrl: np.ndarray) -> np.ndarray:
        ctrl = np.asarray(ctrl, dtype=np.float64).copy()
        if not self.enable_clipping:
            return ctrl
        for idx in range(min(ctrl.size, self.ctrl_limited.size)):
            if np.isfinite(ctrl[idx]):
                ctrl[idx] = self._clip_value(idx, float(ctrl[idx]))
        return ctrl

    def validate_raw(self, values: Sequence[float]) -> Optional[np.ndarray]:
        if self.actuator_count <= 0:
            self._warn_once("raw_no_actuators", "Raw command ignored: model has no actuators.")
            return None
        if not values:
            return None

        if len(values) != self.actuator_count:
            if not self.allow_partial_raw_commands:
                self._warn_once(
                    "raw_length",
                    (
                        "Raw command ignored: data length must match actuator "
                        f"count ({self.actuator_count})."
                    ),
                )
                return None
            self._warn_once(
                "raw_partial",
                "Partial raw command accepted; unspecified actuators are filled with zero.",
            )

        length = min(len(values), self.actuator_count)
        ctrl = np.zeros(self.actuator_count, dtype=np.float64)
        ctrl[:length] = np.asarray(values[:length], dtype=np.float64)

        if not np.all(np.isfinite(ctrl)):
            self._warn_once("raw_nonfinite", "Raw command ignored: command contains NaN or Inf.")
            return None

        return self.clip_full_ctrl(ctrl)

    def validate_joint(
        self,
        names: Sequence[str],
        values: Sequence[float],
    ) -> Optional[np.ndarray]:
        if self.actuator_count <= 0:
            self._warn_once("joint_no_actuators", "Joint command ignored: model has no actuators.")
            return None
        if not names or not values:
            return None
        if len(values) < len(names):
            self._warn_once(
                "joint_short_values",
                "Joint command contains fewer values than joint names; extra names are ignored.",
            )

        joint_target = np.full(self.actuator_count, np.nan, dtype=np.float64)
        for name, value in zip(names, values):
            ctrl_index = self.joint_name_to_ctrl_index.get(str(name))
            if ctrl_index is None:
                self._warn_once(
                    f"joint_unmapped_{name}",
                    f"Joint '{name}' has no actuator control mapping; command entry ignored.",
                )
                continue
            value = float(value)
            if not np.isfinite(value):
                self._warn_once(
                    f"joint_nonfinite_{name}",
                    f"Joint '{name}' command ignored: value is NaN or Inf.",
                )
                continue
            joint_target[ctrl_index] = self._clip_value(ctrl_index, value)

        if not np.any(np.isfinite(joint_target)):
            return None
        return joint_target

    def validate_cartesian(self, msg: PoseStamped) -> Optional[PoseStamped]:
        pose = msg.pose
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64,
        )
        quat_xyzw = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quat_xyzw)):
            self._warn_once(
                "cartesian_nonfinite",
                "Cartesian command ignored: position or orientation contains NaN or Inf.",
            )
            return None

        quat_norm = float(np.linalg.norm(quat_xyzw))
        if quat_norm < 1e-9:
            self._warn_once(
                "cartesian_bad_quat",
                "Cartesian command ignored: orientation quaternion has near-zero norm.",
            )
            return None

        frame_id = msg.header.frame_id
        if frame_id and frame_id != self.world_frame:
            self._warn_once(
                "cartesian_frame",
                (
                    "Cartesian command frame_id is not transformed by this node; "
                    f"received '{frame_id}' and interpreting it as '{self.world_frame}'."
                ),
            )

        normalized = quat_xyzw / quat_norm
        out = deepcopy(msg)
        out.pose.orientation.x = float(normalized[0])
        out.pose.orientation.y = float(normalized[1])
        out.pose.orientation.z = float(normalized[2])
        out.pose.orientation.w = float(normalized[3])
        return out
