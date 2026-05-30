import enum
from typing import Sequence, Tuple

from ultra_scanning_system.asm_ee_command_math import rotate_vector


Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Pose = Tuple[Vector3, Quat]
Wrench6 = Tuple[float, float, float, float, float, float]


class ContactScanState(enum.Enum):
    APPROACH = 'approach'
    PRE_CONTACT = 'pre_contact'
    CONTACT_SCAN = 'contact_scan'
    RETRACT = 'retract'
    FINISHED = 'finished'
    FAULT = 'fault'


def _sign(value: float) -> float:
    return 1.0 if value >= 0.0 else -1.0


def compute_approach_axis(
    orientation_xyzw: Sequence[float],
    *,
    approach_axis_sign: float,
) -> Vector3:
    axis = rotate_vector(orientation_xyzw, (0.0, 0.0, 1.0))
    signed = _sign(approach_axis_sign)
    return (signed * axis[0], signed * axis[1], signed * axis[2])


def apply_contact_offset(
    path: Sequence[Pose],
    *,
    delta_c: float,
    approach_axis_sign: float,
) -> list[Pose]:
    shifted: list[Pose] = []
    for position, orientation in path:
        axis = compute_approach_axis(
            orientation,
            approach_axis_sign=approach_axis_sign,
        )
        shifted.append((
            (
                position[0] + delta_c * axis[0],
                position[1] + delta_c * axis[1],
                position[2] + delta_c * axis[2],
            ),
            orientation,
        ))
    return shifted


def normal_force(
    wrench: Wrench6,
    *,
    axis: str,
    force_axis_sign: float,
) -> float:
    if axis == 'x':
        value = wrench[0]
    elif axis == 'y':
        value = wrench[1]
    elif axis == 'z':
        value = wrench[2]
    else:
        raise ValueError("axis must be one of 'x', 'y', or 'z'")
    return _sign(force_axis_sign) * value


def should_enter_contact(
    normal_force_value: float,
    *,
    contact_force_threshold: float,
) -> bool:
    return normal_force_value >= contact_force_threshold


def should_fault_by_force(
    normal_force_value: float,
    *,
    max_contact_force: float,
) -> bool:
    return normal_force_value >= max_contact_force


def should_fault_by_search_distance(
    search_distance: float,
    *,
    max_search_distance: float,
) -> bool:
    return search_distance >= max_search_distance


def compute_retract_pose(
    final_pose: Pose,
    *,
    retract_distance: float,
    approach_axis_sign: float,
) -> Pose:
    position, orientation = final_pose
    axis = compute_approach_axis(
        orientation,
        approach_axis_sign=approach_axis_sign,
    )
    return (
        (
            position[0] - retract_distance * axis[0],
            position[1] - retract_distance * axis[1],
            position[2] - retract_distance * axis[2],
        ),
        orientation,
    )
