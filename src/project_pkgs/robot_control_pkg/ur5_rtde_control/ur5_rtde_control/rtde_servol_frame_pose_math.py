import math
from typing import Callable, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Transform = Tuple[Vector3, Quat]
LookupTransform = Callable[[str, str], Transform]

IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)


def finite_transform(transform: Transform) -> bool:
    """检查位姿中是否包含 NaN/Inf，防止非法目标进入 servoL。"""
    position, orientation = transform
    return all(math.isfinite(float(value)) for value in (*position, *orientation))


def normalize_quat(quat: Sequence[float]) -> Quat:
    x, y, z, w = (float(value) for value in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12 or not math.isfinite(norm):
        return IDENTITY_QUAT
    return (x / norm, y / norm, z / norm, w / norm)


def _multiply_quat_raw(left: Sequence[float], right: Sequence[float]) -> Quat:
    x1, y1, z1, w1 = (float(value) for value in left)
    x2, y2, z2, w2 = (float(value) for value in right)
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def multiply_quat(left: Sequence[float], right: Sequence[float]) -> Quat:
    return normalize_quat(_multiply_quat_raw(normalize_quat(left), normalize_quat(right)))


def conjugate_quat(quat: Sequence[float]) -> Quat:
    x, y, z, w = normalize_quat(quat)
    return (-x, -y, -z, w)


def rotate_vector(quat: Sequence[float], vector: Sequence[float]) -> Vector3:
    vx, vy, vz = (float(value) for value in vector)
    q = normalize_quat(quat)
    rotated = _multiply_quat_raw(
        _multiply_quat_raw(q, (vx, vy, vz, 0.0)),
        conjugate_quat(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def compose_transform(left: Transform, right: Transform) -> Transform:
    left_xyz, left_quat = left
    right_xyz, right_quat = right
    rotated_right = rotate_vector(left_quat, right_xyz)
    return (
        (
            left_xyz[0] + rotated_right[0],
            left_xyz[1] + rotated_right[1],
            left_xyz[2] + rotated_right[2],
        ),
        multiply_quat(left_quat, right_quat),
    )


def invert_transform(transform: Transform) -> Transform:
    xyz, quat = transform
    inv_quat = conjugate_quat(quat)
    inv_xyz_rotated = rotate_vector(inv_quat, xyz)
    return (
        (-inv_xyz_rotated[0], -inv_xyz_rotated[1], -inv_xyz_rotated[2]),
        inv_quat,
    )


def compute_base_to_tool_command_from_frame(
    *,
    command_to_controlled: Transform,
    command_frame: str,
    base_frame: str,
    controlled_frame: str,
    tool_frame: str,
    lookup_transform: LookupTransform,
) -> Transform:
    """
    将“command_frame 下 controlled_frame 的目标位姿”转换成 base 下 tool_frame 目标位姿。

    现有导纳输出是 base->asm_ee_site；servoL 需要 base->tool0。这里先把
    command_frame->controlled_frame 转为 base->controlled_frame，再消去
    tool_frame->controlled_frame 的固定/动态 TF 偏置。
    """
    if not finite_transform(command_to_controlled):
        raise ValueError('command_to_controlled must contain only finite values')

    if command_frame == base_frame:
        base_to_controlled = command_to_controlled
    else:
        base_to_command = lookup_transform(base_frame, command_frame)
        if not finite_transform(base_to_command):
            raise ValueError('base_to_command transform must contain only finite values')
        base_to_controlled = compose_transform(base_to_command, command_to_controlled)

    tool_to_controlled = lookup_transform(tool_frame, controlled_frame)
    if not finite_transform(tool_to_controlled):
        raise ValueError('tool_to_controlled transform must contain only finite values')

    base_to_tool = compose_transform(base_to_controlled, invert_transform(tool_to_controlled))
    if not finite_transform(base_to_tool):
        raise ValueError('base_to_tool command must contain only finite values')
    return base_to_tool
