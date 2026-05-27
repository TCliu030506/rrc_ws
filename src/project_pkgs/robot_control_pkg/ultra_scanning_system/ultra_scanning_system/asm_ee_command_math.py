import math
from typing import Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Transform = Tuple[Vector3, Quat]

IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)


def normalize_quat(quat: Sequence[float]) -> Quat:
    x, y, z, w = (float(value) for value in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return IDENTITY_QUAT
    return (x / norm, y / norm, z / norm, w / norm)


def multiply_quat(left: Sequence[float], right: Sequence[float]) -> Quat:
    x1, y1, z1, w1 = normalize_quat(left)
    x2, y2, z2, w2 = normalize_quat(right)
    return normalize_quat(_multiply_quat_raw((x1, y1, z1, w1), (x2, y2, z2, w2)))


def _multiply_quat_raw(left: Sequence[float], right: Sequence[float]) -> Quat:
    x1, y1, z1, w1 = (float(value) for value in left)
    x2, y2, z2, w2 = (float(value) for value in right)
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


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
    xyz = (
        left_xyz[0] + rotated_right[0],
        left_xyz[1] + rotated_right[1],
        left_xyz[2] + rotated_right[2],
    )
    quat = multiply_quat(left_quat, right_quat)
    return (xyz, quat)


def invert_transform(transform: Transform) -> Transform:
    xyz, quat = transform
    inv_quat = conjugate_quat(quat)
    inv_xyz_rotated = rotate_vector(inv_quat, xyz)
    inv_xyz = (
        -inv_xyz_rotated[0],
        -inv_xyz_rotated[1],
        -inv_xyz_rotated[2],
    )
    return (inv_xyz, inv_quat)


def compute_base_to_tool_command(
    base_to_controlled: Transform,
    tool_to_controlled: Transform,
) -> Transform:
    return compose_transform(
        base_to_controlled,
        invert_transform(tool_to_controlled),
    )
