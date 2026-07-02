import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class TransformSpec:
    parent: str
    child: str
    xyz: tuple[float, float, float]
    quat_xyzw: tuple[float, float, float, float]


IDENTITY_QUAT_XYZW = (0.0, 0.0, 0.0, 1.0)


def normalize_quat_xyzw(
    quat_xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in quat_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return IDENTITY_QUAT_XYZW
    return (x / norm, y / norm, z / norm, w / norm)


def mujoco_quat_wxyz_to_ros_xyzw(
    quat_wxyz: Sequence[float],
) -> tuple[float, float, float, float]:
    w, x, y, z = (float(value) for value in quat_wxyz)
    return normalize_quat_xyzw((x, y, z, w))


def has_required_joint_states(
    asm_version: int,
    received_joint1: bool,
    received_joint2: bool,
    received_joint3: bool,
) -> bool:
    if int(asm_version) in (1, 3):
        return bool(received_joint1 and received_joint2)
    if int(asm_version) in (2, 4):
        return bool(received_joint1 and received_joint2 and received_joint3)
    raise ValueError(f"Unsupported asm_version: {asm_version}")


def axis_angle_to_quat_xyzw(
    axis_xyz: Sequence[float],
    angle_rad: float,
) -> tuple[float, float, float, float]:
    x, y, z = (float(value) for value in axis_xyz)
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-12:
        return IDENTITY_QUAT_XYZW

    half_angle = float(angle_rad) / 2.0
    scale = math.sin(half_angle) / norm
    return normalize_quat_xyzw((
        x * scale,
        y * scale,
        z * scale,
        math.cos(half_angle),
    ))


def default_static_transforms(
    parent_frame: str = "tool0",
    asm_version: int = 1,
) -> list[TransformSpec]:
    if int(asm_version) == 1:
        return _v1_static_transforms(parent_frame)
    if int(asm_version) == 2:
        return _v2_static_transforms(parent_frame)
    if int(asm_version) == 3:
        return _v3_static_transforms(parent_frame)
    if int(asm_version) == 4:
        return _v4_static_transforms(parent_frame)
    raise ValueError(f"Unsupported asm_version: {asm_version}")


def _v1_static_transforms(
    parent_frame: str,
) -> list[TransformSpec]:
    return [
        TransformSpec(
            parent_frame,
            "asm_base",
            (0.0, 0.0, 0.0),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_base",
            "camera_link",
            (0.1, -0.0011863, 0.005),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_base",
            "asm_force_sensor_link",
            (0.0, 0.0, 0.0145),
            mujoco_quat_wxyz_to_ros_xyzw((0.0, 0.0, 0.0, -1.0)),
            # IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_force_sensor_link",
            "asm_tool_base_link",
            (0.0, 0.0, 0.0095),
            IDENTITY_QUAT_XYZW,
            # mujoco_quat_wxyz_to_ros_xyzw((0.0, 0.0, 0.0, -1.0)),
        ),
        TransformSpec(
            "asm_tool_link2",
            "asm_waterproof_link",
            (0.0, -0.00025, 0.016),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_tool_link2",
            "asm_ee_site",
            (0.0, 0.0, 0.0),
            IDENTITY_QUAT_XYZW,
        ),
    ]


def default_dynamic_transforms(
    joint1_rad: float,
    joint2_rad: float,
    joint3_rad: float = 0.0,
    asm_version: int = 1,
) -> list[TransformSpec]:
    if int(asm_version) == 1:
        return _v1_dynamic_transforms(joint1_rad, joint2_rad)
    if int(asm_version) == 2:
        return _v2_dynamic_transforms(joint1_rad, joint2_rad, joint3_rad)
    if int(asm_version) == 3:
        return _v3_dynamic_transforms(joint1_rad, joint2_rad)
    if int(asm_version) == 4:
        return _v4_dynamic_transforms(joint1_rad, joint2_rad, joint3_rad)
    raise ValueError(f"Unsupported asm_version: {asm_version}")


def _v1_dynamic_transforms(
    joint1_rad: float,
    joint2_rad: float,
) -> list[TransformSpec]:
    return [
        TransformSpec(
            "asm_tool_base_link",
            "asm_tool_link1",
            (0.0, 0.0, 0.074),
            axis_angle_to_quat_xyzw((0.0, -1.0, 0.0), joint1_rad),
        ),
        TransformSpec(
            "asm_tool_link1",
            "asm_tool_link2",
            (0.0, 0.0, 0.0495),
            axis_angle_to_quat_xyzw((1.0, 0.0, 0.0), joint2_rad),
        ),
    ]


def _v2_static_transforms(
    parent_frame: str,
) -> list[TransformSpec]:
    return [
        TransformSpec(
            parent_frame,
            "asm_base",
            (0.0, 0.0, 0.0),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_base",
            "camera_link",
            (0.1, -0.0011879, 0.005),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_base",
            "asm_force_sensor_link",
            (0.0, 0.0, 0.0145),
            mujoco_quat_wxyz_to_ros_xyzw((0.0, 0.0, 0.0, -1.0)),
        ),
        TransformSpec(
            "asm_force_sensor_link",
            "asm_tool_base_link",
            (0.0, 0.0, 0.0095),
            mujoco_quat_wxyz_to_ros_xyzw((0.0, 0.0, 0.0, 1.0)),
        ),
        TransformSpec(
            "asm_tool_link3",
            "asm_waterproof_link",
            (-0.00034889, -0.0013839, 0.028),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_waterproof_link",
            "asm_ee_site",
            (0.0, 0.0, 0.0),
            IDENTITY_QUAT_XYZW,
        ),
    ]


def _v2_dynamic_transforms(
    joint1_rad: float,
    joint2_rad: float,
    joint3_m: float,
) -> list[TransformSpec]:
    link1_z = round(0.098 + float(joint3_m), 8)
    return [
        TransformSpec(
            "asm_tool_base_link",
            "asm_tool_link1",
            (0.0, 0.0, link1_z),
            IDENTITY_QUAT_XYZW,
        ),
        *_v2_rotary_transforms(joint1_rad, joint2_rad),
    ]


def _v2_rotary_transforms(
    joint1_rad: float,
    joint2_rad: float,
) -> list[TransformSpec]:
    return [
        TransformSpec(
            "asm_tool_link1",
            "asm_tool_link2",
            (0.0, 0.0, 0.068),
            axis_angle_to_quat_xyzw((0.0, 1.0, 0.0), joint1_rad),
        ),
        TransformSpec(
            "asm_tool_link2",
            "asm_tool_link3",
            (0.00034889, 0.0, 0.0495),
            axis_angle_to_quat_xyzw((-1.0, 0.0, 0.0), -0.039683 + joint2_rad),
        ),
    ]


def _v3_static_transforms(
    parent_frame: str,
) -> list[TransformSpec]:
    return [
        *_v2_static_transforms(parent_frame),
        TransformSpec(
            "asm_tool_base_link",
            "asm_tool_link1",
            # (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.088),
            IDENTITY_QUAT_XYZW,
        ),
    ]


def _v3_dynamic_transforms(
    joint1_rad: float,
    joint2_rad: float,
) -> list[TransformSpec]:
    return _v2_rotary_transforms(joint1_rad, joint2_rad)

def _v4_static_transforms(
    parent_frame: str,
) -> list[TransformSpec]:
    return [
        TransformSpec(
            parent_frame,
            "asm_base",
            (0.0, 0.0, 0.0),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_base",
            "camera_link",
            (0.1, -0.0011879, 0.005),
            IDENTITY_QUAT_XYZW,
        ),
        TransformSpec(
            "asm_base",
            "asm_force_sensor_link",
            (0.0, 0.0, 0.0145),
            mujoco_quat_wxyz_to_ros_xyzw((0.0, 0.0, 0.0, -1.0)),
        ),
        TransformSpec(
            "asm_force_sensor_link",
            "asm_tool_base_link",
            (0.0, 0.0, 0.0095),
            mujoco_quat_wxyz_to_ros_xyzw((0.0, 0.0, 0.0, 1.0)),
        ),
        TransformSpec(
            "asm_tool_link3",
            "asm_ee_site",
            (0.0, 0.0, 0.06585),
            IDENTITY_QUAT_XYZW,
        )
    ]

def _v4_dynamic_transforms(
    joint1_rad: float,
    joint2_rad: float,
    joint3_m: float,
) -> list[TransformSpec]:
    return _v2_dynamic_transforms(joint1_rad, joint2_rad, joint3_m)
