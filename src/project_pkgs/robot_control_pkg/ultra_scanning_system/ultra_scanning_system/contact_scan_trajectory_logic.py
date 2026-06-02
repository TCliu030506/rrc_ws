import enum
from typing import Sequence, Tuple

from ultra_scanning_system.asm_ee_command_math import rotate_vector


"""接触扫查状态机的纯逻辑工具。

本文件不依赖 ROS，只保存几何计算、力阈值判断和状态枚举。
这样 ROS 节点只负责话题通信，核心规则可以用 pytest 单独验证。
"""

Vector3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Pose = Tuple[Vector3, Quat]
Wrench6 = Tuple[float, float, float, float, float, float]


class ContactScanState(enum.Enum):
    """
    接触扫查上层状态.

    APPROACH: 从当前位置运动到视觉粗路径起点。
    PRE_CONTACT: 沿工具接近方向低速搜索接触面。
    CONTACT_SETTLE: 原地爬升目标力并等待接触稳定。
    CONTACT_SCAN: 使用接触修正路径扫查，并发布目标接触力。
    RETRACT: 扫查结束后沿接近方向反向撤离。
    FINISHED/FAULT: 正常结束或安全故障停止。
    """

    APPROACH = 'approach'
    PRE_CONTACT = 'pre_contact'
    CONTACT_SETTLE = 'contact_settle'
    CONTACT_SCAN = 'contact_scan'
    RETRACT = 'retract'
    FINISHED = 'finished'
    FAULT = 'fault'


def _sign(value: float) -> float:
    """把用户配置的符号参数归一化为 +1 或 -1."""
    return 1.0 if value >= 0.0 else -1.0


def compute_approach_axis(
    orientation_xyzw: Sequence[float],
    *,
    approach_axis_sign: float,
) -> Vector3:
    """
    计算 base 坐标系下的工具接近方向.

    当前约定接近方向来自工具 Z 轴，`approach_axis_sign` 用于适配探头
    是沿工具 +Z 还是 -Z 压向表面。返回值始终表达在控制基坐标系下。
    """
    axis = rotate_vector(orientation_xyzw, (0.0, 0.0, 1.0))
    signed = _sign(approach_axis_sign)
    return (signed * axis[0], signed * axis[1], signed * axis[2])


def apply_contact_offset(
    path: Sequence[Pose],
    *,
    delta_c: float,
    approach_axis_sign: float,
) -> list[Pose]:
    """
    把起点接触搜索得到的法向偏置应用到整条粗路径.

    这一步对应 P0 -> P2：保留每个路径点原姿态，只沿该点工具接近方向
    平移 `delta_c`。初版假设视觉路径主要存在整体高度偏差。
    """
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


def align_contact_path_start(
    path: Sequence[Pose],
    *,
    stable_pose: Pose,
) -> list[Pose]:
    """
    将接触路径整体平移到稳定接触位姿起点.

    `delta_c` 只修正了粗路径的法向高度误差，不能修正切向误差。进入
    CONTACT_SCAN 前用稳定后的实际接触位姿对齐路径起点，可避免
    `/scan/desired_pose` 在状态切换时跳变。
    """
    if not path:
        return []

    stable_position, stable_orientation = stable_pose
    start_position, _ = path[0]
    offset = (
        stable_position[0] - start_position[0],
        stable_position[1] - start_position[1],
        stable_position[2] - start_position[2],
    )

    aligned: list[Pose] = []
    for index, (position, orientation) in enumerate(path):
        aligned_orientation = stable_orientation if index == 0 else orientation
        aligned.append((
            (
                position[0] + offset[0],
                position[1] + offset[1],
                position[2] + offset[2],
            ),
            aligned_orientation,
        ))
    return aligned


def normal_force(
    wrench: Wrench6,
    *,
    axis: str,
    force_axis_sign: float,
) -> float:
    """
    按配置读取法向接触力.

    项目当前的力符号以实测为准；这里不强行重新定义物理方向，只通过
    `force_axis_sign` 把“压紧力增大”为正的约定传给状态机。
    """
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
    """判断预接触搜索是否已经建立初始接触."""
    return normal_force_value >= contact_force_threshold


def should_fault_by_force(
    normal_force_value: float,
    *,
    max_contact_force: float,
) -> bool:
    """判断接触力是否超过安全上限."""
    return normal_force_value >= max_contact_force


def should_fault_by_search_distance(
    search_distance: float,
    *,
    max_search_distance: float,
) -> bool:
    """判断预接触搜索距离是否超过允许范围."""
    return search_distance >= max_search_distance


def ramp_toward(
    current_value: float,
    target_value: float,
    *,
    max_rate: float,
    dt: float,
) -> float:
    """按最大变化率让当前值缓慢逼近目标值."""
    if max_rate <= 0.0:
        return target_value
    if dt <= 0.0:
        return current_value

    max_step = max_rate * dt
    delta = target_value - current_value
    if delta > max_step:
        return current_value + max_step
    if delta < -max_step:
        return current_value - max_step
    return target_value


def compute_retract_pose(
    final_pose: Pose,
    *,
    retract_distance: float,
    approach_axis_sign: float,
) -> Pose:
    """
    根据最终位姿生成撤离目标.

    撤离方向取接近方向的反方向，姿态保持最终扫查姿态不变。
    """
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
