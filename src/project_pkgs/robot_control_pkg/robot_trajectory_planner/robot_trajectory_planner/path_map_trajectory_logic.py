import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


"""基于 path_map 的扫查轨迹纯逻辑工具。

本模块刻意不依赖 ROS。这样 ROS 节点可以只负责话题和参数，
路径解析、重采样、插值和速度估计都可以用普通 pytest 测试覆盖。
"""


Vector3 = tuple[float, float, float]
QuatXyzw = tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseState:
    position: Vector3
    orientation_xyzw: QuatXyzw


@dataclass(frozen=True)
class PathPoint(PoseState):
    pass


@dataclass(frozen=True)
class TrajectoryCommand:
    pose: PoseState
    twist_linear: Vector3
    twist_angular: Vector3
    accel_linear: Vector3
    accel_angular: Vector3
    path_index: int
    finished: bool


def parse_path_map_file(path: str | Path) -> list[PathPoint]:
    """读取 path_map.txt 文件并解析其中的位姿行。"""
    return parse_path_map_lines(Path(path).read_text(encoding='utf-8').splitlines())


def parse_path_map_lines(lines: Iterable[str]) -> list[PathPoint]:
    """解析格式为 x y z rx ry rz 的路径行。

    姿态部分是单位为 rad 的 rotation vector。这里会立即转换成四元数，
    这样后续姿态插值可以使用 slerp，而不是在 rotation vector 空间里
    直接做线性插值。
    """
    points: list[PathPoint] = []
    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue

        fields = line.split()
        if len(fields) != 6:
            raise ValueError(
                f'path_map line {line_number} must contain 6 numbers: x y z rx ry rz'
            )

        try:
            x, y, z, rx, ry, rz = (float(value) for value in fields)
        except ValueError as exc:
            raise ValueError(
                f'path_map line {line_number} contains a non-numeric value'
            ) from exc

        points.append(
            PathPoint(
                position=(x, y, z),
                orientation_xyzw=rotvec_to_quat_xyzw((rx, ry, rz)),
            )
        )

    if not points:
        raise ValueError('path_map contains no valid path points')
    return points


def rotvec_to_quat_xyzw(rotvec: Sequence[float]) -> QuatXyzw:
    """将 rotation vector 转换为归一化的 ROS xyzw 四元数。"""
    rx, ry, rz = (float(value) for value in rotvec)
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)

    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / angle
    return normalize_quat_xyzw((
        rx * scale,
        ry * scale,
        rz * scale,
        math.cos(half_angle),
    ))


def quat_to_rotvec(quat: Sequence[float]) -> Vector3:
    """将 ROS xyzw 四元数转换为 rotation vector。"""
    qx, qy, qz, qw = normalize_quat_xyzw(quat)
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw

    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector_norm < 1e-12:
        return (0.0, 0.0, 0.0)

    angle = 2.0 * math.atan2(vector_norm, qw)
    scale = angle / vector_norm
    return (qx * scale, qy * scale, qz * scale)


def normalize_quat_xyzw(quat: Sequence[float]) -> QuatXyzw:
    """返回单位四元数；输入无效时回退到单位姿态。"""
    x, y, z, w = (float(value) for value in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def offset_path_points_along_tool_z(
    points: Sequence[PoseState],
    *,
    offset_distance: float,
) -> list[PathPoint]:
    """把路径点沿各自姿态的局部 Z 轴偏置给定距离。"""
    shifted: list[PathPoint] = []
    for point in points:
        local_z_in_base = rotate_vector_by_quat(
            point.orientation_xyzw,
            (0.0, 0.0, 1.0),
        )
        shifted.append(PathPoint(
            position=(
                point.position[0] + offset_distance * local_z_in_base[0],
                point.position[1] + offset_distance * local_z_in_base[1],
                point.position[2] + offset_distance * local_z_in_base[2],
            ),
            orientation_xyzw=normalize_quat_xyzw(point.orientation_xyzw),
        ))
    return shifted


def format_path_map_lines(points: Sequence[PoseState]) -> list[str]:
    """把路径点格式化为 path_map.txt 的 x y z rx ry rz 行。"""
    lines: list[str] = []
    for point in points:
        rx, ry, rz = quat_to_rotvec(point.orientation_xyzw)
        values = (
            point.position[0],
            point.position[1],
            point.position[2],
            rx,
            ry,
            rz,
        )
        lines.append(' '.join(f'{value:.9g}' for value in values))
    return lines


def resample_waypoints(
    waypoints: Sequence[PoseState],
    *,
    max_linear_step: float,
    max_angular_step: float,
) -> list[PoseState]:
    """在轨迹时间参数化前，对稀疏路径点做加密重采样。

    每一段原始路径都会被拆分，直到相邻内部点之间的位置变化和姿态变化
    都低于配置上限。这样可以避免过大的稀疏目标点，同时保持原始路径的
    几何形状不变。
    """
    if not waypoints:
        raise ValueError('waypoints must not be empty')
    if max_linear_step <= 0.0:
        raise ValueError('max_linear_step must be > 0')
    if max_angular_step <= 0.0:
        raise ValueError('max_angular_step must be > 0')

    resampled = [
        PoseState(
            position=_as_vector3(waypoints[0].position),
            orientation_xyzw=normalize_quat_xyzw(waypoints[0].orientation_xyzw),
        )
    ]

    for start, target in zip(waypoints, waypoints[1:]):
        start_pose = PoseState(
            position=_as_vector3(start.position),
            orientation_xyzw=normalize_quat_xyzw(start.orientation_xyzw),
        )
        target_pose = PoseState(
            position=_as_vector3(target.position),
            orientation_xyzw=normalize_quat_xyzw(target.orientation_xyzw),
        )
        linear_steps = math.ceil(
            vector_distance(start_pose.position, target_pose.position) / max_linear_step
        )
        angular_steps = math.ceil(
            quat_angle(start_pose.orientation_xyzw, target_pose.orientation_xyzw) /
            max_angular_step
        )
        step_count = max(1, linear_steps, angular_steps)

        for step_index in range(1, step_count + 1):
            ratio = float(step_index) / float(step_count)
            resampled.append(interpolate_pose(start_pose, target_pose, ratio))

    return resampled


class PathMapTrajectory:
    """基于已解析路径点的时间参数化轨迹。"""

    def __init__(
        self,
        *,
        waypoints: Sequence[PoseState],
        start_pose: PoseState,
        max_linear_speed: float,
        max_angular_speed: float,
        loop_path: bool,
        min_segment_duration: float = 0.0,
    ) -> None:
        if not waypoints:
            raise ValueError('waypoints must not be empty')
        if max_linear_speed <= 0.0:
            raise ValueError('max_linear_speed must be > 0')
        if max_angular_speed <= 0.0:
            raise ValueError('max_angular_speed must be > 0')
        if min_segment_duration < 0.0:
            raise ValueError('min_segment_duration must be >= 0')

        self._waypoints = [
            PoseState(
                position=_as_vector3(point.position),
                orientation_xyzw=normalize_quat_xyzw(point.orientation_xyzw),
            )
            for point in waypoints
        ]
        self._current_pose = PoseState(
            position=_as_vector3(start_pose.position),
            orientation_xyzw=normalize_quat_xyzw(start_pose.orientation_xyzw),
        )
        self._max_linear_speed = float(max_linear_speed)
        self._max_angular_speed = float(max_angular_speed)
        self._loop_path = bool(loop_path)
        self._min_segment_duration = float(min_segment_duration)
        # _target_index 表示当前正在接近的路径点。因此第一段轨迹会从
        # 机器人实时位姿移动到 waypoints[0]。
        self._target_index = 0
        self._segment_start = self._current_pose
        self._segment_elapsed = 0.0
        self._finished = False
        self._segment_duration = self._compute_segment_duration(
            self._segment_start,
            self._waypoints[self._target_index],
        )

    @property
    def current_path_index(self) -> int:
        return self._target_index

    @property
    def finished(self) -> bool:
        return self._finished

    def advance(self, dt_sec: float) -> TrajectoryCommand:
        """按经过的时间推进轨迹，并生成当前周期应发布的控制指令。

        dt_sec 通常来自 ROS timer 两次触发之间的实际时间间隔。函数会把
        当前轨迹段的已运行时间向前推进 dt_sec，在当前段内按比例插值出
        新的目标位姿；如果本次 dt_sec 足够大导致越过当前目标点，则会
        自动切换到后续路径段并继续消耗剩余时间。返回的 TrajectoryCommand
        包含当前应发布的目标位姿、由本次位姿变化差分得到的线速度/角速度、
        置零的加速度，以及当前路径索引和轨迹结束标志。
        """
        dt = max(0.0, float(dt_sec))
        previous_pose = self._current_pose
        remaining = dt

        while remaining > 1e-12 and not self._finished:
            if self._segment_duration <= 1e-12:
                self._finish_segment()
                continue

            # 如果 timer 间隔较大，可能一次跨过某个路径点。这里先消耗
            # 当前段所需的时间，再继续进入下一段。
            segment_remaining = self._segment_duration - self._segment_elapsed
            step = min(remaining, segment_remaining)
            self._segment_elapsed += step
            remaining -= step

            ratio = min(1.0, self._segment_elapsed / self._segment_duration)
            self._current_pose = interpolate_pose(
                self._segment_start,
                self._waypoints[self._target_index],
                ratio,
            )

            if self._segment_elapsed >= self._segment_duration - 1e-12:
                self._current_pose = self._waypoints[self._target_index]
                self._finish_segment()

        # desired twist 根据本次 timer 周期实际发布的位姿变化计算。
        # 输入路径没有可靠的加速度信息，因此 desired accel 保持为 0。
        linear_velocity = (
            (self._current_pose.position[0] - previous_pose.position[0]) / dt,
            (self._current_pose.position[1] - previous_pose.position[1]) / dt,
            (self._current_pose.position[2] - previous_pose.position[2]) / dt,
        ) if dt > 1e-12 else (0.0, 0.0, 0.0)
        angular_velocity = quat_delta_to_rotvec(
            previous_pose.orientation_xyzw,
            self._current_pose.orientation_xyzw,
            dt,
        ) if dt > 1e-12 else (0.0, 0.0, 0.0)

        return TrajectoryCommand(
            pose=self._current_pose,
            twist_linear=linear_velocity,
            twist_angular=angular_velocity,
            accel_linear=(0.0, 0.0, 0.0),
            accel_angular=(0.0, 0.0, 0.0),
            path_index=self._target_index,
            finished=self._finished,
        )

    def _finish_segment(self) -> None:
        """切换到下一个路径点，或标记有限路径已经结束。"""
        if self._target_index + 1 < len(self._waypoints):
            self._target_index += 1
        elif self._loop_path:
            self._target_index = 0
        else:
            self._finished = True
            return

        self._segment_start = self._current_pose
        self._segment_elapsed = 0.0
        self._segment_duration = self._compute_segment_duration(
            self._segment_start,
            self._waypoints[self._target_index],
        )

    def _compute_segment_duration(self, start: PoseState, target: PoseState) -> float:
        """计算同时满足线速度和角速度限制的段持续时间。"""
        linear_time = vector_distance(start.position, target.position) / self._max_linear_speed
        angular_time = (
            quat_angle(start.orientation_xyzw, target.orientation_xyzw) /
            self._max_angular_speed
        )
        return max(linear_time, angular_time, self._min_segment_duration)


def interpolate_pose(start: PoseState, target: PoseState, ratio: float) -> PoseState:
    """位置线性插值，姿态使用四元数 slerp 插值。"""
    u = min(1.0, max(0.0, float(ratio)))
    return PoseState(
        position=(
            start.position[0] + (target.position[0] - start.position[0]) * u,
            start.position[1] + (target.position[1] - start.position[1]) * u,
            start.position[2] + (target.position[2] - start.position[2]) * u,
        ),
        orientation_xyzw=slerp_quat_xyzw(
            start.orientation_xyzw,
            target.orientation_xyzw,
            u,
        ),
    )


def vector_distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(
        (b[0] - a[0]) ** 2 +
        (b[1] - a[1]) ** 2 +
        (b[2] - a[2]) ** 2
    )


def quat_angle(a: QuatXyzw, b: QuatXyzw) -> float:
    """返回两个四元数之间的最短姿态角距离。"""
    qa = normalize_quat_xyzw(a)
    qb = normalize_quat_xyzw(b)
    dot = abs(_quat_dot(qa, qb))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def slerp_quat_xyzw(a: QuatXyzw, b: QuatXyzw, ratio: float) -> QuatXyzw:
    """对 ROS xyzw 四元数执行球面线性插值。"""
    qa = normalize_quat_xyzw(a)
    qb = normalize_quat_xyzw(b)
    dot = _quat_dot(qa, qb)
    if dot < 0.0:
        qb = tuple(-value for value in qb)  # type: ignore[assignment]
        dot = -dot

    if dot > 0.9995:
        return normalize_quat_xyzw(tuple(
            qa[index] + (qb[index] - qa[index]) * ratio
            for index in range(4)
        ))

    dot = min(1.0, max(-1.0, dot))
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    scale_a = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_b = sin_theta / sin_theta_0
    return normalize_quat_xyzw(tuple(
        qa[index] * scale_a + qb[index] * scale_b
        for index in range(4)
    ))


def rotate_vector_by_quat(quat: Sequence[float], vector: Sequence[float]) -> Vector3:
    """把局部坐标向量按 xyzw 四元数旋转到基坐标表达。"""
    q = normalize_quat_xyzw(quat)
    vx, vy, vz = _as_vector3(vector)
    rotated = _quat_multiply(
        _quat_multiply(q, (vx, vy, vz, 0.0)),
        _quat_inverse(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def quat_delta_to_rotvec(start: QuatXyzw, end: QuatXyzw, dt_sec: float) -> Vector3:
    """将 dt 时间内的四元数变化转换为角速度向量。"""
    delta = _quat_multiply(_quat_inverse(start), end)
    delta = normalize_quat_xyzw(delta)
    if delta[3] < 0.0:
        delta = (-delta[0], -delta[1], -delta[2], -delta[3])

    vector_norm = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
    if vector_norm < 1e-12:
        return (0.0, 0.0, 0.0)

    angle = 2.0 * math.atan2(vector_norm, delta[3])
    scale = angle / (vector_norm * dt_sec)
    return (delta[0] * scale, delta[1] * scale, delta[2] * scale)


def _as_vector3(values: Sequence[float]) -> Vector3:
    if len(values) != 3:
        raise ValueError('vector must contain exactly 3 values')
    return (float(values[0]), float(values[1]), float(values[2]))


def _quat_dot(a: QuatXyzw, b: QuatXyzw) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def _quat_inverse(q: QuatXyzw) -> QuatXyzw:
    x, y, z, w = normalize_quat_xyzw(q)
    return (-x, -y, -z, w)


def _quat_multiply(a: QuatXyzw, b: QuatXyzw) -> QuatXyzw:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
