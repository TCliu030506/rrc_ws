#!/usr/bin/env python3
"""
圆柱形表面扫描轨迹生成节点

功能：生成沿圆柱表面（Y轴方向管道）的扫查轨迹
发布 Pose/Twist/Accel 到可配置的话题（与 scan_raster_trajectory_node 接口一致）

轨迹规划：
- 沿 Y 轴分层扫描（从 y_start 到 y_end，步长 y_step）
- 每层在 XOZ 平面生成一段圆弧（圆心在 (center_x, center_z)，指定半径）
- 相邻层交替方向，实现圆柱表面的往返覆盖
"""

import ast
import math
import time
from copy import deepcopy
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Accel, Pose, PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


def _quat_from_two_vectors(v_from: Tuple[float, float, float], v_to: Tuple[float, float, float]):
    """
    根据两个向量计算四元数
    返回将 v_from 旋转到 v_to 的四元数 (x,y,z,w)
    
    参数：
        v_from: 起始向量
        v_to: 目标向量
    
    返回：
        四元数 (qx, qy, qz, qw)
    """
    fx, fy, fz = v_from
    tx, ty, tz = v_to
    
    # 归一化向量
    fn = math.sqrt(fx * fx + fy * fy + fz * fz)
    tn = math.sqrt(tx * tx + ty * ty + tz * tz)
    if fn == 0 or tn == 0:
        return (0.0, 0.0, 0.0, 1.0)
    
    fx, fy, fz = fx / fn, fy / fn, fz / fn
    tx, ty, tz = tx / tn, ty / tn, tz / tn
    
    # 计算点积
    dot = fx * tx + fy * ty + fz * tz
    dot = max(-1.0, min(1.0, dot))
    
    # 特殊情况：向量几乎相同
    if dot > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    
    # 特殊情况：向量几乎相反（180度旋转）
    if dot < -0.999999:
        # 找正交轴
        ax = 1.0 if abs(fx) < 0.1 else 0.0
        ay = 0.0
        az = 0.0
        # 与任意轴的叉积
        cx = fy * az - fz * ay
        cy = fz * ax - fx * az
        cz = fx * ay - fy * ax
        cn = math.sqrt(cx * cx + cy * cy + cz * cz)
        if cn == 0:
            return (0.0, 0.0, 0.0, 1.0)
        cx, cy, cz = cx / cn, cy / cn, cz / cn
        return (cx * math.sin(math.pi / 2), cy * math.sin(math.pi / 2), cz * math.sin(math.pi / 2), math.cos(math.pi / 2))
    
    # 计算旋转轴（叉积）
    ax = fy * tz - fz * ty
    ay = fz * tx - fx * tz
    az = fx * ty - fy * tx
    an = math.sqrt(ax * ax + ay * ay + az * az)
    if an == 0:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = ax / an, ay / an, az / an
    
    # 计算旋转角度和四元数
    angle = math.acos(dot)
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


class ScanCylindricalSurfaceNode(Node):
    """
    圆柱形表面扫描轨迹节点类
    
    功能：生成圆柱表面的螺旋扫描轨迹，适用于管道检测等场景
    """
    
    def __init__(self) -> None:
        """初始化节点"""
        super().__init__('scan_cylindrical_surface_node')

        # ========== 话题参数 ==========
        self.declare_parameter('topic_desired_pose', '/scan/desired_pose')
        self.declare_parameter('topic_desired_twist', '/scan/desired_twist')
        self.declare_parameter('topic_desired_accel', '/scan/desired_accel')
        self.declare_parameter('publish_rate', 125.0)

        # ========== 圆柱几何参数（管道沿Y轴）==========
        self.declare_parameter('center_x', 0.45)      # 圆柱中心 X 坐标
        self.declare_parameter('center_z', 0.45)      # 圆柱中心 Z 坐标
        self.declare_parameter('radius', 0.20)        # 圆柱半径
        self.declare_parameter('y_start', 0.0)        # Y轴起始位置
        self.declare_parameter('y_end', 0.2)          # Y轴结束位置
        self.declare_parameter('y_step', 0.02)        # Y轴步长

        # ========== 圆弧采样参数 ==========
        self.declare_parameter('arc_points', 200)     # 每段圆弧的采样点数
        self.declare_parameter('arc_start_deg', 0.0)  # 圆弧起始角度（度）
        self.declare_parameter('arc_end_deg', 360.0)  # 圆弧结束角度（度）

        # ========== 姿态控制参数 ==========
        # orientation_mode: 'fixed' 使用固定四元数; 'normal' 工具Z轴对齐表面法线
        self.declare_parameter('orientation_mode', 'normal')
        self.declare_parameter('orientation_xyzw', [0.0, 0.7071, 0.0, 0.7071])
        # 可选的沿法线向内的压入偏移（m），用于让末端贴服管道表面向内压紧
        self.declare_parameter('contact_offset', 0.0)

        # ========== 初始过渡时间 ==========
        self.declare_parameter('initial_blend_duration', 10.0)
        self.declare_parameter('current_pose_topic', '/end_effector_pose')
        # ========== 层间Y平滑过渡参数 ==========
        # 当完成一层圆弧后，在从当前 y 移动到下一层 y 时使用该时长进行线性过渡
        # 单位：秒。若设置为0则仍为原有跳变行为
        self.declare_parameter('y_transition_duration', 0.5)

        # ========== 获取参数值 ==========
        topic_pose = str(self.get_parameter('topic_desired_pose').value)
        topic_twist = str(self.get_parameter('topic_desired_twist').value)
        topic_accel = str(self.get_parameter('topic_desired_accel').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.cx = float(self.get_parameter('center_x').value)
        self.cz = float(self.get_parameter('center_z').value)
        self.radius = float(self.get_parameter('radius').value)
        self.y_start = float(self.get_parameter('y_start').value)
        self.y_end = float(self.get_parameter('y_end').value)
        self.y_step = float(self.get_parameter('y_step').value)

        self.arc_points = int(self.get_parameter('arc_points').value)
        self.arc_start_deg = float(self.get_parameter('arc_start_deg').value)
        self.arc_end_deg = float(self.get_parameter('arc_end_deg').value)

        self.orientation_mode = str(self.get_parameter('orientation_mode').value)
        self.fixed_orientation = self._parse_orientation_xyzw(
            self.get_parameter('orientation_xyzw').value
        )
        self.contact_offset = float(self.get_parameter('contact_offset').value)

        self.initial_blend_duration = float(self.get_parameter('initial_blend_duration').value)
        self.current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        self.y_transition_duration = float(self.get_parameter('y_transition_duration').value)

        # ========== 参数校验 ==========
        if self.publish_rate <= 0:
            raise ValueError('publish_rate 必须大于 0')
        if self.radius <= 0:
            raise ValueError('radius 必须大于 0')
        if self.y_step == 0:
            raise ValueError('y_step 不能为 0')
        if self.arc_points < 2:
            raise ValueError('arc_points 必须大于等于 2')
        if self.contact_offset < 0.0:
            raise ValueError('contact_offset 必须 >= 0')
        if self.contact_offset >= self.radius:
            raise ValueError('contact_offset 必须小于 radius')

        # ========== 创建发布者 ==========
        self.pose_pub = self.create_publisher(Pose, topic_pose, 10)
        self.twist_pub = self.create_publisher(Twist, topic_twist, 10)
        self.accel_pub = self.create_publisher(Accel, topic_accel, 10)

        # 当前末端位姿，用于在开始扫查前生成平滑过渡段
        self.current_pose: PoseStamped | None = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.current_pose_topic,
            self._current_pose_callback,
            qos_profile_sensor_data,
        )

        # ========== 预计算轨迹数据 ==========
        # 计算所有Y层位置
        self.layers = self._compute_layers(self.y_start, self.y_end, self.y_step)
        # 预计算所有角度（弧度）
        self.angles = [
            math.radians(self.arc_start_deg + (self.arc_end_deg - self.arc_start_deg) * i / (self.arc_points - 1)) 
            for i in range(self.arc_points)
        ]

        # ========== 状态变量 ==========
        self._start_time = time.monotonic()           # 节点启动时间
        self._initial_blend_started = False            # 起始过渡是否已初始化
        self._initial_blend_complete = False           # 起始过渡是否已完成
        self._initial_blend_step = 0                   # 起始过渡当前步数
        self._initial_blend_steps = max(1, int(round(self.initial_blend_duration * self.publish_rate)))  # 起始过渡总步数
        self._initial_blend_start_pose = None          # 起始过渡起点位姿
        self._initial_blend_target_pose = None         # 起始过渡终点位姿
        self._waiting_for_pose_logged = False          # 是否已提示等待当前位姿

        self._layer_idx = 0                            # 当前层索引
        self._angle_idx = 0                            # 当前角度索引
        # 层间过渡状态
        self._in_transition = False
        self._transition_steps = max(1, int(self.y_transition_duration * self.publish_rate))
        self._transition_step = 0
        self._transition_from_y = None
        self._transition_to_y = None
        self._transition_angle = None

        # ========== 创建定时器 ==========
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        # 输出启动信息
        self.get_logger().info(
            f'圆柱形表面扫描启动！'
            f'层数={len(self.layers)}, 每圆弧点数={self.arc_points}, '
            f'话题=({topic_pose}, {topic_twist}, {topic_accel})'
        )

    @staticmethod
    def _parse_orientation_xyzw(value) -> List[float]:
        if isinstance(value, str):
            text = value.strip()
            try:
                value = ast.literal_eval(text)
            except (ValueError, SyntaxError):
                value = [part.strip() for part in text.split(',') if part.strip()]

        if not isinstance(value, (list, tuple)) or len(value) != 4:
            raise ValueError('orientation_xyzw must be length 4')

        orientation = [float(v) for v in value]
        norm = math.sqrt(sum(v * v for v in orientation))
        if norm < 1e-12:
            raise ValueError('orientation_xyzw quaternion norm is too small')
        return [v / norm for v in orientation]

    @staticmethod
    def _compute_layers(y_start: float, y_end: float, y_step: float) -> List[float]:
        """
        计算所有Y层的位置
        
        参数：
            y_start: Y轴起始位置
            y_end: Y轴结束位置
            y_step: Y轴步长（可正可负）
        
        返回：
            Y层位置列表
        """
        layers = []
        if y_step > 0:
            # 正向扫描
            y = y_start
            while y <= y_end + 1e-9:  # 添加微小偏移避免浮点误差
                layers.append(y)
                y += y_step
        else:
            # 反向扫描
            y = y_start
            while y >= y_end - 1e-9:
                layers.append(y)
                y += y_step
        # 确保至少有一个层
        if not layers:
            layers = [y_start]
        return layers

    def _compute_point(self, angle: float, layer_y: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        计算圆柱表面上的点坐标和法线向量
        
        参数：
            angle: 当前角度（弧度）
            layer_y: 当前层的Y坐标
        
        返回：
            (position, normal): 位置 (x,y,z) 和表面法线 (nx,ny,nz)
        """
        # 圆柱表面位置（XOZ平面圆弧）
        x = self.cx + self.radius * math.cos(angle)
        z = self.cz + self.radius * math.sin(angle)
        y = layer_y
        
        # 表面法线（指向外侧）
        nx = math.cos(angle)  # X方向分量
        ny = 0.0              # Y方向分量为0（圆柱沿Y轴）
        nz = math.sin(angle)  # Z方向分量
        
        return (x, y, z), (nx, ny, nz)

    @staticmethod
    def _slerp_quaternion(
        q0: Tuple[float, float, float, float],
        q1: Tuple[float, float, float, float],
        alpha: float,
    ) -> Tuple[float, float, float, float]:
        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1

        norm0 = math.sqrt(x0 * x0 + y0 * y0 + z0 * z0 + w0 * w0)
        norm1 = math.sqrt(x1 * x1 + y1 * y1 + z1 * z1 + w1 * w1)
        if norm0 == 0.0 or norm1 == 0.0:
            return (0.0, 0.0, 0.0, 1.0)

        x0, y0, z0, w0 = x0 / norm0, y0 / norm0, z0 / norm0, w0 / norm0
        x1, y1, z1, w1 = x1 / norm1, y1 / norm1, z1 / norm1, w1 / norm1

        dot = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1
        if dot < 0.0:
            x1, y1, z1, w1 = -x1, -y1, -z1, -w1
            dot = -dot

        dot = max(-1.0, min(1.0, dot))

        if dot > 0.9995:
            x = x0 + alpha * (x1 - x0)
            y = y0 + alpha * (y1 - y0)
            z = z0 + alpha * (z1 - z0)
            w = w0 + alpha * (w1 - w0)
            norm = math.sqrt(x * x + y * y + z * z + w * w)
            if norm == 0.0:
                return (0.0, 0.0, 0.0, 1.0)
            return (x / norm, y / norm, z / norm, w / norm)

        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        if sin_theta_0 == 0.0:
            return (x0, y0, z0, w0)

        theta = theta_0 * alpha
        sin_theta = math.sin(theta)

        scale0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        scale1 = sin_theta / sin_theta_0

        return (
            scale0 * x0 + scale1 * x1,
            scale0 * y0 + scale1 * y1,
            scale0 * z0 + scale1 * z1,
            scale0 * w0 + scale1 * w1,
        )

    def _current_pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _build_scan_pose(self, layer_idx: int, angle_idx: int) -> Pose:
        layer_y = self.layers[layer_idx % len(self.layers)]
        forward = (layer_idx % 2) == 0
        if forward:
            actual_angle_idx = angle_idx % len(self.angles)
        else:
            actual_angle_idx = (len(self.angles) - 1) - (angle_idx % len(self.angles))

        angle = self.angles[actual_angle_idx]
        (x, y, z), normal = self._compute_point(angle, layer_y)

        if self.contact_offset != 0.0:
            x = x - self.contact_offset * normal[0]
            z = z - self.contact_offset * normal[2]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        if self.orientation_mode == 'fixed':
            pose.orientation.x = float(self.fixed_orientation[0])
            pose.orientation.y = float(self.fixed_orientation[1])
            pose.orientation.z = float(self.fixed_orientation[2])
            pose.orientation.w = float(self.fixed_orientation[3])
        else:
            inward_normal = (-normal[0], -normal[1], -normal[2])
            qx, qy, qz, qw = _quat_from_two_vectors((0.0, 0.0, 1.0), inward_normal)
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)

        return pose

    @staticmethod
    def _interpolate_pose(start_pose: Pose, target_pose: Pose, alpha: float) -> Pose:
        pose = Pose()
        pose.position.x = start_pose.position.x + (target_pose.position.x - start_pose.position.x) * alpha
        pose.position.y = start_pose.position.y + (target_pose.position.y - start_pose.position.y) * alpha
        pose.position.z = start_pose.position.z + (target_pose.position.z - start_pose.position.z) * alpha

        qx, qy, qz, qw = ScanCylindricalSurfaceNode._slerp_quaternion(
            (
                start_pose.orientation.x,
                start_pose.orientation.y,
                start_pose.orientation.z,
                start_pose.orientation.w,
            ),
            (
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ),
            alpha,
        )
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    @staticmethod
    def _ease_out_cubic(alpha: float) -> float:
        """
        三次缓出曲线：前快后慢，适合靠近目标时降低速度。
        alpha 输入范围应为 [0, 1]。
        """
        alpha = max(0.0, min(1.0, alpha))
        return 1.0 - (1.0 - alpha) ** 3

    def _prepare_initial_blend(self) -> None:
        if self.current_pose is None:
            return

        self._initial_blend_start_pose = deepcopy(self.current_pose.pose)
        self._initial_blend_target_pose = self._build_scan_pose(0, 0)
        self._initial_blend_started = True
        self._initial_blend_complete = False
        self._initial_blend_step = 0

        self.get_logger().info(
            f'已收到 {self.current_pose_topic}，开始从当前位姿平滑过渡到扫查起点，'
            f'过渡时长={self.initial_blend_duration:.2f}s'
        )

    def _publish_initial_blend_step(self) -> None:
        assert self._initial_blend_start_pose is not None
        assert self._initial_blend_target_pose is not None

        if self.initial_blend_duration <= 0.0:
            pose = deepcopy(self._initial_blend_target_pose)
            twist = Twist()
            accel = Accel()
            self.pose_pub.publish(pose)
            self.twist_pub.publish(twist)
            self.accel_pub.publish(accel)
            self._initial_blend_complete = True
            self._layer_idx = 0
            self._angle_idx = 1
            return

        step = min(self._initial_blend_step, self._initial_blend_steps)
        next_step = min(step + 1, self._initial_blend_steps)

        blend_ratio = float(step) / float(self._initial_blend_steps)
        next_blend_ratio = float(next_step) / float(self._initial_blend_steps)
        smooth_ratio = self._ease_out_cubic(blend_ratio)
        next_smooth_ratio = self._ease_out_cubic(next_blend_ratio)

        pose = self._interpolate_pose(self._initial_blend_start_pose, self._initial_blend_target_pose, smooth_ratio)
        next_pose = self._interpolate_pose(self._initial_blend_start_pose, self._initial_blend_target_pose, next_smooth_ratio)

        dt = 1.0 / self.publish_rate
        twist = Twist()
        twist.linear.x = float((next_pose.position.x - pose.position.x) / dt)
        twist.linear.y = float((next_pose.position.y - pose.position.y) / dt)
        twist.linear.z = float((next_pose.position.z - pose.position.z) / dt)

        accel = Accel()
        accel.linear.x = 0.0
        accel.linear.y = 0.0
        accel.linear.z = 0.0

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.accel_pub.publish(accel)

        self._initial_blend_step += 1
        if self._initial_blend_step > self._initial_blend_steps:
            self._initial_blend_complete = True
            self._layer_idx = 0
            self._angle_idx = 1

    def _on_timer(self) -> None:
        """
        定时器回调函数：生成并发布轨迹数据
        """
        if self.current_pose is None:
            if not self._waiting_for_pose_logged:
                self.get_logger().info(f'等待从 {self.current_pose_topic} 接收当前位姿后再开始扫查...')
                self._waiting_for_pose_logged = True
            return

        if not self._initial_blend_started:
            self._prepare_initial_blend()

        if not self._initial_blend_complete:
            self._publish_initial_blend_step()
            return

        # 如果没有层数据，直接返回
        if not self.layers:
            return

        # ========== 计算当前位置 ==========
        # 当前层索引（循环）
        layer_idx = self._layer_idx % len(self.layers)
        # 如果处于层间过渡中，则按过渡进度计算 layer_y
        if self._in_transition:
            step = self._transition_step
            steps = max(1, self._transition_steps)
            alpha = float(step) / float(steps)
            layer_y = (1.0 - alpha) * self._transition_from_y + alpha * self._transition_to_y
        else:
            layer_y = self.layers[layer_idx]

        # 相邻层交替方向（往返扫描）
        forward = (layer_idx % 2) == 0
        if forward:
            angle_idx = self._angle_idx % len(self.angles)
        else:
            angle_idx = (len(self.angles) - 1) - (self._angle_idx % len(self.angles))

        # 获取当前角度并计算位置
        # 过渡阶段保持角度不变，确保仅在 Y 方向移动，避免 XZ 跳变
        if self._in_transition and self._transition_angle is not None:
            angle = self._transition_angle
        else:
            angle = self.angles[angle_idx]
        (x, y, z), normal = self._compute_point(angle, layer_y)

        # 如果需要向内压紧，沿法线方向向内偏移位置（法线为外向，向内为取反）
        if self.contact_offset != 0.0:
            x = x - self.contact_offset * normal[0]
            z = z - self.contact_offset * normal[2]

        # ========== 构建位姿消息 ==========
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)

        # 设置姿态
        if self.orientation_mode == 'fixed':
            # 使用固定姿态
            pose.orientation.x = float(self.fixed_orientation[0])
            pose.orientation.y = float(self.fixed_orientation[1])
            pose.orientation.z = float(self.fixed_orientation[2])
            pose.orientation.w = float(self.fixed_orientation[3])
        else:
            # 将工具 Z 轴对齐至指向管道内部的法线（取外向法线的相反方向）
            # 假设工具本地前向向量为 [0,0,1]
            inward_normal = (-normal[0], -normal[1], -normal[2])
            qx, qy, qz, qw = _quat_from_two_vectors((0.0, 0.0, 1.0), inward_normal)
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)

        # ========== 计算速度（有限差分）==========
        dt = 1.0 / self.publish_rate  # 时间间隔

        # 计算下一时刻的位置以估算速度：如果处于层间过渡，则下一步继续沿过渡移动；否则沿圆弧前进一步
        if self._in_transition:
            next_step = min(self._transition_step + 1, self._transition_steps)
            next_alpha = float(next_step) / float(max(1, self._transition_steps))
            next_y = (1.0 - next_alpha) * self._transition_from_y + next_alpha * self._transition_to_y
            # 保持过渡时使用的角度不变，保证只在 Y 方向慢速移动
            transition_angle = self._transition_angle if self._transition_angle is not None else angle
            (nx_pos, ny_pos, nz_pos), _ = self._compute_point(transition_angle, next_y)
        else:
            next_angle_idx = (angle_idx + 1) % len(self.angles)
            next_angle = self.angles[next_angle_idx]
            (nx_pos, ny_pos, nz_pos), _ = self._compute_point(next_angle, layer_y)

        vx = (nx_pos - x) / dt
        vy = (ny_pos - y) / dt
        vz = (nz_pos - z) / dt

        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.linear.z = float(vz)

        # ========== 构建加速度消息（设为0）==========
        accel = Accel()
        accel.linear.x = 0.0
        accel.linear.y = 0.0
        accel.linear.z = 0.0

        # ========== 发布消息 ==========
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.accel_pub.publish(accel)

        # ========== 更新索引与过渡逻辑 ==========
        if self._in_transition:
            # 处于过渡中，推进过渡步数，过渡完成后切换到下一层实际索引
            self._transition_step += 1
            if self._transition_step >= self._transition_steps:
                # 过渡完成
                self._in_transition = False
                self._transition_step = 0
                self._transition_from_y = None
                self._transition_to_y = None
                self._transition_angle = None
                # 切换到下一层索引（已在过渡开始时未增加）
                self._layer_idx += 1
        else:
            # 正常沿弧运动
            self._angle_idx += 1
            if self._angle_idx >= len(self.angles):
                # 一层结束，准备进入层间过渡（若 y_transition_duration>0 则平滑过渡）
                next_layer_idx = (self._layer_idx + 1) % len(self.layers)
                next_y = self.layers[next_layer_idx]
                current_y = self.layers[self._layer_idx % len(self.layers)]
                if abs(next_y - current_y) > 1e-9 and self.y_transition_duration > 0.0:
                    # 进入过渡阶段
                    self._in_transition = True
                    self._transition_steps = max(1, int(self.y_transition_duration * self.publish_rate))
                    self._transition_step = 0
                    self._transition_from_y = current_y
                    self._transition_to_y = next_y
                    # 使用当前层结束时的角度作为过渡角度，使平滑移动仅在 Y 方向
                    self._transition_angle = self.angles[-1] if forward else self.angles[0]
                    # 将角度索引重置为0，下一层开始时从0计数
                    self._angle_idx = 0
                else:
                    # 无过渡或无需移动，直接切换到下一层
                    self._angle_idx = 0
                    self._layer_idx += 1

    def destroy_node(self):
        """销毁节点"""
        super().destroy_node()


def main(args=None) -> None:
    """
    主函数：初始化并运行节点
    """
    rclpy.init(args=args)
    node = ScanCylindricalSurfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 用户按下 Ctrl+C 退出
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
