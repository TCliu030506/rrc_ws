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

import math
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Accel, Pose, Twist
from rclpy.node import Node
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
        self.declare_parameter('initial_blend_duration', 2.0)

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
        self.fixed_orientation = self.get_parameter('orientation_xyzw').value
        self.contact_offset = float(self.get_parameter('contact_offset').value)

        self.initial_blend_duration = float(self.get_parameter('initial_blend_duration').value)

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
        self._initial_position_set = False             # 初始位置设置标志
        self._start_pose = None                        # 起始位姿

        self._layer_idx = 0                            # 当前层索引
        self._angle_idx = 0                            # 当前角度索引

        # ========== 创建定时器 ==========
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        # 输出启动信息
        self.get_logger().info(
            f'圆柱形表面扫描启动！'
            f'层数={len(self.layers)}, 每圆弧点数={self.arc_points}, '
            f'话题=({topic_pose}, {topic_twist}, {topic_accel})'
        )

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

    def _on_timer(self) -> None:
        """
        定时器回调函数：生成并发布轨迹数据
        """
        # 计算运行时间
        t = time.monotonic() - self._start_time
        
        # 初始过渡阶段：等待 initial_blend_duration 后再开始扫描
        if t < self.initial_blend_duration and not self._initial_position_set:
            self._initial_position_set = True
            self.get_logger().info(f'等待 {self.initial_blend_duration} 秒后开始扫描...')
            return

        # 如果没有层数据，直接返回
        if not self.layers:
            return

        # ========== 计算当前位置 ==========
        # 当前层索引（循环）
        layer_idx = self._layer_idx % len(self.layers)
        layer_y = self.layers[layer_idx]

        # 相邻层交替方向（往返扫描）
        forward = (layer_idx % 2) == 0
        if forward:
            angle_idx = self._angle_idx % len(self.angles)
        else:
            angle_idx = (len(self.angles) - 1) - (self._angle_idx % len(self.angles))

        # 获取当前角度并计算位置
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
        # 向前看一个点计算速度
        next_angle_idx = (angle_idx + 1) % len(self.angles)
        next_angle = self.angles[next_angle_idx]
        (nx_pos, ny_pos, nz_pos), _ = self._compute_point(next_angle, layer_y)
        
        dt = 1.0 / self.publish_rate  # 时间间隔
        vx = (nx_pos - x) / dt         # X方向速度
        vy = 0.0                       # Y方向速度为0（沿圆弧运动）
        vz = (nz_pos - z) / dt         # Z方向速度

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

        # ========== 更新索引 ==========
        self._angle_idx += 1
        if self._angle_idx >= len(self.angles):
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