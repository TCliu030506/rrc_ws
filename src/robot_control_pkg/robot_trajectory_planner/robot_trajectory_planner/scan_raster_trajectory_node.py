import math
import time

import rclpy
from geometry_msgs.msg import Accel, Pose, Twist
from rclpy.node import Node


class ScanRasterTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_raster_trajectory_node')

        self.declare_parameter('topic_desired_pose', '/scan/desired_pose')
        self.declare_parameter('topic_desired_twist', '/scan/desired_twist')
        self.declare_parameter('topic_desired_accel', '/scan/desired_accel')

        self.declare_parameter('publish_rate', 125.0)
        self.declare_parameter('scan_center', [0.45, 0.0, 0.45])
        self.declare_parameter('scan_length_x', 0.20)
        self.declare_parameter('scan_width_y', 0.10)
        self.declare_parameter('scan_speed', 0.001)
        self.declare_parameter('line_spacing', 0.01)
        self.declare_parameter('line_turnaround_pause', 0.15)
        self.declare_parameter('orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        # 初始过渡参数
        self.declare_parameter('initial_blend_duration', 5.0)  # 初始过渡时间（秒）
        self.declare_parameter('initial_blend_speed', 0.02)    # 初始过渡速度（m/s）

        topic_pose = str(self.get_parameter('topic_desired_pose').value)
        topic_twist = str(self.get_parameter('topic_desired_twist').value)
        topic_accel = str(self.get_parameter('topic_desired_accel').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        center = self.get_parameter('scan_center').value
        self.length_x = float(self.get_parameter('scan_length_x').value)
        self.width_y = float(self.get_parameter('scan_width_y').value)
        self.scan_speed = float(self.get_parameter('scan_speed').value)
        self.line_spacing = float(self.get_parameter('line_spacing').value)
        self.turn_pause = float(self.get_parameter('line_turnaround_pause').value)
        orientation = self.get_parameter('orientation_xyzw').value
        # 初始过渡参数
        self.initial_blend_duration = float(self.get_parameter('initial_blend_duration').value)
        self.initial_blend_speed = float(self.get_parameter('initial_blend_speed').value)

        if len(center) != 3:
            raise ValueError('scan_center must be length 3')
        if len(orientation) != 4:
            raise ValueError('orientation_xyzw must be length 4')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        if self.length_x <= 0.0 or self.width_y < 0.0:
            raise ValueError('scan_length_x must be > 0 and scan_width_y must be >= 0')
        if self.scan_speed <= 0.0:
            raise ValueError('scan_speed must be > 0')
        if self.line_spacing <= 0.0:
            raise ValueError('line_spacing must be > 0')
        if self.turn_pause < 0.0:
            raise ValueError('line_turnaround_pause must be >= 0')

        self.cx, self.cy, self.cz = [float(v) for v in center]
        self.qx, self.qy, self.qz, self.qw = [float(v) for v in orientation]

        raw_lines = int(math.ceil(self.width_y / self.line_spacing)) + 1
        self.line_count = max(2, raw_lines)
        self.actual_spacing = 0.0 if self.line_count <= 1 else self.width_y / float(self.line_count - 1)

        self.x_min = self.cx - 0.5 * self.length_x
        self.x_max = self.cx + 0.5 * self.length_x
        self.y_min = self.cy - 0.5 * self.width_y
        self.y_max = self.cy + 0.5 * self.width_y

        self.line_duration = self.length_x / self.scan_speed
        self.segment_duration = self.line_duration + self.turn_pause
        self.full_cycle_duration = self.segment_duration * self.line_count

        self.pose_pub = self.create_publisher(Pose, topic_pose, 10)
        self.twist_pub = self.create_publisher(Twist, topic_twist, 10)
        self.accel_pub = self.create_publisher(Accel, topic_accel, 10)

        # 初始位置跟踪变量
        self.initial_position_set = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0

        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            'Scan raster trajectory started. '
            f'lines={self.line_count}, spacing={self.actual_spacing:.4f}, '
            f'topics=({topic_pose}, {topic_twist}, {topic_accel})'
        )

    def _on_timer(self) -> None:
        t = time.monotonic() - self.t0

        # 初始过渡阶段：从当前位置移动到扫描起始位置
        if t < self.initial_blend_duration:
            # 第一次进入时记录当前位置
            if not self.initial_position_set:
                # 使用扫描起始位置作为初始目标
                # 如果能获取当前机器人位置，可以在这里订阅并记录
                # 这里简化处理：使用扫描区域的起始点作为过渡目标
                # 实际应用中可以订阅 /ee_pose 等话题获取当前位置
                self.start_x = self.x_min  # 可替换为实际当前位置
                self.start_y = self.y_min
                self.start_z = self.cz
                self.initial_position_set = True
                self.get_logger().info(
                    f'Starting initial blend from ({self.start_x:.3f}, {self.start_y:.3f}, {self.start_z:.3f}) '
                    f'to scan start position'
                )

            # 计算过渡进度 [0, 1]
            blend_ratio = t / self.initial_blend_duration

            # 使用正弦函数实现平滑加速/减速
            # sin(blend_ratio * pi) 在 0 到 1 之间平滑变化
            smooth_ratio = (1 - math.cos(blend_ratio * math.pi)) / 2

            # 扫描起始位置
            target_x = self.x_min
            target_y = self.y_min
            target_z = self.cz

            # 线性插值位置
            x = self.start_x + (target_x - self.start_x) * smooth_ratio
            y = self.start_y + (target_y - self.start_y) * smooth_ratio
            z = self.start_z + (target_z - self.start_z) * smooth_ratio

            # 计算过渡速度
            dx = (target_x - self.start_x) / self.initial_blend_duration
            dy = (target_y - self.start_y) / self.initial_blend_duration
            dz = (target_z - self.start_z) / self.initial_blend_duration

            # 速度也应用相同的平滑函数
            vx = dx * smooth_ratio * 2  # *2 补偿正弦曲线的平均值
            vy = dy * smooth_ratio * 2
            vz = dz * smooth_ratio * 2

            # 限制最大过渡速度
            vel_norm = math.sqrt(vx**2 + vy**2 + vz**2)
            if vel_norm > self.initial_blend_speed:
                scale = self.initial_blend_speed / vel_norm
                vx *= scale
                vy *= scale
                vz *= scale

        else:
            # 正常扫描阶段
            phase = (t - self.initial_blend_duration) % self.full_cycle_duration

            line_idx = int(phase // self.segment_duration)
            seg_t = phase - float(line_idx) * self.segment_duration

            forward = (line_idx % 2) == 0
            y = self.y_min + float(line_idx) * self.actual_spacing

            vx = 0.0
            vy = 0.0
            vz = 0.0
            x = self.x_max if forward else self.x_min

            if seg_t <= self.line_duration:
                u = seg_t / self.line_duration
                # 添加梯形速度曲线（加速-匀速-减速）
                accel_ratio = 0.1  # 加速/减速阶段占比
                if u < accel_ratio:
                    # 加速阶段
                    speed_factor = u / accel_ratio
                    vx = self.scan_speed * (1 - math.cos(speed_factor * math.pi)) / 2
                elif u > 1 - accel_ratio:
                    # 减速阶段
                    speed_factor = (1 - u) / accel_ratio
                    vx = self.scan_speed * (1 - math.cos(speed_factor * math.pi)) / 2
                else:
                    # 匀速阶段
                    vx = self.scan_speed

                if forward:
                    x = self.x_min + u * self.length_x
                else:
                    x = self.x_max - u * self.length_x
                    vx = -vx

            z = self.cz

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = self.qx
        pose.orientation.y = self.qy
        pose.orientation.z = self.qz
        pose.orientation.w = self.qw

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        accel = Accel()
        accel.linear.x = 0.0
        accel.linear.y = 0.0
        accel.linear.z = 0.0
        accel.angular.x = 0.0
        accel.angular.y = 0.0
        accel.angular.z = 0.0

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.accel_pub.publish(accel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRasterTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()