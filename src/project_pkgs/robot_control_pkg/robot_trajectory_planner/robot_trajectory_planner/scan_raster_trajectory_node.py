import math
import time

import rclpy
from geometry_msgs.msg import Accel, Pose, Twist ,PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node


class ScanRasterTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_raster_trajectory_node')

        self.declare_parameter('topic_desired_pose', '/scan/desired_pose')
        self.declare_parameter('topic_desired_twist', '/scan/desired_twist')
        self.declare_parameter('topic_desired_accel', '/scan/desired_accel')

        self.declare_parameter('publish_rate', 50.0)
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
        # 当前末端位姿订阅
        self.declare_parameter('current_pose_topic', '/end_effector_pose')

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

        self.current_pose_topic = str(self.get_parameter('current_pose_topic').value)

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

        # desired_* 是控制参考，使用默认 reliable QoS 以匹配导纳控制器订阅端
        self.pose_pub = self.create_publisher(Pose, topic_pose, 10)
        self.twist_pub = self.create_publisher(Twist, topic_twist, 10)
        self.accel_pub = self.create_publisher(Accel, topic_accel, 10)

        # 订阅当前末端位置，用于等待机器人达到 home 并稳定
        self.current_pose = None
        self.pose_sub = self.create_subscription(PoseStamped, self.current_pose_topic, self._pose_callback, qos_profile=qos_profile_sensor_data)

        # 初始位置跟踪变量
        self.initial_position_set = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0

        # t0 在确认初始位置并准备开始过渡后设置
        self.t0 = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            'Scan raster trajectory started. '
            f'lines={self.line_count}, spacing={self.actual_spacing:.4f}, '
            f'topics=({topic_pose}, {topic_twist}, {topic_accel}), '
            f'current_pose_topic={self.current_pose_topic}'
        )

    def _on_timer(self) -> None:
        # 正常运行阶段
        if self.t0 is None:
            # 若未初始化 t0，则设置为当前时间以启动过渡
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0

        # 初始过渡阶段：从当前位置移动到扫描起始位置
        if t < self.initial_blend_duration:
            # 第一次进入时记录当前位置（如果之前未记录）
            if not self.initial_position_set:
                # 若在启动时未等待 home 或未收到位姿，则回退到扫描起始点
                if self.current_pose is not None:
                    self.start_x = self.current_pose.pose.position.x
                    self.start_y = self.current_pose.pose.position.y
                    self.start_z = self.current_pose.pose.position.z
                else:
                    self.start_x = self.x_min
                    self.start_y = self.y_min
                    self.start_z = self.cz

                # 根据距离动态调整过渡时长，确保不会产生超速命令
                dist = math.sqrt((self.start_x - self.x_min) ** 2 + (self.start_y - self.y_min) ** 2 + (self.start_z - self.cz) ** 2)
                required_duration = dist / max(self.initial_blend_speed, 1e-6)
                if required_duration > self.initial_blend_duration:
                    old = self.initial_blend_duration
                    self.initial_blend_duration = required_duration * 1.2
                    self.get_logger().info(f'Extended initial_blend_duration from {old:.2f}s to {self.initial_blend_duration:.2f}s to limit speed.')

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

            # 精确计算位置导数：smooth_ratio' = (pi/(2*T)) * sin(pi * blend_ratio)
            T = self.initial_blend_duration
            smooth_dot = (math.pi / (2.0 * T)) * math.sin(math.pi * blend_ratio)
            vx = (target_x - self.start_x) * smooth_dot
            vy = (target_y - self.start_y) * smooth_dot
            vz = (target_z - self.start_z) * smooth_dot

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

        desired_pose = Pose()
        desired_pose.position.x = x
        desired_pose.position.y = y
        desired_pose.position.z = z
        desired_pose.orientation.x = self.qx
        desired_pose.orientation.y = self.qy
        desired_pose.orientation.z = self.qz
        desired_pose.orientation.w = self.qw

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

        self.pose_pub.publish(desired_pose)
        self.twist_pub.publish(twist)
        self.accel_pub.publish(accel)

    def _pose_callback(self, msg: PoseStamped) -> None:
        # 存储当前位姿
        self.current_pose = msg


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