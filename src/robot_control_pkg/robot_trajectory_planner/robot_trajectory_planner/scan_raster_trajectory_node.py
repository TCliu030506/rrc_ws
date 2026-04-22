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

        self.line_duration = self.length_x / self.scan_speed
        self.segment_duration = self.line_duration + self.turn_pause
        self.full_cycle_duration = self.segment_duration * self.line_count

        self.pose_pub = self.create_publisher(Pose, topic_pose, 10)
        self.twist_pub = self.create_publisher(Twist, topic_twist, 10)
        self.accel_pub = self.create_publisher(Accel, topic_accel, 10)

        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            'Scan raster trajectory started. '
            f'lines={self.line_count}, spacing={self.actual_spacing:.4f}, '
            f'topics=({topic_pose}, {topic_twist}, {topic_accel})'
        )

    def _on_timer(self) -> None:
        t = time.monotonic() - self.t0
        phase = t % self.full_cycle_duration

        line_idx = int(phase // self.segment_duration)
        seg_t = phase - float(line_idx) * self.segment_duration

        forward = (line_idx % 2) == 0
        y = self.y_min + float(line_idx) * self.actual_spacing

        vx = 0.0
        x = self.x_max if forward else self.x_min

        if seg_t <= self.line_duration:
            u = seg_t / self.line_duration
            if forward:
                x = self.x_min + u * self.length_x
                vx = self.scan_speed
            else:
                x = self.x_max - u * self.length_x
                vx = -self.scan_speed

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = self.cz
        pose.orientation.x = self.qx
        pose.orientation.y = self.qy
        pose.orientation.z = self.qz
        pose.orientation.w = self.qw

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = 0.0
        twist.linear.z = 0.0
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
