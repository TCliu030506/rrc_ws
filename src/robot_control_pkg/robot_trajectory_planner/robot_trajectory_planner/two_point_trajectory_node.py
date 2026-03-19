import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Accel


class TwoPointTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('two_point_trajectory_node')

        self.declare_parameter('topic_desired_pose', '/desired_pose')
        self.declare_parameter('topic_desired_twist', '/desired_twist')
        self.declare_parameter('topic_desired_accel', '/desired_accel')

        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('cycle_period', 8.0)

        self.declare_parameter('point_a', [0.360, 0.100, 0.450])
        self.declare_parameter('point_b', [0.360, -0.200, 0.450])
        self.declare_parameter('orientation_xyzw', [0.0, 0.0, 0.0, 1.0])

        topic_pose = self.get_parameter('topic_desired_pose').value
        topic_twist = self.get_parameter('topic_desired_twist').value
        topic_accel = self.get_parameter('topic_desired_accel').value

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cycle_period = float(self.get_parameter('cycle_period').value)

        point_a = self.get_parameter('point_a').value
        point_b = self.get_parameter('point_b').value
        orientation = self.get_parameter('orientation_xyzw').value

        if len(point_a) != 3 or len(point_b) != 3:
            raise ValueError('point_a and point_b must be 3D vectors')
        if len(orientation) != 4:
            raise ValueError('orientation_xyzw must be length 4')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        if self.cycle_period <= 0.0:
            raise ValueError('cycle_period must be > 0')

        self.a = [float(v) for v in point_a]
        self.b = [float(v) for v in point_b]
        self.delta = [self.b[i] - self.a[i] for i in range(3)]
        self.q = [float(v) for v in orientation]

        self.pose_pub = self.create_publisher(Pose, topic_pose, 10)
        self.twist_pub = self.create_publisher(Twist, topic_twist, 10)
        self.accel_pub = self.create_publisher(Accel, topic_accel, 10)

        self.t0 = time.monotonic()
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            'Two-point trajectory planner started. Publishing to: '
            f'{topic_pose}, {topic_twist}, {topic_accel}'
        )

    def on_timer(self) -> None:
        t = time.monotonic() - self.t0

        # s in [0, 1], smoothly moving A->B->A in one cycle.
        omega = 2.0 * math.pi / self.cycle_period
        theta = omega * t
        s = 0.5 * (1.0 - math.cos(theta))
        ds = 0.5 * math.sin(theta) * omega
        dds = 0.5 * math.cos(theta) * omega * omega

        pose = Pose()
        twist = Twist()
        accel = Accel()

        pose.position.x = self.a[0] + s * self.delta[0]
        pose.position.y = self.a[1] + s * self.delta[1]
        pose.position.z = self.a[2] + s * self.delta[2]

        pose.orientation.x = self.q[0]
        pose.orientation.y = self.q[1]
        pose.orientation.z = self.q[2]
        pose.orientation.w = self.q[3]

        twist.linear.x = ds * self.delta[0]
        twist.linear.y = ds * self.delta[1]
        twist.linear.z = ds * self.delta[2]
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        accel.linear.x = dds * self.delta[0]
        accel.linear.y = dds * self.delta[1]
        accel.linear.z = dds * self.delta[2]
        accel.angular.x = 0.0
        accel.angular.y = 0.0
        accel.angular.z = 0.0

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.accel_pub.publish(accel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwoPointTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
