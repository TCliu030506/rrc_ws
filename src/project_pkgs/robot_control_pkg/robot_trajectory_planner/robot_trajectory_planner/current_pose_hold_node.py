import rclpy
from geometry_msgs.msg import Accel, Pose, Twist
from rclpy.node import Node

from robot_trajectory_planner.current_pose_hold_logic import (
    copy_pose,
    latch_initial_pose,
    set_zero_accel,
    set_zero_twist,
    validate_publish_rate,
)


class CurrentPoseHoldNode(Node):
    def __init__(self) -> None:
        super().__init__('current_pose_hold_node')

        self.declare_parameter('current_pose_topic', '/asm_ee_site/pose')
        self.declare_parameter('topic_desired_pose', '/scan/desired_pose')
        self.declare_parameter('topic_desired_twist', '/scan/desired_twist')
        self.declare_parameter('topic_desired_accel', '/scan/desired_accel')
        self.declare_parameter('publish_rate', 20.0)

        current_pose_topic = str(self.get_parameter('current_pose_topic').value)
        desired_pose_topic = str(self.get_parameter('topic_desired_pose').value)
        desired_twist_topic = str(self.get_parameter('topic_desired_twist').value)
        desired_accel_topic = str(self.get_parameter('topic_desired_accel').value)
        publish_rate = validate_publish_rate(
            self.get_parameter('publish_rate').value
        )

        self._hold_pose: Pose | None = None

        self.pose_pub = self.create_publisher(Pose, desired_pose_topic, 10)
        self.twist_pub = self.create_publisher(Twist, desired_twist_topic, 10)
        self.accel_pub = self.create_publisher(Accel, desired_accel_topic, 10)
        self.pose_sub = self.create_subscription(
            Pose,
            current_pose_topic,
            self._on_current_pose,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            'Current-pose hold test trajectory started. '
            'The first received pose will be latched: '
            f'{current_pose_topic} -> {desired_pose_topic}, '
            f'{desired_twist_topic}, {desired_accel_topic}'
        )

    def _on_current_pose(self, msg: Pose) -> None:
        previous_pose = self._hold_pose
        self._hold_pose = latch_initial_pose(self._hold_pose, msg, Pose)
        if previous_pose is None and self._hold_pose is not None:
            self.get_logger().info('Initial EE pose latched as hold target.')

    def _on_timer(self) -> None:
        if self._hold_pose is None:
            self.get_logger().info(
                'Waiting for current EE pose...',
                throttle_duration_sec=2.0,
            )
            return

        pose = Pose()
        twist = Twist()
        accel = Accel()
        copy_pose(self._hold_pose, pose)
        set_zero_twist(twist)
        set_zero_accel(accel)

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.accel_pub.publish(accel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentPoseHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
