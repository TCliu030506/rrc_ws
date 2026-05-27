#!/usr/bin/env python3

import select
import sys
import termios
import tty
from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


HELP_TEXT = """
Keyboard teleop for asm_description arm_controller
-------------------------------------------------
q/a : tool_joint1 + / -
w/s : tool_joint2 + / -
space: send home [0.0, 0.0]
x    : quit

Each keypress sends a new trajectory goal (1.0 s) to /arm_controller/follow_joint_trajectory.
"""


class ArmKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('arm_keyboard_teleop')

        self._joint_names: List[str] = ['tool_joint1', 'tool_joint2']
        self._joint_limits = [(-0.130, 0.130), (-0.270, 0.270)]
        self._step = 0.02
        self._goal_time_sec = 0.01
        self._target_positions = [0.0, 0.0]

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

    def wait_for_controller(self) -> bool:
        self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory ...')
        ok = self._action_client.wait_for_server(timeout_sec=8.0)
        if not ok:
            self.get_logger().error('Action server not available. Is arm_controller active?')
            return False
        self.get_logger().info('Action server is ready.')
        return True

    def clamp(self, idx: int, value: float) -> float:
        low, high = self._joint_limits[idx]
        return max(low, min(high, value))

    def send_goal(self) -> None:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = self._target_positions
        sec = int(self._goal_time_sec)
        nanosec = int((self._goal_time_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        goal_msg.trajectory.points = [point]

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by arm_controller.')
            return
        self.get_logger().info(
            f'Goal accepted: tool_joint1={self._target_positions[0]:.3f}, '
            f'tool_joint2={self._target_positions[1]:.3f}'
        )

    def update_target(self, idx: int, delta: float) -> None:
        self._target_positions[idx] = self.clamp(idx, self._target_positions[idx] + delta)
        self.send_goal()

    def go_home(self) -> None:
        self._target_positions = [0.0, 0.0]
        self.send_goal()


def get_key(timeout: float = 0.1) -> str:
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmKeyboardTeleop()

    if not node.wait_for_controller():
        node.destroy_node()
        rclpy.shutdown()
        return

    print(HELP_TEXT)

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(0.1)

            if key == 'q':
                node.update_target(0, +node._step)
            elif key == 'a':
                node.update_target(0, -node._step)
            elif key == 'w':
                node.update_target(1, +node._step)
            elif key == 's':
                node.update_target(1, -node._step)
            elif key == ' ':
                node.go_home()
            elif key == 'x':
                running = False

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
