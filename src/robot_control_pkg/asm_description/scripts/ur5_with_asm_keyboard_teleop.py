#!/usr/bin/env python3

import math
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
Keyboard teleop for UR5 + ASM (ur5_with_asm_gazebo.launch.py)
-------------------------------------------------------------
UR5 joints (joint_trajectory_controller):
q/a : shoulder_pan_joint   + / -
w/s : shoulder_lift_joint  + / -
e/d : elbow_joint          + / -
r/f : wrist_1_joint        + / -
t/g : wrist_2_joint        + / -
y/h : wrist_3_joint        + / -

ASM joints (asm_arm_controller):
u/j : asm_tool_joint1      + / -
i/k : asm_tool_joint2      + / -

space: send home (all joints -> 0.0)
p    : print this help
x    : quit

Each keypress sends a trajectory goal (0.2 s) to the relevant controller.
"""


class Ur5WithAsmKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('ur5_with_asm_keyboard_teleop')

        self._ur_joint_names: List[str] = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        self._asm_joint_names: List[str] = ['asm_tool_joint1', 'asm_tool_joint2']

        # Conservative limits for interactive testing.
        self._ur_joint_limits = [(-math.pi, math.pi)] * 6
        self._asm_joint_limits = [(-0.15, 0.15), (-0.30, 0.30)]

        self._ur_step = 0.05
        self._asm_step = 0.02
        self._goal_time_sec = 0.2

        self._ur_targets = [0.0] * len(self._ur_joint_names)
        self._asm_targets = [0.0] * len(self._asm_joint_names)

        self._ur_action_candidates = [
            '/joint_trajectory_controller/follow_joint_trajectory',
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
        ]
        self._ur_controller_label = 'ur_controller'

        self._ur_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._ur_action_candidates[0],
        )
        self._asm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/asm_arm_controller/follow_joint_trajectory',
        )

    def wait_for_controllers(self) -> bool:
        self.get_logger().info('Waiting for UR5 controller action...')
        ur_ok = False
        for ur_action in self._ur_action_candidates:
            candidate_client = ActionClient(self, FollowJointTrajectory, ur_action)
            if candidate_client.wait_for_server(timeout_sec=4.0):
                self._ur_client = candidate_client
                self._ur_controller_label = ur_action.split('/')[1]
                ur_ok = True
                self.get_logger().info(f'Using UR action server: {ur_action}')
                break

        if not ur_ok:
            self.get_logger().error(
                'UR5 action server not available. Tried: '
                + ', '.join(self._ur_action_candidates)
            )
            return False

        self.get_logger().info('Waiting for ASM controller action...')
        asm_ok = self._asm_client.wait_for_server(timeout_sec=8.0)
        if not asm_ok:
            self.get_logger().error(
                'ASM action server not available: '
                '/asm_arm_controller/follow_joint_trajectory'
            )
            return False

        self.get_logger().info('Both action servers are ready.')
        return True

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _send_goal(
        self,
        client: ActionClient,
        joint_names: List[str],
        targets: List[float],
        controller_name: str,
    ) -> None:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = list(targets)
        sec = int(self._goal_time_sec)
        nanosec = int((self._goal_time_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

        goal_msg.trajectory.points = [point]

        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future: self._goal_response_callback(future, controller_name, joint_names, targets)
        )

    def _goal_response_callback(self, future, controller_name: str, joint_names: List[str], targets: List[float]) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected by {controller_name}.')
            return

        state_text = ', '.join(f'{name}={value:.3f}' for name, value in zip(joint_names, targets))
        self.get_logger().info(f'{controller_name} goal accepted: {state_text}')

    def update_ur_joint(self, idx: int, delta: float) -> None:
        low, high = self._ur_joint_limits[idx]
        self._ur_targets[idx] = self._clamp(self._ur_targets[idx] + delta, low, high)
        self._send_goal(
            self._ur_client,
            self._ur_joint_names,
            self._ur_targets,
            self._ur_controller_label,
        )

    def update_asm_joint(self, idx: int, delta: float) -> None:
        low, high = self._asm_joint_limits[idx]
        self._asm_targets[idx] = self._clamp(self._asm_targets[idx] + delta, low, high)
        self._send_goal(
            self._asm_client,
            self._asm_joint_names,
            self._asm_targets,
            'asm_arm_controller',
        )

    def go_home(self) -> None:
        self._ur_targets = [0.0] * len(self._ur_joint_names)
        self._asm_targets = [0.0] * len(self._asm_joint_names)
        self._send_goal(
            self._ur_client,
            self._ur_joint_names,
            self._ur_targets,
            self._ur_controller_label,
        )
        self._send_goal(
            self._asm_client,
            self._asm_joint_names,
            self._asm_targets,
            'asm_arm_controller',
        )


def get_key(timeout: float = 0.1) -> str:
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur5WithAsmKeyboardTeleop()

    if not node.wait_for_controllers():
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
                node.update_ur_joint(0, +node._ur_step)
            elif key == 'a':
                node.update_ur_joint(0, -node._ur_step)
            elif key == 'w':
                node.update_ur_joint(1, +node._ur_step)
            elif key == 's':
                node.update_ur_joint(1, -node._ur_step)
            elif key == 'e':
                node.update_ur_joint(2, +node._ur_step)
            elif key == 'd':
                node.update_ur_joint(2, -node._ur_step)
            elif key == 'r':
                node.update_ur_joint(3, +node._ur_step)
            elif key == 'f':
                node.update_ur_joint(3, -node._ur_step)
            elif key == 't':
                node.update_ur_joint(4, +node._ur_step)
            elif key == 'g':
                node.update_ur_joint(4, -node._ur_step)
            elif key == 'y':
                node.update_ur_joint(5, +node._ur_step)
            elif key == 'h':
                node.update_ur_joint(5, -node._ur_step)
            elif key == 'u':
                node.update_asm_joint(0, +node._asm_step)
            elif key == 'j':
                node.update_asm_joint(0, -node._asm_step)
            elif key == 'i':
                node.update_asm_joint(1, +node._asm_step)
            elif key == 'k':
                node.update_asm_joint(1, -node._asm_step)
            elif key == ' ':
                node.go_home()
            elif key == 'p':
                print(HELP_TEXT)
            elif key == 'x':
                running = False

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()