#!/usr/bin/env python3

import argparse
import math
import select
import sys
import termios
import tty
from copy import deepcopy
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from asm_description_mujoco.qos_profiles import make_control_qos


JOINT_NAMES: List[str] = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

JOINT_LIMITS = [
    (-math.pi*2, math.pi*2),
    (-math.pi*2, math.pi*2),
    (-math.pi*2, math.pi*2),
    (-math.pi*2, math.pi*2),
    (-math.pi*2, math.pi*2),
    (-math.pi*2, math.pi*2),
]

JOINT_STEPS = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
TRANSLATION_STEP = 0.01
ROTATION_STEP = math.radians(5.0)

HELP_TEXT = {
    'joint': '''
Keyboard teleop for ASM robot - joint mode
------------------------------------------
q / a : shoulder_pan      + / -
w / s : shoulder_lift     + / -
e / d : elbow             + / -
r / f : wrist_1           + / -
t / g : wrist_2           + / -
y / h : wrist_3           + / -
space : send current targets
p     : print this help
x     : quit
''',
    'cartesian': '''
Keyboard teleop for ASM robot - cartesian mode
----------------------------------------------
w / s : x      + / -
a / d : y      + / -
r / f : z      + / -
u / j : roll   + / -
i / k : pitch  + / -
o / l : yaw    + / -
space : send current target pose
p     : print this help
x     : quit
''',
}


class KeyboardTeleop(Node):
    def __init__(self, mode: str) -> None:
        super().__init__('asm_keyboard_teleop')
        self._mode = mode

        control_qos = make_control_qos(10)
        self._joint_pub = self.create_publisher(JointState, '/command_joint', control_qos)
        self._cartesian_pub = self.create_publisher(PoseStamped, '/cartesian_target_pose', control_qos)

        self._latest_joint_positions: Dict[str, float] = {}
        self._joint_targets = [0.0] * len(JOINT_NAMES)
        self._joints_ready = False

        self._target_pose: Optional[PoseStamped] = None
        self._pose_ready = False

        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/end_effector_pose', self._pose_cb, qos_profile_sensor_data)

    def _joint_states_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._latest_joint_positions[name] = float(position)

        if self._mode != 'joint' or self._joints_ready:
            return

        if all(name in self._latest_joint_positions for name in JOINT_NAMES):
            self._joint_targets = [self._latest_joint_positions[name] for name in JOINT_NAMES]
            self._joints_ready = True
            self.get_logger().info('Joint targets initialized from /joint_states.')

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self._mode != 'cartesian' or self._pose_ready:
            return

        self._target_pose = deepcopy(msg)
        self._pose_ready = True
        self.get_logger().info('Cartesian target initialized from /end_effector_pose.')

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
        w1, x1, y1, z1 = left
        w2, x2, y2, z2 = right
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _axis_angle_to_quat(axis: np.ndarray, angle: float) -> np.ndarray:
        axis = np.asarray(axis, dtype=np.float64)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12 or abs(angle) < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        axis = axis / norm
        half = 0.5 * angle
        sin_half = math.sin(half)
        return np.array([math.cos(half), *(axis * sin_half)], dtype=np.float64)

    def _publish_joint_targets(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(self._joint_targets)
        self._joint_pub.publish(msg)

    def _publish_pose_target(self) -> None:
        if self._target_pose is None:
            return
        msg = deepcopy(self._target_pose)
        msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = 'world'
        self._cartesian_pub.publish(msg)

    def _update_joint_target(self, index: int, delta: float) -> None:
        low, high = JOINT_LIMITS[index]
        self._joint_targets[index] = self._clamp(
            self._joint_targets[index] + delta,
            low,
            high,
        )
        self._publish_joint_targets()

    def _update_cartesian_target(
        self,
        translation: Optional[np.ndarray] = None,
        rotation_axis: Optional[np.ndarray] = None,
        rotation_delta: float = 0.0,
    ) -> None:
        if self._target_pose is None:
            self.get_logger().warn('No /end_effector_pose received yet.')
            return

        if translation is not None:
            self._target_pose.pose.position.x += float(translation[0])
            self._target_pose.pose.position.y += float(translation[1])
            self._target_pose.pose.position.z += float(translation[2])

        if rotation_axis is not None and abs(rotation_delta) > 1e-12:
            current = np.array(
                [
                    self._target_pose.pose.orientation.w,
                    self._target_pose.pose.orientation.x,
                    self._target_pose.pose.orientation.y,
                    self._target_pose.pose.orientation.z,
                ],
                dtype=np.float64,
            )
            delta_quat = self._axis_angle_to_quat(rotation_axis, rotation_delta)
            updated = self._quat_multiply(delta_quat, current)
            updated = updated / max(float(np.linalg.norm(updated)), 1e-12)
            self._target_pose.pose.orientation.w = float(updated[0])
            self._target_pose.pose.orientation.x = float(updated[1])
            self._target_pose.pose.orientation.y = float(updated[2])
            self._target_pose.pose.orientation.z = float(updated[3])

        self._publish_pose_target()

    def _handle_key_joint_mode(self, key: str) -> bool:
        if key == 'q':
            self._update_joint_target(0, +JOINT_STEPS[0])
        elif key == 'a':
            self._update_joint_target(0, -JOINT_STEPS[0])
        elif key == 'w':
            self._update_joint_target(1, +JOINT_STEPS[1])
        elif key == 's':
            self._update_joint_target(1, -JOINT_STEPS[1])
        elif key == 'e':
            self._update_joint_target(2, +JOINT_STEPS[2])
        elif key == 'd':
            self._update_joint_target(2, -JOINT_STEPS[2])
        elif key == 'r':
            self._update_joint_target(3, +JOINT_STEPS[3])
        elif key == 'f':
            self._update_joint_target(3, -JOINT_STEPS[3])
        elif key == 't':
            self._update_joint_target(4, +JOINT_STEPS[4])
        elif key == 'g':
            self._update_joint_target(4, -JOINT_STEPS[4])
        elif key == 'y':
            self._update_joint_target(5, +JOINT_STEPS[5])
        elif key == 'h':
            self._update_joint_target(5, -JOINT_STEPS[5])
        elif key == ' ':
            self._publish_joint_targets()
        elif key == 'p':
            print(HELP_TEXT['joint'])
        elif key == 'x':
            return False
        return True

    def _handle_key_cartesian_mode(self, key: str) -> bool:
        if key == 'w':
            self._update_cartesian_target(translation=np.array([+TRANSLATION_STEP, 0.0, 0.0]))
        elif key == 's':
            self._update_cartesian_target(translation=np.array([-TRANSLATION_STEP, 0.0, 0.0]))
        elif key == 'a':
            self._update_cartesian_target(translation=np.array([0.0, +TRANSLATION_STEP, 0.0]))
        elif key == 'd':
            self._update_cartesian_target(translation=np.array([0.0, -TRANSLATION_STEP, 0.0]))
        elif key == 'r':
            self._update_cartesian_target(translation=np.array([0.0, 0.0, +TRANSLATION_STEP]))
        elif key == 'f':
            self._update_cartesian_target(translation=np.array([0.0, 0.0, -TRANSLATION_STEP]))
        elif key == 'u':
            self._update_cartesian_target(rotation_axis=np.array([1.0, 0.0, 0.0]), rotation_delta=+ROTATION_STEP)
        elif key == 'j':
            self._update_cartesian_target(rotation_axis=np.array([1.0, 0.0, 0.0]), rotation_delta=-ROTATION_STEP)
        elif key == 'i':
            self._update_cartesian_target(rotation_axis=np.array([0.0, 1.0, 0.0]), rotation_delta=+ROTATION_STEP)
        elif key == 'k':
            self._update_cartesian_target(rotation_axis=np.array([0.0, 1.0, 0.0]), rotation_delta=-ROTATION_STEP)
        elif key == 'o':
            self._update_cartesian_target(rotation_axis=np.array([0.0, 0.0, 1.0]), rotation_delta=+ROTATION_STEP)
        elif key == 'l':
            self._update_cartesian_target(rotation_axis=np.array([0.0, 0.0, 1.0]), rotation_delta=-ROTATION_STEP)
        elif key == ' ':
            self._publish_pose_target()
        elif key == 'p':
            print(HELP_TEXT['cartesian'])
        elif key == 'x':
            return False
        return True

    def wait_for_initial_state(self, timeout_sec: float = 5.0) -> bool:
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if self._mode == 'joint' and self._joints_ready:
                self._publish_joint_targets()
                return True
            if self._mode == 'cartesian' and self._pose_ready:
                self._publish_pose_target()
                return True
            if elapsed > timeout_sec:
                break

        if self._mode == 'joint' and not self._joints_ready:
            self.get_logger().warn('Joint targets were not initialized from /joint_states in time; aborting control.')
        if self._mode == 'cartesian' and not self._pose_ready:
            self.get_logger().warn('No /end_effector_pose received in time; aborting control.')
        return False

    def spin_keyboard(self) -> None:
        print(HELP_TEXT[self._mode])
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            running = True
            while running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                key = get_key(0.02)
                if not key:
                    continue
                if self._mode == 'joint':
                    running = self._handle_key_joint_mode(key)
                else:
                    running = self._handle_key_cartesian_mode(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def get_key(timeout: float = 0.1) -> str:
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ''


def main(args=None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--mode', choices=['joint', 'cartesian'], default='joint')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = KeyboardTeleop(parsed_args.mode)
    try:
        if not node.wait_for_initial_state(timeout_sec=5.0):
            return
        node.spin_keyboard()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
