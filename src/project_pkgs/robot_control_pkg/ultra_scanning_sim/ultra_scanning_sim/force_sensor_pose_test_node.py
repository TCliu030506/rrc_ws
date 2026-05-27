#!/usr/bin/env python3
"""Drive UR5 to several standard tool orientations and print force checks.

This node publishes target poses to /admittance/cmd_pose so that:
- ur5_ik.py solves IK
- ur5_joints_controller.py sends trajectory goal

It also subscribes transformed wrench topics in base_link and reports averages.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, WrenchStamped
from rclpy.node import Node


@dataclass
class PoseTarget:
    name: str
    pos: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]


class ForceSensorPoseTestNode(Node):
    def __init__(self) -> None:
        super().__init__('force_sensor_pose_test_node')

        self.declare_parameter('cmd_pose_topic', '/admittance/cmd_pose')
        self.declare_parameter('wrench_base_topic', '/wrench_base_link')
        self.declare_parameter('wrench_comp_base_topic', '/wrench_compensated_base_link')

        self.declare_parameter('position_x', 0.45)
        self.declare_parameter('position_y', 0.00)
        self.declare_parameter('position_z', 0.45)

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('hold_seconds_per_pose', 4.0)
        self.declare_parameter('start_delay_sec', 8.0)

        self._cmd_pose_topic = str(self.get_parameter('cmd_pose_topic').value)
        self._wrench_base_topic = str(self.get_parameter('wrench_base_topic').value)
        self._wrench_comp_base_topic = str(self.get_parameter('wrench_comp_base_topic').value)

        px = float(self.get_parameter('position_x').value)
        py = float(self.get_parameter('position_y').value)
        pz = float(self.get_parameter('position_z').value)

        self._publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._hold_seconds = max(1.0, float(self.get_parameter('hold_seconds_per_pose').value))
        self._start_delay = max(0.0, float(self.get_parameter('start_delay_sec').value))

        self._pose_pub = self.create_publisher(Pose, self._cmd_pose_topic, 20)
        self._sub_wrench = self.create_subscription(WrenchStamped, self._wrench_base_topic, self._on_wrench, 20)
        self._sub_wrench_comp = self.create_subscription(WrenchStamped, self._wrench_comp_base_topic, self._on_wrench_comp, 20)

        # Tool Z axis orientation test set.
        sq2 = math.sqrt(0.5)
        self._targets: List[PoseTarget] = [
            PoseTarget('tool_up', (px, py, pz), (0.0, 0.0, 0.0, 1.0)),
            PoseTarget('tool_down', (px, py, pz), (1.0, 0.0, 0.0, 0.0)),
            PoseTarget('tool_right', (px, py, pz), (0.0, -sq2, 0.0, sq2)),
            PoseTarget('tool_left', (px, py, pz), (0.0, sq2, 0.0, sq2)),
        ]

        self._phase_index = 0
        self._phase_start_time = self.get_clock().now()
        self._active = False

        self._sum_raw = [0.0, 0.0, 0.0]
        self._cnt_raw = 0
        self._sum_comp = [0.0, 0.0, 0.0]
        self._cnt_comp = 0

        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)
        self._start_timer = self.create_timer(self._start_delay if self._start_delay > 0.0 else 0.001, self._start_once)

        self.get_logger().info(
            'Force pose test node started. '
            f'cmd={self._cmd_pose_topic}, raw={self._wrench_base_topic}, comp={self._wrench_comp_base_topic}, '
            f'hold={self._hold_seconds}s, rate={self._publish_rate_hz}Hz'
        )

    def _start_once(self) -> None:
        if self._active:
            return
        self._active = True
        self._phase_start_time = self.get_clock().now()
        self._reset_accum()
        self.get_logger().info('Starting pose sequence test now.')
        self._start_timer.cancel()

    def _reset_accum(self) -> None:
        self._sum_raw = [0.0, 0.0, 0.0]
        self._cnt_raw = 0
        self._sum_comp = [0.0, 0.0, 0.0]
        self._cnt_comp = 0

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._sum_raw[0] += float(msg.wrench.force.x)
        self._sum_raw[1] += float(msg.wrench.force.y)
        self._sum_raw[2] += float(msg.wrench.force.z)
        self._cnt_raw += 1

    def _on_wrench_comp(self, msg: WrenchStamped) -> None:
        self._sum_comp[0] += float(msg.wrench.force.x)
        self._sum_comp[1] += float(msg.wrench.force.y)
        self._sum_comp[2] += float(msg.wrench.force.z)
        self._cnt_comp += 1

    def _publish_target(self, target: PoseTarget) -> None:
        msg = Pose()
        msg.position.x = target.pos[0]
        msg.position.y = target.pos[1]
        msg.position.z = target.pos[2]
        msg.orientation.x = target.quat_xyzw[0]
        msg.orientation.y = target.quat_xyzw[1]
        msg.orientation.z = target.quat_xyzw[2]
        msg.orientation.w = target.quat_xyzw[3]
        self._pose_pub.publish(msg)

    def _mean_force(self, sums: List[float], cnt: int) -> Optional[Tuple[float, float, float]]:
        if cnt <= 0:
            return None
        return (sums[0] / cnt, sums[1] / cnt, sums[2] / cnt)

    def _fmt_vec(self, v: Optional[Tuple[float, float, float]]) -> str:
        if v is None:
            return 'N/A'
        return f'[{v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f}]'

    def _report_phase(self, pose_name: str) -> None:
        mean_raw = self._mean_force(self._sum_raw, self._cnt_raw)
        mean_comp = self._mean_force(self._sum_comp, self._cnt_comp)
        self.get_logger().info(
            f'Pose {pose_name}: '
            f'raw_mean_Fxyz={self._fmt_vec(mean_raw)} (n={self._cnt_raw}), '
            f'comp_mean_Fxyz={self._fmt_vec(mean_comp)} (n={self._cnt_comp})'
        )

    def _on_timer(self) -> None:
        if not self._active:
            return

        target = self._targets[self._phase_index]
        self._publish_target(target)

        elapsed = (self.get_clock().now() - self._phase_start_time).nanoseconds * 1e-9
        if elapsed < self._hold_seconds:
            return

        self._report_phase(target.name)

        self._phase_index += 1
        if self._phase_index >= len(self._targets):
            self.get_logger().info('Pose sequence test completed.')
            self._active = False
            return

        self._phase_start_time = self.get_clock().now()
        self._reset_accum()
        self.get_logger().info(f'Switching to next pose: {self._targets[self._phase_index].name}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceSensorPoseTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
