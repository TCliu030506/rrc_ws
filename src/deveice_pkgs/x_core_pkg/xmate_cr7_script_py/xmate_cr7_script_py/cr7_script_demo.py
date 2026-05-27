from __future__ import annotations

import math
import time
from typing import Sequence, Tuple

import rclpy
from rclpy.node import Node

from xmate_cr7_script_py.cr7_script import Cr7ScriptClient


class Cr7ScriptDemo(Node):
    def __init__(self) -> None:
        super().__init__('cr7_script_demo_py')
        # 透传 service_name 给内部客户端
        self.declare_parameter('service_name', 'cr7_script')
        # 控制可选 demo 段落
        self.declare_parameter('enable_follow_demo', False)
        self.declare_parameter('enable_rt_joint_demo', False)
        self.declare_parameter('enable_rt_cart_demo', False)
        self.declare_parameter('enable_read_and_status_demo', False)
        self.declare_parameter('enable_kinematics_demo', True)

        self.client = Cr7ScriptClient()

    def _sleep_ms(self, ms: int) -> None:
        time.sleep(ms / 1000.0)

    def run(self) -> None:
        # 示例1：movej x 2
        joint_angles1 = [math.pi/2, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
        ok, res = self.client.movej(joint_angles1, 100, 30)
        self.get_logger().info(f"movej result: {res}")
        self._sleep_ms(3000)

        joint_angles2 = [math.pi/2, 0.0, math.pi/2, math.pi/6, math.pi/2, 0.0]
        ok, res = self.client.movej(joint_angles2, 100, 30)
        self.get_logger().info(f"movej result: {res}")

        # 示例2：movep x 2
        pose1 = [0.200, -0.356, 0.724, -150.0*math.pi/180.0, 0.0, -85.0*math.pi/180.0]
        ok, res = self.client.movep(pose1, 100, 30)
        self.get_logger().info(f"movep result: {res}")

        pose2 = [0.150, -0.360, 0.660, -math.pi, 0.0, -math.pi/2]
        ok, res = self.client.movep(pose2, 100, 30)
        self.get_logger().info(f"movep result: {res}")
        self._sleep_ms(2000)

        # 示例3：stop
        ok, res = self.client.stop()
        self.get_logger().info(f"stop result: {res}")
        self._sleep_ms(2000)

        # 示例4：movel x 2
        posel1 = [0.200, -0.356, 0.724, -150.0*math.pi/180.0, 0.0, -85.0*math.pi/180.0]
        ok, res = self.client.movel(posel1, 100, 30)
        self.get_logger().info(f"movel result: {res}")

        posel2 = [0.150, -0.360, 0.660, -math.pi, 0.0, -math.pi/2]
        ok, res = self.client.movel(posel2, 100, 30)
        self.get_logger().info(f"movel result: {res}")

        # 可选：示例5 follow loop（默认关闭）
        if self.get_parameter('enable_follow_demo').get_parameter_value().bool_value:
            ok, res = self.client.follow_pos_start()
            self.get_logger().info(f"follow_pos_start result: {res}")

            joint_update = [math.pi/2, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
            for _ in range(1000):
                self._sleep_ms(10)
                joint_update[4] += math.pi*6/10000.0
                ok, res = self.client.follow_pos_update(joint_update, 400)

            for _ in range(1000):
                self._sleep_ms(10)
                joint_update[4] -= math.pi*6/10000.0
                ok, res = self.client.follow_pos_update(joint_update, 400)

            ok, res = self.client.follow_pos_stop()
            self.get_logger().info(f"follow_pos_stop result: {res}")

        # 可选：示例6/7 实时控制（默认关闭）
        if self.get_parameter('enable_rt_joint_demo').get_parameter_value().bool_value:
            ok, res = self.client.open_rtControl_loop('jointPosition')
            self.get_logger().info(f"open_rtControl_loop result: {res}")
            self._sleep_ms(5000)

            joint_update = [math.pi/2, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
            for _ in range(10000):
                self._sleep_ms(1)
                joint_update[4] += math.pi*6/100000.0
                ok, res = self.client.rtControl_jointpos_update(joint_update)

            for _ in range(10000):
                self._sleep_ms(1)
                joint_update[4] -= math.pi*6/100000.0
                ok, res = self.client.rtControl_jointpos_update(joint_update)

        if self.get_parameter('enable_rt_cart_demo').get_parameter_value().bool_value:
            ok, res = self.client.open_rtControl_loop('cartesianPosition')
            self.get_logger().info(f"open_rtControl_loop (cartesianPosition) result: {res}")
            self._sleep_ms(5000)

            posel_update = [0.150, -0.360, 0.660, -math.pi, 0.0, -math.pi/2]
            for _ in range(10000):
                self._sleep_ms(1)
                posel_update[1] += 0.2/10000.0
                ok, res = self.client.rtControl_cartesianpos_update(posel_update)

            for _ in range(10000):
                self._sleep_ms(1)
                posel_update[1] -= 0.2/10000.0
                ok, res = self.client.rtControl_cartesianpos_update(posel_update)

        # 可选：示例8~16 状态与标定（默认关闭）
        if self.get_parameter('enable_read_and_status_demo').get_parameter_value().bool_value:
            ok, res = self.client.readp("world"); self.get_logger().info(f"readp result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.readj(); self.get_logger().info(f"readj result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.get_vel(); self.get_logger().info(f"get_vel result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.get_joint_vel(); self.get_logger().info(f"get_joint_vel result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.calibrateForceSensor(True, 0); self.get_logger().info(f"calibrateForceSensor result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.getEndTorque('world'); self.get_logger().info(f"getEndTorque result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.get_joint_torque(); self.get_logger().info(f"get_joint_torque result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.get_acc(); self.get_logger().info(f"get_acc result: {res}"); self._sleep_ms(1000)
            ok, res = self.client.get_jerk(); self.get_logger().info(f"get_jerk result: {res}"); self._sleep_ms(1000)

        # 可选：示例17/18 运动学（默认关闭）
        if self.get_parameter('enable_kinematics_demo').get_parameter_value().bool_value:
            inv_pos = [0.15, -0.35, 0.66, math.pi, 0.0, -math.pi/2]
            ok, res = self.client.inv_kinematics(inv_pos)
            self.get_logger().info(f"inv_kinematics result: {res}")
            self._sleep_ms(1000)

            joint_for_fk = [math.pi/2, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
            ok, res = self.client.forward_kinematics(joint_for_fk)
            self.get_logger().info(f"forward_kinematics result: {res}")
            self._sleep_ms(1000)


def main() -> None:
    rclpy.init()
    node = Cr7ScriptDemo()
    try:
        node.run()
    finally:
        node.client.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
