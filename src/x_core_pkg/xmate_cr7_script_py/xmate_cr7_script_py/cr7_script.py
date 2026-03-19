from __future__ import annotations

import math
from typing import Tuple, Sequence

import rclpy
from rclpy.node import Node

from coordinate.srv import StringScript


class ScriptCmd:
    """构建与 C++ 版本一致的脚本指令字符串。"""

    @staticmethod
    def _join_params(params: Sequence[Tuple[str, str]]) -> str:
        return ",".join([f"{k}:{v}" for k, v in params])

    @staticmethod
    def build_movej(joint_angles: Sequence[float], speed: int, zone: int) -> str:
        params = [(f"joint{i+1}", str(val)) for i, val in enumerate(joint_angles)]
        params.append(("speed", str(speed)))
        params.append(("zone", str(zone)))
        return f"movej({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_movep(target_pos: Sequence[float], speed: int, zone: int) -> str:
        names = ("x", "y", "z", "rx", "ry", "rz")
        params = [(names[i], str(target_pos[i])) for i in range(6)]
        params.append(("speed", str(speed)))
        params.append(("zone", str(zone)))
        return f"movep({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_movel(target_pos: Sequence[float], speed: int, zone: int) -> str:
        names = ("x", "y", "z", "rx", "ry", "rz")
        params = [(names[i], str(target_pos[i])) for i in range(6)]
        params.append(("speed", str(speed)))
        params.append(("zone", str(zone)))
        return f"movel({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_stop() -> str:
        return "stop()"

    @staticmethod
    def build_rtControl_jointpos_update(joint_angles: Sequence[float]) -> str:
        params = [(f"joint{i+1}", str(joint_angles[i])) for i in range(6)]
        params.append(("mode", "joint"))
        return f"rtControl_jointpos_update({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_rtControl_cartesianpos_update(cartesian_pos: Sequence[float]) -> str:
        names = ("x", "y", "z", "rx", "ry", "rz")
        params = [(names[i], str(cartesian_pos[i])) for i in range(6)]
        params.append(("mode", "cartesian"))
        return f"rtControl_cartesianpos_update({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_follow_pos_update(joint_angles: Sequence[float], speed: int) -> str:
        params = [(f"joint{i+1}", str(joint_angles[i])) for i in range(6)]
        params.append(("speed", str(speed)))
        return f"follow_pos_update({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_set_follow_speed(speed: int) -> str:
        return f"set_follow_speed(speed:{speed})"

    @staticmethod
    def build_calibrateForceSensor(all_axes: bool, axis_index: int) -> str:
        axes = "true" if all_axes else "false"
        return f"calibrateForceSensor(all_axes:{axes},axis_index:{axis_index})"

    @staticmethod
    def build_getEndTorque(ref_type: str) -> str:
        return f"getEndTorque(ref_type:{ref_type})"
    
    def build_readp(ref_type: str) -> str:
        return f"readp(ref_type:{ref_type})"

    @staticmethod
    def build_inv_kinematics(Pos: Sequence[float]) -> str:
        names = ("x", "y", "z", "rx", "ry", "rz")
        params = [(names[i], str(Pos[i])) for i in range(6)]
        return f"inv_kinematics({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_forward_kinematics(jntPos: Sequence[float]) -> str:
        params = [(f"joint{i+1}", str(jntPos[i])) for i in range(6)]
        return f"forward_kinematics({ScriptCmd._join_params(params)})"

    @staticmethod
    def build_open_rtControl_loop(rt_type: str) -> str:
        return f"open_rtControl_loop(rt_type:{rt_type})"


class Cr7ScriptClient(Node):
    """与 coordinate/srv/StringScript 服务通信的客户端节点。"""

    def __init__(self) -> None:
        super().__init__('cr7_script_client_py')
        # 服务名可参数化，默认与 C++ 版一致
        self.declare_parameter('service_name', 'cr7_script')
        service_name = self.get_parameter('service_name').get_parameter_value().string_value

        self._client = self.create_client(StringScript, service_name)
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name} service...')
        self.get_logger().info(f'Connected to {service_name} service.')

    # 基础发送接口
    def send_script(self, command: str) -> Tuple[bool, str]:
        req = StringScript.Request()
        req.command = command
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.done() and future.result() is not None:
            return True, future.result().result
        else:
            return False, 'Service call failed'

    # 快捷接口：构造并发送
    def movej(self, joint_angles: Sequence[float], speed: int, zone: int) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_movej(joint_angles, speed, zone))

    def movep(self, target_pos: Sequence[float], speed: int, zone: int) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_movep(target_pos, speed, zone))

    def movel(self, target_pos: Sequence[float], speed: int, zone: int) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_movel(target_pos, speed, zone))

    def stop(self) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_stop())

    def open_rtControl_loop(self, rt_type: str) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_open_rtControl_loop(rt_type))

    def rtControl_jointpos_update(self, joint_angles: Sequence[float]) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_rtControl_jointpos_update(joint_angles))

    def rtControl_cartesianpos_update(self, cartesian_pos: Sequence[float]) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_rtControl_cartesianpos_update(cartesian_pos))

    def follow_pos_start(self) -> Tuple[bool, str]:
        return self.send_script('follow_pos_start()')

    def follow_pos_stop(self) -> Tuple[bool, str]:
        return self.send_script('follow_pos_stop()')

    def follow_pos_update(self, joint_angles: Sequence[float], speed: int) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_follow_pos_update(joint_angles, speed))

    def set_follow_speed(self, speed: int) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_set_follow_speed(speed))

    def calibrateForceSensor(self, all_axes: bool, axis_index: int) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_calibrateForceSensor(all_axes, axis_index))

    def readp(self, ref_type: str) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_readp(ref_type))

    def readj(self) -> Tuple[bool, str]:
        return self.send_script('readj()')

    def get_vel(self) -> Tuple[bool, str]:
        return self.send_script('get_vel()')

    def get_joint_vel(self) -> Tuple[bool, str]:
        return self.send_script('get_joint_vel()')

    def get_acc(self) -> Tuple[bool, str]:
        return self.send_script('get_acc()')

    def get_jerk(self) -> Tuple[bool, str]:
        return self.send_script('get_jerk()')

    def get_joint_torque(self) -> Tuple[bool, str]:
        return self.send_script('get_joint_torque()')

    def getEndTorque(self, ref_type: str) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_getEndTorque(ref_type))

    def inv_kinematics(self, Pos: Sequence[float]) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_inv_kinematics(Pos))

    def forward_kinematics(self, jntPos: Sequence[float]) -> Tuple[bool, str]:
        return self.send_script(ScriptCmd.build_forward_kinematics(jntPos))


def main() -> None:
    rclpy.init()
    node = Cr7ScriptClient()
    # 示例：两次 movej，与 C++ 示例保持一致
    joints = [math.pi/2, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
    ok, res = node.movej(joints, speed=100, zone=5)
    node.get_logger().info(f'Service result: {res}')

    joints2 = [math.pi/2, 0.0, math.pi/2, 0.0, 0.0, 0.0]
    ok, res = node.movej(joints2, speed=100, zone=5)
    node.get_logger().info(f'Service result: {res}')

    # 清理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
