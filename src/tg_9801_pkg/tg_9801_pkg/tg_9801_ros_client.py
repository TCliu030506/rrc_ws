"""TG-9801 ROS2 客户端封装。

用于通过 tg_9801_node 暴露的 ROS2 服务/话题进行统一调用，
便于上层成员直接以 Python 方法控制夹爪。
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class TG9801RosClient(Node):
    """与 tg_9801_node 通信的高层客户端。"""

    def __init__(
        self,
        service_name: str = 'set_grip',
        raw_topic: str = 'raw_tx',
        rx_topic: str = 'rx_hex',
        command_topic: str = 'command_json',
        response_topic: str = 'response_json',
        wait_service_timeout: float = 5.0,
    ) -> None:
        super().__init__('tg_9801_ros_client')

        self._service_name = service_name
        self._raw_topic = raw_topic
        self._rx_topic = rx_topic
        self._command_topic = command_topic
        self._response_topic = response_topic

        self._set_grip_client = self.create_client(SetBool, self._service_name)
        self._raw_pub = self.create_publisher(String, self._raw_topic, 10)
        self._command_pub = self.create_publisher(String, self._command_topic, 10)

        self._last_rx: Optional[str] = None
        self._last_response: Optional[Dict[str, Any]] = None

        self.create_subscription(String, self._rx_topic, self._on_rx, 10)
        self.create_subscription(String, self._response_topic, self._on_response, 10)

        if not self._set_grip_client.wait_for_service(timeout_sec=max(0.1, wait_service_timeout)):
            raise RuntimeError(f'等待服务 {self._service_name} 超时')

    def _on_rx(self, msg: String) -> None:
        self._last_rx = msg.data

    def _on_response(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, dict):
                self._last_response = parsed
        except Exception:
            self._last_response = {
                'ok': False,
                'error': f'response_json 不是合法 JSON: {msg.data}',
            }

    def _wait_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(predicate())

    def set_grip(self, grip: bool, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        """调用 set_grip 服务。"""
        req = SetBool.Request()
        req.data = bool(grip)

        future = self._set_grip_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(0.1, timeout_sec))

        if not future.done():
            return False, f'调用 {self._service_name} 超时({timeout_sec:.2f}s)'

        exc = future.exception()
        if exc is not None:
            return False, f'调用异常: {exc}'

        result = future.result()
        if result is None:
            return False, '服务返回为空'

        return bool(result.success), str(result.message)

    def grip(self, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        """快捷方法：夹取。"""
        return self.set_grip(True, timeout_sec=timeout_sec)

    def release(self, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        """快捷方法：松开。"""
        return self.set_grip(False, timeout_sec=timeout_sec)

    def send_raw(self, payload: str) -> None:
        """向 raw_tx 发布原始字节串（如 '0xA3 0xB4 0x02 0x01 0x01 0x04'）。"""
        self._raw_pub.publish(String(data=payload))

    def get_last_rx(self) -> Optional[str]:
        """获取最近一次 rx_hex 内容。"""
        return self._last_rx

    def send_command(
        self,
        command: Dict[str, Any],
        timeout_sec: float = 2.0,
    ) -> Tuple[bool, Dict[str, Any]]:
        """向 command_json 发送命令并等待 response_json。"""
        action = str(command.get('action', '')).strip().lower()
        if not action:
            return False, {'ok': False, 'error': 'command 缺少 action'}

        self._last_response = None
        self._command_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))

        ok = self._wait_until(lambda: self._last_response is not None, timeout_sec)
        if not ok:
            return False, {'ok': False, 'error': f'等待 response_json 超时({timeout_sec:.2f}s)', 'action': action}

        response = dict(self._last_response or {})
        if 'action' not in response:
            response['action'] = action
        return bool(response.get('ok', False)), response

    def get_status(self, protocol: str = 'modbus_rtu', device_addr: int = 1, timeout_sec: float = 2.0) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command({'action': 'get_status', 'protocol': protocol, 'device_addr': int(device_addr)}, timeout_sec)

    def get_force(self, protocol: str = 'modbus_rtu', device_addr: int = 1, timeout_sec: float = 2.0) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command({'action': 'get_force', 'protocol': protocol, 'device_addr': int(device_addr)}, timeout_sec)

    def set_force(
        self,
        target_force: int,
        protocol: str = 'modbus_rtu',
        device_addr: int = 1,
        timeout_sec: float = 2.0,
    ) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command(
            {
                'action': 'set_force',
                'protocol': protocol,
                'device_addr': int(device_addr),
                'target_force': int(target_force),
            },
            timeout_sec,
        )

    def get_id(self, protocol: str = 'modbus_rtu', device_addr: int = 1, timeout_sec: float = 2.0) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command({'action': 'get_id', 'protocol': protocol, 'device_addr': int(device_addr)}, timeout_sec)

    def get_serial(self, protocol: str = 'modbus_rtu', device_addr: int = 1, timeout_sec: float = 2.0) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command({'action': 'get_serial', 'protocol': protocol, 'device_addr': int(device_addr)}, timeout_sec)

    def move_position(
        self,
        position: int,
        speed: int = 1000,
        protocol: str = 'modbus_rtu',
        device_addr: int = 1,
        timeout_sec: float = 2.0,
    ) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command(
            {
                'action': 'position',
                'protocol': protocol,
                'device_addr': int(device_addr),
                'position': int(position),
                'speed': int(speed),
            },
            timeout_sec,
        )

    def set_position_mode(
        self,
        mode: str = 'position',
        protocol: str = 'modbus_rtu',
        device_addr: int = 1,
        timeout_sec: float = 2.0,
    ) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command(
            {
                'action': 'set_position_mode',
                'protocol': protocol,
                'device_addr': int(device_addr),
                'position_mode': mode,
            },
            timeout_sec,
        )

    def read_holding(
        self,
        start_addr: int,
        count: int,
        device_addr: int = 1,
        timeout_sec: float = 2.0,
    ) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command(
            {
                'action': 'read_holding',
                'protocol': 'modbus_rtu',
                'device_addr': int(device_addr),
                'read_start_addr': int(start_addr),
                'read_count': int(count),
            },
            timeout_sec,
        )

    def write_single(
        self,
        register_addr: int,
        register_value: int,
        device_addr: int = 1,
        timeout_sec: float = 2.0,
    ) -> Tuple[bool, Dict[str, Any]]:
        return self.send_command(
            {
                'action': 'write_single',
                'protocol': 'modbus_rtu',
                'device_addr': int(device_addr),
                'register_addr': int(register_addr),
                'register_value': int(register_value),
            },
            timeout_sec,
        )


def send_raw_and_wait_rx(
    client: TG9801RosClient,
    payload: str,
    timeout_sec: float = 1.0,
) -> Tuple[bool, Optional[str]]:
    """工具函数：发送 raw 数据并等待一次 rx_hex。"""
    before = client.get_last_rx()
    client.send_raw(payload)
    got = client._wait_until(lambda: client.get_last_rx() != before, timeout_sec)
    return got, client.get_last_rx()
