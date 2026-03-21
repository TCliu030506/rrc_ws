"""TG-9801 ROS 客户端 demo。

示例展示如何像普通 Python SDK 一样调用 tg_9801_node 的 ROS2 接口。
运行前请先启动：
    ros2 launch tg_9801_pkg tg_9801.launch.py
"""

from __future__ import annotations

import json
import time
from typing import Callable, Dict, Tuple

import rclpy

from .tg_9801_ros_client import TG9801RosClient


def _print_result(tag: str, ok: bool, payload) -> None:
    if isinstance(payload, dict):
        text = json.dumps(payload, ensure_ascii=False)
    else:
        text = str(payload)
    print(f'[{tag}] ok={ok} | {text}')


def _call_with_retry(
    fn: Callable[[], Tuple[bool, Dict]],
    retries: int = 3,
    interval_sec: float = 0.15,
) -> Tuple[bool, Dict]:
    last_ok = False
    last_payload: Dict = {'ok': False, 'error': 'unknown'}
    for _ in range(max(1, retries)):
        ok, payload = fn()
        last_ok = ok
        last_payload = payload
        if ok:
            return True, payload
        time.sleep(max(0.0, interval_sec))
    return last_ok, last_payload


def main() -> None:
    rclpy.init()
    node = TG9801RosClient()

    try:
        ok, msg = node.release(timeout_sec=2.0)
        _print_result('release(service)', ok, msg)
        time.sleep(0.2)

        ok, status = _call_with_retry(
            lambda: node.get_status(protocol='modbus_rtu', device_addr=1, timeout_sec=2.0),
            retries=3,
            interval_sec=0.2,
        )
        _print_result('get_status(command_json)', ok, status)

        ok, force = _call_with_retry(
            lambda: node.get_force(protocol='modbus_rtu', device_addr=1, timeout_sec=2.0),
            retries=2,
            interval_sec=0.1,
        )
        _print_result('get_force(command_json)', ok, force)

        ok, serial = _call_with_retry(
            lambda: node.get_serial(protocol='modbus_rtu', device_addr=1, timeout_sec=2.0),
            retries=2,
            interval_sec=0.1,
        )
        _print_result('get_serial(command_json)', ok, serial)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
