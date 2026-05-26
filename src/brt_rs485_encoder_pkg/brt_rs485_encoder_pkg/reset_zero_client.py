#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple ROS 2 client for requesting BRT RS485 encoder zero reset."""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ResetZeroClient(Node):
    """Call a std_srvs/Trigger reset-zero service."""

    def __init__(self, service_name: str) -> None:
        super().__init__("brt_rs485_encoder_reset_zero_client")
        self.client = self.create_client(Trigger, service_name)
        self.service_name = service_name

    def call(self) -> bool:
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"Service {self.service_name} is not available."
            )
            return False

        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error("Service call failed: no response.")
            return False

        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().error(result.message)
        return bool(result.success)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)

    service_name = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "/brt_rs485_encoder_node/reset_zero"
    )
    node = ResetZeroClient(service_name)
    try:
        ok = node.call()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()