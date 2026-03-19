#!/usr/bin/env python3

"""
通用消息过滤节点

使用说明:
 - 以 ROS2 节点运行
 - 通过参数指定输入/输出话题、消息类型和目标 frame_id

示例:
ros2 run interfaces_py msg_filter_demo --ros-args -p input_topic:=/aimooe_tracker -p output_topic:=/out -p target_frame_id:=fake_tool

此模块提供一个通用实现，能动态导入消息类型（要求消息具有 `header` 属性），
订阅该类型的话题，检查 `msg.header.frame_id`（如果存在），并在匹配时将消息发布到输出话题。
"""

from __future__ import annotations

import rclpy
from aimooe_sdk.msg import AimCoord
from interfaces_py.interfaces import MsgFilterNode

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MsgFilterNode(AimCoord)
    except Exception:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

