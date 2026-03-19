#!/usr/bin/env python3

from typing import Type, List, Tuple
from interfaces_py.interfaces import MsgRecorder
from std_msgs.msg import Float64MultiArray
from aimooe_sdk.msg import AimCoord

import os
pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def main():
    """
    示例脚本，演示如何使用 MsgRecorder 类记录ROS2消息。
    该脚本订阅指定的主题，并在用户输入时记录消息数据，
    最后将记录的数据保存到文件中。
    运行前请确保ROS2环境已正确加载，并有相应的消息发布者在运行。
    运行步骤如下：
    1. 确保ROS2环境已被正确加载。
        source install/setup.bash
    2. 启动多个消息发布者。
        ros2 topic pub /topic_1 std_msgs/msg/Float64MultiArray "data: [1.0, 2.0, 3.0, 4.0, 5.0]" --rate 1
        ros2 run aimooe_pub pub_test
    3. 运行此脚本进行测试。
        python3 interfaces_py/msg_record_demo.py
    退出程序时，输入0即可结束记录并保存数据。
    记录的消息将保存在 "recorded_{timestamp}.csv" 文件中。
    """

    # 要修改的目标节点名称（不含前导斜杠）
    msg_list: List[Tuple[Type, str]] = [
        (Float64MultiArray, "topic_1"),
        (AimCoord, "aimooe_tracker")
    ]

    # 实例化 MsgRecorder 类
    handler = MsgRecorder(msg_list, package_path=pkg_path, save_path="record")

    user_input: int = 1

    while not user_input == 0:
        num = input("请输入一个整数：(0则结束程序)")
        user_input = int(num)
        handler.add_msg()
        handler.report_recorded_data()

    handler.save_recording()
    handler.stop_node()

if __name__ == "__main__":
    main()