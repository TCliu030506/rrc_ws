#!/usr/bin/env python3

from typing import Type, List, Tuple
from interfaces_py.interfaces import MsgRecorder
from force_sensor_msg.msg import ThreeAxisForce
from dg_msg.msg import DgState

import os
pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def main():
    """
    监听多个话题并记录消息保存到同级record文件夹
    """
    msg_list: List[Tuple[Type, str]] = [
        (ThreeAxisForce, "forcesensor"),
        (DgState, "dg/state")
    ]

    # 实例化 MsgRecorder 类
    handler = MsgRecorder(msg_list, package_path=pkg_path, save_path="record")

    input("请输入一个数以开始录制")
    handler.start_recording(50)
    input("请输入一个数以结束录制")
    handler.stop_recording()
    handler.stop_node()

if __name__ == "__main__":
    main()