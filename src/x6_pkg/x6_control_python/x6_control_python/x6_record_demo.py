#!/usr/bin/env python3

from typing import Type, List, Tuple
from interfaces_py.interfaces import MsgRecorder
from x6_msg.msg import X6Position
from lkactuator_msg.msg import LkPosition
from aimooe_sdk.msg import AimCoord

import os
pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def main():
    """
    监听多个话题并记录消息保存到同级record文件夹
    """
    msg_list: List[Tuple[Type, str]] = [
        (X6Position, "x6position"),
        (AimCoord, "aim_coord"),
        (X6Position, "x6theo"),
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