#!/usr/bin/env python3

from typing import Type, List, Tuple
from interfaces_py.interfaces import MsgRecorder
from x6_msg.msg import X6Position
from aimooe_sdk.msg import AimCoord
from lkactuator_msg.msg import LkPosition

import os
pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def main():
    """
    监听多个话题并记录消息保存到同级record文件夹
    """
    msg_list: List[Tuple[Type, str]] = [
        (LkPosition, "out_1"),
        (LkPosition, "out_2"),
        (LkPosition, "out_3"),
        (LkPosition, "out_4"),
        (LkPosition, "out_5"),
        (LkPosition, "out_6"),
        (X6Position, "x6position"),
        #(X6Position, "x6theo"),
        (AimCoord, "aimooe_coord")
    ]

    # 实例化 MsgRecorder 类
    handler = MsgRecorder(msg_list, package_path=pkg_path, save_path="record")

    input("请输入一个数以开始录制")
    handler.start_recording(30)
    input("请输入一个数以结束录制")
    handler.stop_recording()
    handler.stop_node()

if __name__ == "__main__":
    main()