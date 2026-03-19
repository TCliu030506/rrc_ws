#!/usr/bin/env python3

from typing import Type, List, Tuple
from interfaces_py.interfaces import MsgRecorder
from aimooe_sdk.msg import AimCoord
from coordinate.msg import ArrayInt16

import os
pkg_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def main():
    """
    监听多个话题并记录消息保存到同级record文件夹
    """
    msg_list: List[Tuple[Type, str]] = [
        (AimCoord, "out_1"),
        (AimCoord, "out_2"),
        (AimCoord, "out_3"),
        (AimCoord, "out_4"),    
        (AimCoord, "out_5"),
        (AimCoord, "out_6"),    
        (AimCoord, "out_7"),
        (AimCoord, "out_8"),
        (AimCoord, "out_9"),
        (AimCoord, "out_10"),    
        (AimCoord, "out_11"),
        (AimCoord, "out_12"),
        (AimCoord, "out_13"),
        (ArrayInt16, "data_4"),
        (ArrayInt16, "data_5"),
    ]

    # 实例化 MsgRecorder 类
    handler = MsgRecorder(msg_list, package_path=pkg_path, save_path="record")

    while True:
        user_input = input("请输入操作指令：1-记录一条消息，2-保存并退出，0-直接退出(不保存): ")
        
        try:
            num = int(user_input)
        except ValueError:
            print("请输入有效的数字！")
            continue
            
        if num == 1:
            # 只记录消息，不退出
            handler.add_msg()
            handler.report_recorded_data()
            print("消息已记录，继续...")
            
        elif num == 2:
            # 保存并退出
            handler.save_recording()
            print("数据已保存，正在退出...")
            break
            
        elif num == 0:
            # 直接退出，不保存
            print("直接退出，不保存数据...")
            break
            
        else:
            print("无效输入，请输入 0、1 或 2")

    handler.stop_node()

if __name__ == "__main__":
    main()