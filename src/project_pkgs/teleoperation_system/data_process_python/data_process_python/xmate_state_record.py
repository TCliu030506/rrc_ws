#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from xmate_cr7_msg.msg import Cr7State

RECORD_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "record")
CSV_PATH = os.path.join(RECORD_DIR, "xmate_state.csv")

class XmateStateRecorder(Node):
    def __init__(self):
        super().__init__('xmate_state_recorder')
        # 订阅 xmate_state
        self.subscription1 = self.create_subscription(
            Cr7State,
            'xmate_state',
            lambda msg: self.listener_callback(msg, 'xmate_state'),
            10)
        # 订阅 xmate_target_pos
        self.subscription2 = self.create_subscription(
            Cr7State,
            'xmate_target_pos',
            lambda msg: self.listener_callback(msg, 'xmate_target_pos'),
            10)

        # 创建目录
        os.makedirs(RECORD_DIR, exist_ok=True)
        self.csv_file = open(CSV_PATH, 'w', newline='')
        self.csv_writer = None
        self.header_written = False

    def listener_callback(self, msg: Cr7State, topic_name: str):
        # 将消息转为字典
        msg_dict = self.msg_to_dict(msg)
        msg_dict['topic'] = topic_name  # 增加来源字段
        # 写入表头
        if not self.header_written:
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=msg_dict.keys())
            self.csv_writer.writeheader()
            self.header_written = True
        # 写入数据
        self.csv_writer.writerow(msg_dict)
        self.csv_file.flush()

    def msg_to_dict(self, msg):
        # 简单展开消息为字典（如有嵌套字段可自行展开）
        result = {}
        for field in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field)
            # 处理嵌套消息
            if hasattr(value, 'get_fields_and_field_types'):
                for sub_field in value.get_fields_and_field_types().keys():
                    result[f"{field}.{sub_field}"] = getattr(value, sub_field)
            else:
                result[field] = value
        return result

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = XmateStateRecorder()
    try:
        print("开始记录 xmate_state 和 xmate_target_pos 消息，按 Ctrl+C 退出并保存数据。")
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("收到退出信号，正在保存数据...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()