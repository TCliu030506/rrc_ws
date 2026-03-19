#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import threading
from coordinate.msg import ArrayInt16
import rclpy.clock

import os
package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

class RecoredNode(Node):
    def __init__(self):
        super().__init__('record_node'+str(rclpy.clock.Clock().now().nanoseconds))

        self.package_name = 'insactuator_client'

        self.subscription = self.create_subscription(ArrayInt16,'insactuator_msg',self.listener,2)

        self.data_list = []
        self.is_storing = False
        self.is_running = True

    # 订阅者回调函数，接收订阅的数据
    def listener(self, msg: ArrayInt16):
        if self.is_storing:
            self.data_list.append(msg)

    # 控制数据存储的函数
    def control_storage(self):
        while self.is_running:
            command = input("Enter command (0: stop and clear data, 1: start storing, 2: stop and save data, 3: quit directly): ")
            if int(command) == 0:
                self.is_storing = False
                self.data_list[:] = []
            elif int(command) == 1:
                self.is_storing = True
            elif int(command) == 2:
                self.is_storing = False
                self.save_data_to_file()
            elif int(command) == 3:
                self.is_storing = False
                self.is_running = False
            else:
                print("Invalid command")
        self.destroy_node()
        rclpy.shutdown()

    # 保存数据到文件的函数
    def save_data_to_file(self):
        if len(self.data_list) == 0:
            print("No data to save")
            return

        file_name = package_path + "/record/data.txt"
        print(f"The path of {self.package_name} is: {file_name}")

        with open(file_name, 'w') as file:
            for data in self.data_list:
                file.write(str(data) + "\n")
        print("Data saved to " + file_name)

def main(args=None):
    rclpy.init(args=args)

    rec = RecoredNode()
    
    # 控制数据存储的线程
    storage_thread = threading.Thread(target=rec.control_storage)
    storage_thread.start()

    rclpy.spin(rec)

if __name__ == '__main__':

    main()