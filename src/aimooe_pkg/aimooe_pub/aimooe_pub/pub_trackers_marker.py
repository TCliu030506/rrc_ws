# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from aimooe_pub.aim import package_path
from aimooe_pub.aim import tracker
from aimooe_sdk.msg import AimCoord

# topic pub cycle
class Time:
    freq = 0.1 # seconds
    
class Publisher(Node):

    def __init__(self):
        super().__init__('Aimooe_optical_tracker')
        self.obj = tracker(package_path+'/Aimtools/')
        self.msg = AimCoord()
        self.publisher_ = self.create_publisher(AimCoord, 'aimooe_tracker', 10)
        timer_period = Time.freq  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pub_msg()
        self.pub_marker()

    def pub_msg(self):
        # read data
        msg_list = self.obj.require_tools()
        if not msg_list:
            # print("List is empty")
            pass
        else:
            # print("List is not empty")
            for msg in msg_list:
                self.msg.header.frame_id = msg[0]
                self.msg.header.stamp = self.get_clock().now().to_msg()
                pose_axang = msg[1]
                self.msg.position.x     = pose_axang[0]
                self.msg.position.y     = pose_axang[1]
                self.msg.position.z     = pose_axang[2]
                self.msg.orientation.x  = pose_axang[3]
                self.msg.orientation.y  = pose_axang[4]
                self.msg.orientation.z  = pose_axang[5]
                self.msg.mean_error = msg[2]

                self.publisher_.publish(self.msg)

    def pub_marker(self):
        # 调用函数获取所有标记点数据
        all_markers = self.obj.read_markers_info()
        # 遍历所有标记点，寻找z值最小的点
        min_z = 10000.0  # 初始化为一个较大的值
        min_marker = None
        # 遍历所有标记点寻找最小Z值
        for marker in all_markers:
            current_z = marker['z']  # 假设Z坐标在索引3的位置
            if current_z < min_z:
                min_z = current_z
                min_marker = marker
        # 发布最小Z值的标记点
        self.msg.header.frame_id = 'Tip_Marker'
        self.msg.position.x     = min_marker['x']
        self.msg.position.y     = min_marker['y']
        self.msg.position.z     = min_marker['z']
        self.msg.orientation.x = 0.0
        self.msg.orientation.y = 0.0
        self.msg.orientation.z = 0.0
        self.publisher_.publish(self.msg)
                
def main(args=None):
    rclpy.init(args=args)

    # python_interpreter_path = sys.executable
    # print(f"The Python interpreter is located at: {python_interpreter_path}")

    optical_pub = Publisher()

    rclpy.spin(optical_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    optical_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()