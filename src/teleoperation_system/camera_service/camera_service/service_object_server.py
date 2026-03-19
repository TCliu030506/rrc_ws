#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                           # ROS2 Python接口库
from rclpy.node import Node                            # ROS2 节点类
import rclpy.time
from sensor_msgs.msg import Image                      # 图像消息类型
import numpy as np                                     # Python数值计算库
from cv_bridge import CvBridge                         # ROS与OpenCV图像转换类
import cv2                                             # Opencv图像处理库
import threading                                       # 导入线程模块
import time                                            # 导入 time 模块
from std_msgs.msg import Float64                       # 导入 Float64 消息类型

# lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
# upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

lower_red = np.array([0, 60, 100])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

# 红色范围 1：低 H 值部分
lower_red1 = np.array([0, 50, 60])
upper_red1 = np.array([18, 255, 255])
# 红色范围 2：高 H 值部分
lower_red2 = np.array([155, 50, 60])
upper_red2 = np.array([180, 255, 255])

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                          # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'camera1/image_raw', self.listener_callback, 10)             # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()                                     # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换
        
        self.objectX = 0
        self.objectY = 0

        self.pub = self.create_publisher(Image, 'image_detected', 10)   # 创建图像发布者
        self.processed_image = None                                     # 用于存储处理后的图像

        self.pub_area = self.create_publisher(Float64, 'area', 10)      # 创建一个发布者，用于发布血肿的面积
        # 启动发布线程
        self.publish_thread = threading.Thread(target=self.publish_image)
        self.publish_thread.start()
        self.frame_skip = 2  # 每 2 帧处理一次
        self.frame_count = 0

    def listener_callback(self, data):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(image)

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)             # 图像从BGR颜色模型转换为HSV模型
        # mask_red = cv2.inRange(hsv_img, lower_red, upper_red)        # 图像二值化

         # 生成两个范围的掩码
        mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        # 合并两个掩码
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)          # 图像中轮廓检测
        whole_area = 0
        for cnt in contours:                                         # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 200:
                continue

            contour_area = cv2.contourArea(cnt)                      # 计算轮廓面积
            # self.get_logger().info(f'Contour area: {contour_area}')
            # 打印出轮廓面积
            # cv2.putText(image, f'Area: {contour_area}', (x, y-10),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 当面积小于1000时，用黄色绘制轮廓
            if contour_area < 2000:
                cv2.drawContours(image, [cnt], -1, (0, 255, 255), 2)     # 黄色绘制轮廓
            else:
                cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)       # 绿色绘制轮廓
            whole_area += contour_area
            
        # 发布血肿面积
        # 验证面积是否为非负数
        if whole_area >= 0:
            try:
                # 发布血肿面积
                # 将 whole_area 转换为 Python 内置的 float 类型
                self.pub_area.publish(Float64(data=float(whole_area)))
            except Exception as e:
                # 记录发布失败的错误信息
                self.get_logger().error(f"Failed to publish area: {e}")
        else:
            # 记录面积为负数的错误信息
            self.get_logger().error(f"Calculated area is negative: {whole_area}")
    
        # cv2.imshow("object", image)                                  # 使用OpenCV显示处理后的图像效果
        # cv2.waitKey(50)

        self.processed_image = image                                 # 保存处理后的图像

    def publish_image(self):
        while rclpy.ok():
            if self.processed_image is not None:
                # 将处理后的图像转换为ROS消息并发布
                img_msg = self.cv_bridge.cv2_to_imgmsg(self.processed_image, 'bgr8')
                self.pub.publish(img_msg)
            # 用于测试发布成功
            # self.get_logger().info('Publishing video frame')
            # 睡眠1ms  // 25帧
            time.sleep(0.04)


def main(args=None):                                 # ROS2节点主入口main函数

    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImageSubscriber("service_object_server")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()