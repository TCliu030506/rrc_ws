#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # 保留ROS图像消息类型
import zmq
import cv2
import numpy as np
import struct

class ZmqReceiver(Node):
    def __init__(self):
        super().__init__('zmq_receiver')
        self.publisher_ = self.create_publisher(Image, 'us_images', 10)

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB) # PULL

        # self.socket.bind("tcp://192.168.2.100:5556")
        return_flag = self.socket.connect("tcp://192.168.2.100:5556")
        self.get_logger().info(f"ZMQ connect return flag: {return_flag}")

        # SUB模式必须订阅主题（空字符串表示订阅所有主题）
        # 若发送端指定了主题（如"image"），需改为 self.socket.setsockopt(zmq.SUBSCRIBE, b"image")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")  

        self.get_logger().info("ZMQ receiver ready.")
        self.timer = self.create_timer(0.001, self.loop)  # 1ms

    def cv2_to_imgmsg(self, cv_img, encoding="mono8"):
        """自定义函数：将OpenCV图像转为ROS 2 Image消息"""
        ros_img = Image()
        # 设置图像基本信息
        ros_img.height = cv_img.shape[0]  # 图像高度
        ros_img.width = cv_img.shape[1]   # 图像宽度
        ros_img.encoding = encoding       # 编码格式（mono8对应灰度图）
        
        # 设置像素步长（单通道灰度图，步长=宽度×1字节）
        ros_img.step = ros_img.width * 1
        
        # 转换图像数据为ROS Image支持的整数列表
        ros_img.data = cv_img.tobytes()
        return ros_img

    def loop(self):
        try:
            # 1. 非阻塞接收ZMQ二进制数据
            msg = self.socket.recv(flags=zmq.NOBLOCK)
            self.get_logger().info(f"接收字节数: {len(msg)}, 前40字节: {msg[:40]}")

            # 2. 定义结构体格式（UNDT_U32 对应 struct 格式 'I'，4字节无符号整数）
            # 包头 HeaderOfPacket_T：5个UNDT_U32 → 5*4=20字节，格式串 'IIIII'
            # 图像头 HeaderOfImagingData_T：7个UNDT_U32 → 7*4=28字节，格式串 'IIIIIII'
            PKT_HEADER_FORMAT = 'IIIII'
            IMG_HEADER_FORMAT = 'IIIIIII'
            PKT_HEADER_SIZE = struct.calcsize(PKT_HEADER_FORMAT)  # 20字节
            IMG_HEADER_SIZE = struct.calcsize(IMG_HEADER_FORMAT)  # 28字节
            TOTAL_HEADER_SIZE = PKT_HEADER_SIZE + IMG_HEADER_SIZE  # 总头大小48字节

            # 3. 校验数据长度（至少要包含完整的包头+图像头）
            if len(msg) < TOTAL_HEADER_SIZE:
                self.get_logger().warning(f"数据长度不足（{len(msg)}字节），无法解析头信息")
                return

            # 4. 解析包头（HeaderOfPacket_T）
            pkt_header_data = msg[:PKT_HEADER_SIZE]
            (pktType, pktSer, pktSize, nextpktOffset, boardId) = struct.unpack(PKT_HEADER_FORMAT, pkt_header_data)
            
            # 5. 判断是否为图像包（UPA_PACKET_TYPE_FMC_IMAGE = 1，对应枚举值）
            UPA_PACKET_TYPE_FMC_IMAGE = 1  # 枚举中UPA_PACKET_TYPE_ASCAN=0，FMC_IMAGE=1，MESSAGE=2
            if pktType != UPA_PACKET_TYPE_FMC_IMAGE:
                self.get_logger().warning(f"非图像包（pktType={pktType}），跳过解析")
                return

            # 6. 校验数据包总大小（避免数据截断）
            # if pktSize != len(msg):
            #     self.get_logger().warning(f"数据包大小不匹配（声明{pktSize}字节，实际{len(msg)}字节）")
            #     return

            # 7. 解析图像头（HeaderOfImagingData_T）
            img_header_data = msg[PKT_HEADER_SIZE:PKT_HEADER_SIZE+IMG_HEADER_SIZE]
            (img_type, img_index, pixelX, pixelY, pixelZ, pixelSize, dataSize) = struct.unpack(IMG_HEADER_FORMAT, img_header_data)
            
            # 8. 打印解析到的图像参数（调试用）
            self.get_logger().info(
                f"图像参数：帧号={img_index}, 分辨率={pixelX}×{pixelY}×{pixelZ}, "
                f"像素字节数={pixelSize}, 数据大小={dataSize}字节"
            )

            # 9. 提取图像数据（跳过包头+图像头，取后续数据）
            img_data_start = TOTAL_HEADER_SIZE
            img_data = msg[img_data_start:img_data_start+dataSize]
            
            # 10. 校验图像数据长度
            if len(img_data) != dataSize:
                self.get_logger().warning(f"图像数据长度不匹配（声明{dataSize}字节，实际{len(img_data)}字节）")
                return

            # 11. 解析像素数据（像素范围0~32767 → 对应16位无符号整数 uint16）
            # pixelSize=2 对应16位（32767是2^15-1，符合uint16范围）
            if pixelSize != 2:
                self.get_logger().warning(f"不支持的像素字节数（{pixelSize}），仅支持2字节（16位）")
                return
            
            # 12. 将二进制数据转为numpy数组（uint16类型）
            # 注意：像素维度优先按 2D 处理（pixelZ=1时），若为3D需调整reshape
            try:
                # 计算总像素数：pixelX * pixelY * pixelZ
                total_pixels = pixelX * pixelY * pixelZ
                # 从二进制数据加载为uint16数组
                img_array = np.frombuffer(img_data, dtype=np.uint16).reshape((pixelX, pixelY))  # 2D图像：(高度, 宽度)
            except Exception as e:
                self.get_logger().error(f"像素数据解析失败：{str(e)}")
                return

            # 13. 归一化到0~255（uint8）
            # 原始范围0~32767，线性映射到0~255
            img_normalized = (img_array / 32767.0 * 255).astype(np.uint8)

            # 14. 将图像 img_normalized 翻转90度
            img_normalized = np.rot90(img_normalized, k=-1)  # 顺时针90度
            # 15. 将图像 img_normalized 水平翻转
            img_normalized = np.fliplr(img_normalized)

            # 15. 转换为ROS 2 Image消息并发布
            ros_img = self.cv2_to_imgmsg(img_normalized, encoding="mono8")
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "upa_camera"  # 自定义帧ID
            self.publisher_.publish(ros_img)

        except zmq.Again:
            # 无数据时正常返回
            return
        except struct.error as e:
            self.get_logger().error(f"结构体解析错误：{str(e)}")
        except Exception as e:
            self.get_logger().error(f"图像解析/发布失败：{str(e)}")

def main():
    rclpy.init()
    node = ZmqReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()