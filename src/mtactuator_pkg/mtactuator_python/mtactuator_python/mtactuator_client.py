# mtactuator_client.py

import rclpy
from mtactuator_msg.srv import MtAction
# from mtactuator_msg.msg import MtPosition
import rclpy.clock
from rclpy.node import Node

class Client(Node):

    def __init__(self, id: int) -> None:
        super().__init__('mtactuator_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.id = int(id)
        self.client = self.create_client(MtAction, 'mtactuator_srv')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mtactuator_srv: waiting...')

        self.get_logger().info(
            "\nMODE1-增量模式输入: 速度限制maxspeed(单位dps)、角度值pos(单位0.01deg)"
            "\nMODE2-旋转速度输入: 速度限制maxspeed(单位0.01dps)"
            "\nMODE3-绝对位置输入: 速度限制maxspeed(单位dps)、角度值pos(单位0.01deg)"
            "\nMODE4-绝对位置跟踪: 速度限制maxspeed(单位dps)、角度值pos(单位0.01deg)"
            "\nMODE5-关闭电机"
        )

    def move_angle_add(self, maxspeed: int, pos: int):
        """
        增量模式
        """
        try:
            maxspeed = int(maxspeed)
            pos = int(pos)
        except ValueError:
            return None
        
        request = MtAction.Request()
        request.mode = 1
        request.maxspeed = maxspeed
        request.pos = pos

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_speed(self, speed: int):
        """
        速度控制模式
        """
        try:
            speed = int(speed)
        except ValueError:
            return None
        
        request = MtAction.Request()
        request.mode = 2
        request.speed = speed
        request.pos = 0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_angle_absolute(self, maxspeed: int, pos: int):
        """
        多圈模式
        """
        try:
            maxspeed = int(maxspeed)
            pos = int(pos)
        except ValueError:
            return None
        
        request = MtAction.Request()
        request.mode = 3
        request.maxspeed = maxspeed
        request.pos = pos

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_angle_track(self, maxspeed: int, pos: int):
        """
        跟踪模式
        """
        try:
            maxspeed = int(maxspeed)
            pos = int(pos)
        except ValueError:
            return None
        
        request = MtAction.Request()
        request.mode = 4
        request.maxspeed = maxspeed
        request.pos = pos

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def close(self):
        """
        关闭电机
        """
        request = MtAction.Request()
        request.mode = 5

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def stop(self):
        """
        停止电机
        """
        request = MtAction.Request()
        request.mode = 6

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')
    
    
    