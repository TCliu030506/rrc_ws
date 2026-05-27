# inspire_client.py

import rclpy
from inspire_msg.srv import InspireScript
from coordinate.msg import ArrayInt16
import rclpy.clock
from rclpy.node import Node


class Subscriber:
    """
    监听订阅一次消息
    """
    def __init__(self,frame) -> None:
        self.on = True
        self.frame = frame
        self.result = ArrayInt16()
        self.force = 0.0
        self.node = rclpy.create_node('inspire_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.subscription = self.node.create_subscription(ArrayInt16,'insactuator_msg',self.msg_callback,2)
        pass
        

    def msg_callback(self, msg: ArrayInt16):
        """
        监听参数frame指定的数据
        """
        if msg.header.frame_id == self.frame:
            self.result = msg
            self.force = msg.data[1]
            self.on = False

    def msg_listen(self):
        """
        听到指定的消息之后返回该消息
        """
        self.on = True
        # self.node.get_logger().info('/insactuator_msg: waiting...')
        while self.on:
            rclpy.spin_once(self.node)
        return self.result
    
    def msg_listen_force_only(self):
        """
        听到指定的消息之后返回该消息
        """
        self.on = True
        while self.on:
            rclpy.spin_once(self.node)
        return self.force

class Client(Node):
    """
    发送电机控制指令
    """
    def __init__(self, id: int) -> None:
        super().__init__('inspire_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.id = int(id)
        self.client = self.create_client(InspireScript, 'insactuator_srv')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/insactuator_srv: waiting...')

    def stop(self):
        """
        暂停电缸的当前运动。
        """
        request = InspireScript.Request()
        request.command = "S"
        request.id = self.id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')
    
    def mode(self, mode: int):
        """
        设置控制模式: 0：定位，1：伺服，2：速度， 3：力控, 4：速度力控
        """
        request = InspireScript.Request()
        request.command = "M"
        request.id = self.id
        request.data.append(mode)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_pos(self, pos: int):
        """
        此模式下，电缸自动规划路径，以最短时间运动到目标位置，同时返回状态信息。
        """
        try:
            pos = int(pos)
        except ValueError:
            return None
        
        request = InspireScript.Request()
        request.command = "P"
        request.id = self.id
        request.data.append(pos)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_srv(self, pos: int):
        """
        此模式下，需要控制器以固定频率（建议不低于 50Hz）向电缸发送目标位置，电缸将进行位置插补运算，跟随目标位置曲线运动。
        """
        try:
            pos = int(pos)
        except ValueError:
            return None
        
        request = InspireScript.Request()
        request.command = "X"
        request.id = self.id
        request.data.append(pos)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_vel(self, pos: int, vel: int):
        """
        此模式下，驱动器将以设定的目标速度匀速运动到目标位置，并停止。
        """
        try:
            pos = int(pos)
            vel = int(vel)
        except ValueError:
            return None
        
        request = InspireScript.Request()
        request.command = "V"
        request.id = self.id
        request.data.append(pos)
        request.data.append(vel)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_fce(self, force: int):
        """
        此模式下，电缸将动态调节位置以保持实际受力值接近力控目标值。
        """
        try:
            force = int(force)
        except ValueError:
            return None
        
        request = InspireScript.Request()
        request.command = "F"
        request.id = self.id
        request.data.append(force)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')
    
    def move_pvf(self, pos: int, vel: int, force: int):
        """
        此模式下，电缸将以设定的速度向目标位置运动，运动过程中如果受力值（挤压力或者拉伸力）超过力控目标值，电缸立即停止运行。
        """
        try:
            pos = int(pos)
            vel = int(vel)
            force = int(force)
        except ValueError:
            return None
        
        request = InspireScript.Request()
        request.command = "W"
        request.id = self.id
        request.data.append(pos)
        request.data.append(vel)
        request.data.append(force)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')