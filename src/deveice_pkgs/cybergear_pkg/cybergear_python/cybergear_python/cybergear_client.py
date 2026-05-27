import rclpy
from rclpy.node import Node
from cybergear_msg.srv import CybergearScript
from cybergear_msg.msg import CybergearState
import rclpy.clock


class Subscriber:
    """
    监听订阅一次消息
    """
    def __init__(self,frame) -> None:
        self.on = True
        # 参数 frame 是一个字符串，比如 "cybergear_actuator_id1"，用来匹配目标消息。
        self.frame = frame
        self.result = CybergearState()
        # 生成ROS节点（包含时间戳，防止重名冲突）
        self.node = rclpy.create_node('cybergear_'+str(rclpy.clock.Clock().now().nanoseconds))
        # 订阅话题cybergear_state,收到消息时回调函数 msg_callback被触发
        self.subscription = self.node.create_subscription(CybergearState,'cybergear_state',self.msg_callback,2)
        pass
        

    def msg_callback(self, msg: CybergearState):
        """
        监听参数frame指定的数据
        """
        if msg.header.frame_id == self.frame:
            self.result = msg
            self.on = False

    def msg_listen(self):
        """
        听到指定的消息之后返回该消息
        """
        self.on = True
        self.node.get_logger().info('/cybergear_state: waiting...')
        while self.on:
            rclpy.spin_once(self.node)
        return self.result



class CybergearClient(Node):
    """
    用于控制 CyberGear 设备的 ROS2 客户端
    """ 
    def __init__(self, id: int) -> None:
        # 生成ROS节点（包含时间戳，防止重名冲突）
        super().__init__('cybergear_client_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.id = int(id)
        self.client = self.create_client(CybergearScript, 'cybergear_srv')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/cybergear_srv: waiting...')

    def wake(self):
        """
        串口唤醒
        """
        # W
        request = CybergearScript.Request()
        request.command = "W"
        request.id = self.id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def enable(self):
        """
        电机使能
        """
        # E
        request = CybergearScript.Request()
        request.command = "E"
        request.id = self.id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def disable(self):
        """
        电机失能
        """
        # D
        request = CybergearScript.Request()
        request.command = "D"
        request.id = self.id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_pos(self, speed: float, pos: float):
        """
        位置控制，速度+位置
        此模式下，电机将以指定速度运动到指定位置。
        """
        # S
        request = CybergearScript.Request()
        request.command = "S"
        request.id = self.id
        request.data.append(speed)
        request.data.append(pos)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def set_current_zero(self):
        """
        设置当前位置为零位
        """
        # Z
        request = CybergearScript.Request()
        request.command = "Z"
        request.id = self.id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def return_to_zero(self):
        """
        回零位
        """
        # R
        request = CybergearScript.Request()
        request.command = "R"
        request.id = self.id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')

    def move_torque_control(self, torque: float, position: float, speed: float, kp: float, kd: float):
        """
        力矩控制模式
        此模式下，电机将以指定速度以指定力矩接近目标位置。
        """
        # T
        request = CybergearScript.Request()
        request.command = "T"
        request.id = self.id
        request.data.append(torque)
        request.data.append(position)
        request.data.append(speed)
        request.data.append(kp)
        request.data.append(kd)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Result:{future.result()}')
        else:
            self.get_logger().error('Service call failed')



