# inspire_client.py

import rclpy
from aimooe_sdk.msg import AimCoord
import rclpy.clock
import rclpy.node

class Subscriber:
    """
    监听指定的消息(topic)下,指定工具坐标系(frame)的数据
    """
    def __init__(self, topic, frame) -> None:
        self.on = True
        self.frame = frame
        self.result = AimCoord()
        self.node = rclpy.create_node('aimooe_'+str(rclpy.clock.Clock().now().nanoseconds))
        self.subscription = self.node.create_subscription(AimCoord,topic,self.msg_callback,2)
        pass

    def msg_callback(self, msg: AimCoord):
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
        self.node.get_logger().info('/aimooe_tracker: waiting...')
        while self.on:
            rclpy.spin_once(self.node)
        return self.result
    
def main(args=None):
    sub = Subscriber('aimooe_tracker','Probe')
    print(sub.msg_listen())

if __name__ == '__main__':

    main()    