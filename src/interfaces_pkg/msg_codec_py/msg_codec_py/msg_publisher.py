import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from msg_codec_py.msg_coder import MsgCoder

class MsgPublisher(Node):
    def __init__(self):
        super().__init__('msg_publisher')
        MsgCoder.add_cmd_script(b'MV', {'cmd': '2s', 'id': 'b', 'pos': 'q', 'vel': 'd'})
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'encoded_msg', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.get_logger().info('MsgPublisher has been started.')
        self.data_to_encode = {
            'cmd': b'MV',
            'id': 6,
            'pos': 1000,
            'vel': 3.1415926
        }

    def publish_message(self):
        self.data_to_encode['pos'] += 1  # 模拟位置变化
        try:
            encoded_msg = MsgCoder.msg_encode(self.data_to_encode)
            msg = UInt8MultiArray()
            msg.data = list(encoded_msg)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published encoded message: {encoded_msg}')
        except Exception as e:
            self.get_logger().error(f'Error encoding message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MsgPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()