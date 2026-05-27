import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from msg_codec_py.msg_coder import MsgCoder

class MsgSubscriber(Node):
    def __init__(self):
        super().__init__('msg_subscriber')
        MsgCoder.add_cmd_script(b'MV', {'cmd': '2s', 'id': 'b', 'pos': 'q', 'vel': 'd'})
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'encoded_msg',
            self.decode_message,
            10
        )
        self.get_logger().info('MsgSubscriber has been started.')

    def decode_message(self, msg: UInt8MultiArray):
        try:
            encoded_msg = bytes(msg.data)
            decoded_msg = MsgCoder.msg_decode(encoded_msg)
            self.get_logger().info(f'Received and decoded message: {decoded_msg}')
        except Exception as e:
            self.get_logger().error(f'Error decoding message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MsgSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()