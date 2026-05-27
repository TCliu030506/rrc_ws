import rclpy
from rclpy.node import Node
import numpy as np
import ur5_control.ur5_kinematics as kinematics
from std_msgs.msg import String
from ur5_control.URControl import URNode

class URNodeTest(Node):
    def __init__(self):
        super().__init__('ur_node')
        self.node = URNode()
        self.send_servoj_command()

    def send_servoj_command(self):
        joint_pos = np.array([0,-90,-100,-90,90,0])
        joint_pos = kinematics.joint_deg_to_rad(joint_pos)
        self.node.servoj([joint_pos])
        print(joint_pos)

def main(args=None):
    rclpy.init(args=args)
    node = URNodeTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
