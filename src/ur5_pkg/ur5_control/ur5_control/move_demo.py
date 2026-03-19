import rclpy
import time
from . import URControl
from .URControl import URNode
from rclpy.node import Node

class URControlNode(Node):
    def __init__(self):
        super().__init__('ur_control_node')
        self.initial_joint_positions = [0,-1.57,-1.57,-1.57,0,1.57]                         #初始关节角度
        self.joint_positions = self.get_user_input_positions()                              #获取用户输入关节角度
        self.joint_parameters = self.get_user_input_parameters()                            #h获取用户输入参数
        self.ur_node = URNode() 
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def get_user_input_positions(self):
        try:
            print ("initial_joint_positions = [0,-90,-90,-90,0,90]" )
            user_input_1 = input("Would you like to enter degrees or radians?(D or R)")   
            user_input_2 = input("Enter 6 joint positions (separated by spaces): ")   
            joint_positions = [float(pos) for pos in user_input_2.split()]
            if len(joint_positions) != 6:
                raise ValueError("Invalid number of joint positions")
            self.get_logger().info("User input successfully received.")
            if user_input_1 == "D" :
                joint_positions[0] = URControl.deg_to_rad(joint_positions[0])
                joint_positions[1] = URControl.deg_to_rad(joint_positions[1])
                joint_positions[2] = URControl.deg_to_rad(joint_positions[2])
                joint_positions[3] = URControl.deg_to_rad(joint_positions[3])
                joint_positions[4] = URControl.deg_to_rad(joint_positions[4])
                joint_positions[5] = URControl.deg_to_rad(joint_positions[5])
            return joint_positions
        except ValueError as e:
            self.get_logger().error(f"Invalid input: {e}")
            return None
        
    def get_user_input_parameters(self):
        try:
            joint_parameters=[0.2,0.2,0,0]
            print("There're four parameters below controling the UR motion:")
            print("a: joint acceleration of leading axis [rad/s^2]")
            print("v: joint speed of leading axis [rad/s]")
            print("t: time [S]")
            print("r: blend radius [m]")
            print ("initial_joint_parameters = [0.2,0.2,0,0]" )
            user_input_1 = input("Would you like to change the parameters?(Y or N)") 
            if user_input_1 == "Y"  :
                user_input_2 = input("Enter 4 joint parameters (separated by spaces): ")   
                joint_parameters = [float(pos) for pos in user_input_2.split()]
                if len(joint_parameters) != 4:
                    raise ValueError("Invalid number of joint parameters")
                self.get_logger().info("User input successfully received.")
            return joint_parameters
        except ValueError as e:
            self.get_logger().error(f"Invalid input: {e}")
            return None

    def timer_callback(self): 
            self.ur_node.movej(self.initial_joint_positions, *self.joint_parameters)  
            time.sleep(10)                                                
            self.ur_node.movej(self.joint_positions, *self.joint_parameters)                          
            time.sleep(10)
            

def main(args=None): 
    rclpy.init(args=args)
    node = URControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass                  
    node.destroy_node()                              
    rclpy.shutdown()     

if __name__ == '__main__':
    main()
