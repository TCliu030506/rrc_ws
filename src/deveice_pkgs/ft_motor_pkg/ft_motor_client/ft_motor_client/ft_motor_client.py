

import rclpy
import time

from rclpy.node import Node
from ft_motor_msg.srv import FTCommand
from ft_motor_msg.msg import FTMotor

class Subscriber:
  def __init__(self,frame) -> None:
    self.on =True
    self.frame = frame
    self.result = FTMotor()
    self.node = rclpy.create_node('ftmotor_subcriber')
    self.subscription = self.node.create_subscription(FTMotor,'ftmotor_msg',self.msg_callback,2)

    pass

  def msg_callback(self, msg):
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
    self.node.get_logger().info('/ftmotor_msg: waiting...')
    while self.on:
      rclpy.spin_once(self.node)
    return self.result
  
class Client(Node):
  def __init__(self) -> None:
      super().__init__('ftmotor_client')
      self.client = self.create_client(FTCommand, 'ftmotor_srv')
      while not self.client.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('/ftmotor_srv: waiting...')

  def Move_pos(self,pos1:float,pos2:float):
    """
    Move to the postion input
    """
    request = FTCommand.Request()
    request.command = "MOV"
    request.motor_pos[0] = pos1
    request.motor_pos[1] = pos2
    request.motor_vel[0] = 0.2
    request.motor_vel[1] = 0.2
    request.motor_tim[0] = 1.0
    request.motor_tim[1] = 1.0

    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)

    if future.result() is not None:
        self.get_logger().info(f'Result:{future.result()}')
    else:
        self.get_logger().error('Service call failed')
  def Turn_zero(self):
    """
    Move to the zero position
    """
    request = FTCommand.Request()
    request.command = "FIX"

    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)

    if future.result() is not None:
        self.get_logger().info(f'Result:{future.result()}')
    else:
        self.get_logger().error('Service call failed')
  def Set_zero(self):
    """
    Move to zero pos
    """
    request = FTCommand.Request()
    request.command = "SET"
    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)

    if future.result() is not None:
        self.get_logger().info(f'Result:{future.result()}')
    else:
        self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    motor = Client()
    sub = Subscriber('ft_motor')
    while rclpy.ok():
      ftmotor_msg = sub.msg_listen()
      print(f'The current rotation angle of ft_motor is [{ftmotor_msg.motor[0]}deg, {ftmotor_msg.motor[1]}deg].\n')
      user_input = input("Enter a motion mode: 1 for move with input ft_motor angle, 2 for turn zero, 3 for set zero, 0 for quit\n")  # 读取用户输入
      
      if int(float(user_input)) == 0:
        break
      elif int(float(user_input)) == 1:
        pos1 = input("Enter the values for motor_1: [-90 - 90]deg\n")
        pos2 = input("Enter the values for motor_2: [-90 - 90]deg\n")
        motor.Move_pos(float(pos1),float(pos2))
      elif int(float(user_input)) == 2:
        motor.Turn_zero()
      elif int(float(user_input)) == 3:
        motor.Set_zero()

      time.sleep(3)

    rclpy.shutdown()

if __name__ == '__main__':
    
    main()