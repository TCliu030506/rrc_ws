

import rclpy
import time
import numpy as np
from rclpy.node import Node

from coordinate.srv import StringScript

class Client(Node):
  def __init__(self) -> None:
        super().__init__('hand_eye_client')
        self.client = self.create_client(StringScript, 'hand_eye_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/hand_eye_command: waiting...')
  def Record(self):
    request = StringScript.Request()
    request.command = "Rec"
    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is not None:
        self.get_logger().info(f'Result:{future.result()}')
    else:
        self.get_logger().error('Service call failed')
  def Calculate(self):
    request = StringScript.Request()
    request.command = "Cal"
    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    if future.result() is not None:
        self.get_logger().info(f'Result:{future.result()}')
    else:
        self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    client = Client()
    while rclpy.ok():
      user_input = input("Enter a motion mode: 1 for record, 2 for calculate, 0 for quit\n")  # 读取用户输入
      
      if int(float(user_input)) == 0:
        break
      elif int(float(user_input)) == 1:
        client.Record()
      elif int(float(user_input)) == 2:
        client.Calculate()

      time.sleep(1)

    rclpy.shutdown()

if __name__ == '__main__':
    
    main()