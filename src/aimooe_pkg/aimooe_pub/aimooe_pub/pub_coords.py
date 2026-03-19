# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from aimooe_sdk.msg import AimCoord
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from typing import List

import numpy as np 
from scipy.spatial.transform import Rotation
import math
import copy

# topic pub cycle
class Params:
    base_coord = 'base_frame'

class Defval:
    base_coord = 'base_coord'
    
class Publisher(Node):

    def __init__(self):
        super().__init__('Aimooe_tracker_reference')
        self.base_coord = AimCoord()
        self.base_coord.header.stamp.sec = 0

        self.declare_parameter(Params.base_coord, Defval.base_coord)
        self.param_subscriber = self.add_on_set_parameters_callback(self.param_callback)
        self.base_coord_name = self.get_parameter('base_frame').get_parameter_value().string_value
        self.get_logger().info(f'Got base_frame: {self.base_coord_name}')

        self.subscriber_ = self.create_subscription(AimCoord, 'aimooe_tracker', self.msg_callback, 10)
        self.publisher_ = self.create_publisher(AimCoord, 'aimooe_coord', 10)

    def param_callback(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == Params.base_coord:
                if param.type_ == Parameter.Type.STRING:
                    self.base_coord_name = param.value
                    self.get_logger().info(f'Parameter base_frame changed to: {self.base_coord_name}')
                else:
                    self.get_logger().warn(f'Parameter base_frame has incorrect type: {param.type_}')
        return SetParametersResult(successful=True)

    def msg_callback(self, msg: AimCoord):
        if msg.header.frame_id == self.base_coord_name:
            self.base_coord = msg
            # self.get_logger().info('base frame updated')
        else:
            if (msg.header.stamp.sec - self.base_coord.header.stamp.sec) > 1.0:
                self.get_logger().warn('tool frame is not updating, time delay ' + str(msg.header.stamp.sec - self.base_coord.header.stamp.sec) + 's')
                return
            else:
                p_base = np.array([ [self.base_coord.position.x],
                                    [self.base_coord.position.y],
                                    [self.base_coord.position.z]])
                rotm_base = Rotation.from_rotvec(np.array([ self.base_coord.orientation.x,
                                                            self.base_coord.orientation.y,
                                                            self.base_coord.orientation.z]))
                
                p_tool = np.array([ [msg.position.x],
                                    [msg.position.y],
                                    [msg.position.z]])
                rotm_tool = Rotation.from_rotvec(np.array([ msg.orientation.x,
                                                            msg.orientation.y,
                                                            msg.orientation.z]))
                
                rotm = np.dot(np.linalg.inv(rotm_base.as_matrix()),rotm_tool.as_matrix())
                p = np.linalg.solve(rotm_base.as_matrix(),p_tool-p_base)
                r = Rotation.from_matrix(rotm).as_rotvec()

                tool_coord = AimCoord()
                tool_coord.header = copy.deepcopy(msg.header)
                tool_coord.position.x = p[0,0]
                tool_coord.position.y = p[1,0]
                tool_coord.position.z = p[2,0]
                tool_coord.orientation.x = r[0]
                tool_coord.orientation.y = r[1]
                tool_coord.orientation.z = r[2]
                tool_coord.mean_error = msg.mean_error
                self.publisher_.publish(tool_coord)
                
def main(args=None):
    rclpy.init(args=args)
    optical_pub = Publisher()

    rclpy.spin(optical_pub)

    optical_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()