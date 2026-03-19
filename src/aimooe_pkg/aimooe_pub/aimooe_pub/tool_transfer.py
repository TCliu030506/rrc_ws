# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from aimooe_pub.aim import package_path, progress_bar
from aimooe_sdk.msg import AimCoord

import numpy as np 
from scipy.spatial.transform import Rotation
import math

import time
import sys
import threading

class Subscriber(Node):

    def __init__(self):
        super().__init__('Aimooe_coordinate_transfer')
        self.subscription = self.create_subscription(
            AimCoord,
            'aimooe_tracker',
            self.handle_msg,
            10
        )
        # save tool coordinate
        self.tool_msg = AimCoord()
        self.tool_frame = input("Enter the name of the tool_frame\n(check file to make sure the tool_frame is in raw state!):")
        # save probe coordinate
        self.probe_msg = AimCoord()
        self.probe_frame = input("Enter the name of the probe_frame:")

        self.msg_lock = False
        self.is_collect = False

        self.ptool_local = np.zeros((3,3))
        self.ptool_refer = np.zeros((3,3))

        self.transfer = np.zeros((1,6))

    def async_handle(self):
        # get coordinates
        self.read_coordinates()

        while 1:
            # collect points
            if self.collect_points():

                # calculation
                self.calculate_tranfer_mat()

                # optical_sub.transfer = np.array([[1, 2, 3, 4, 5, 6]])
                collect = input("Enter 1 to save, else to not save: ")
                if collect == '1':
                    self.save_tranfer_mat()

            if input("Enter 1 to start next collection, 0 to quit:") == '0':
                break

    def handle_msg(self, msg):
        # start recording
        if self.is_collect == False:
            return
        
        self.msg_lock = True
        if msg.header.frame_id == self.tool_frame:
            self.tool_msg = msg
        if msg.header.frame_id == self.probe_frame:
            self.probe_msg = msg
        self.msg_lock = False
    
    def read_coordinates(self):
        coord_filename = package_path + "/Aimtools/" + self.tool_frame + ".transfer"
        try:
            with open(coord_filename, "r") as file:
                point_x = []
                point_y = []
                point_z = []
                for line in file:
                    variables = line.strip().split()
                    if variables and variables[0][-1].isdigit():
                        if len(variables) == 3:
                            point_x.append(variables[0])
                            point_y.append(variables[1])
                            point_z.append(variables[2])
                    else:
                        break
                if len(point_x) >= 4:
                    self.ptool_local = np.vstack(([float(i) for i in point_x],[float(i) for i in point_y],[float(i) for i in point_z]))
                    print("points in local tool-coordinate: ")
                    transpose_m = self.ptool_local.transpose()
                    for row in transpose_m:
                        print(row)
                else:
                    print("no enough points in " + coord_filename + " are collected!") 

        except ValueError:
            print("variables not right!") 

        sys.stdout.flush()    

    def collect_points(self) -> bool:
        self.ptool_refer = np.zeros((self.ptool_local.shape[0],self.ptool_local.shape[1]))

        # according to the column of the matrix, collecting points
        index = 0
        while index < self.ptool_local.shape[1]:
            print("Now we have " + str(index) + " points of " + str(self.ptool_local.shape[1]) + " collected, the next points is " + str(index+1) + ": ", end="")
            print(self.ptool_local[:,index].transpose())
            collect = input("Enter 1 to start collect in 3s, other to quit without saving: ")
            if collect != '1':
                return False
            else:
                progress = progress_bar("进度",30)

                self.tool_msg.position.z = 0.0
                self.probe_msg.position.z = 0.0
                self.is_collect = True
                for i in range(30):
                    time.sleep(0.1)
                    progress.bar((i+1)/30)
                self.is_collect = False

                if abs(self.tool_msg.position.z) < 0.01 or abs(self.probe_msg.position.z) < 0.01:
                    print(" No enough data was collected, try again!")
                    continue

                while self.msg_lock == True:
                    time.sleep(0.001)
                
                p_probe = np.array([[self.probe_msg.position.x],
                                    [self.probe_msg.position.y],
                                    [self.probe_msg.position.z]])
                p_tool  = np.array([[self.tool_msg.position.x],
                                    [self.tool_msg.position.y],
                                    [self.tool_msg.position.z]])
                
                rotm_tool = Rotation.from_rotvec(np.array([self.tool_msg.orientation.x,
                                                           self.tool_msg.orientation.y,
                                                           self.tool_msg.orientation.z]))

                p_intool = np.dot(rotm_tool.as_matrix().transpose(), p_probe - p_tool)

                print(" This point:", end="")
                print(p_intool[:,0].transpose())
                collect = input("Enter 1 to save this point, 0 to miss: ")
                if collect == '1':
                    self.ptool_refer[:,index] = p_intool[:,0]
                    index += 1
                    print("Add " + str(index) + " point:", end="")
                    print(p_intool[:,0].transpose())
        print("Finished! We got points in refer tool-coordinate: ")
        transpose_m = self.ptool_refer.transpose()
        for row in transpose_m:
            print(row)
        return True

    def calculate_tranfer_mat(self):
        ptool_local_slide = np.hstack((self.ptool_local[:,1:],self.ptool_local[:,0:1]))
        ptool_refer_slide = np.hstack((self.ptool_refer[:,1:],self.ptool_refer[:,0:1]))

        # Rot * Mat_local = Mat_refer
        # Rot = Mat_refer * Mat_local^T * (Mat_local * Mat_local^T)^inv
        mat_local = ptool_local_slide - self.ptool_local
        mat_refer = ptool_refer_slide - self.ptool_refer
        mat_local_square = np.dot(mat_local, np.transpose(mat_local))

        if abs(np.linalg.det(mat_local_square)) < 0.0001:
            print("Singular matrix:")
            for row in mat_local_square:
                print(row)
            return

        rot = np.dot( np.dot(mat_refer, np.transpose(mat_local)) , np.linalg.inv(mat_local_square) )
        print("Origin rotation matrix:")
        for row in rot:
            print(row)

        # SVD decomposition
        U,_,V = np.linalg.svd(rot)
        rot_normalized = np.dot(U, V)
        rotm = Rotation.from_matrix(rot_normalized)

        # position
        pos_all = self.ptool_refer - np.dot(rot_normalized, self.ptool_local)
        pose = np.mean(pos_all, axis=1)

        self.transfer[:, :3] = pose
        self.transfer[:,3: ] = rotm.as_rotvec()
        
        print("Finished! We got transfer coordinate:", end="")
        print(self.transfer[0,:])

        # Mean Error
        pos_bias = pos_all - pose.reshape(-1,1)
        pos_norms = np.linalg.norm(pos_bias, axis=0)
        me = np.mean(pos_norms)

        # Check ptool_refer = p_tran + r_tran * ptool_local
        pos_should = np.dot(np.transpose(rot_normalized),self.ptool_refer - pose.reshape(-1,1)) 
        pos_should_bias = pos_should - self.ptool_local

        print("The bias in transfered coordiante are:")
        transpose_m = np.transpose(pos_should_bias)
        for row in transpose_m:
            print(row)

        print("Me:", end="")
        print(me, end="")
        print(" mm")

    def save_tranfer_mat(self): 
        print("Transfer:", end="")
        print(self.transfer[0,:])

        tool_filename = package_path + "/Aimtools/" + self.tool_frame + ".aimtool"
        try:
            with open(tool_filename, "r") as file:
                lines = file.readlines()
            
            ## find 2
            for i, line in enumerate(lines):
                if line.strip() == '2':
                    lines[i+1] = f"{self.transfer[0,0]} {self.transfer[0,1]} {self.transfer[0,2]}\n"
                    lines[i+2] = f"{self.transfer[0,3]} {self.transfer[0,4]} {self.transfer[0,5]}\n"
                    break

            with open(tool_filename, "w") as file:
                file.writelines(lines)

        except FileNotFoundError:
            print("Can not find files in " + tool_filename) 

def main(args=None):
    rclpy.init(args=args)

    optical_sub = Subscriber()

    main_thread = threading.Thread(target=optical_sub.async_handle)
    main_thread.start()
    
    rclpy.spin(optical_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    optical_sub.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':

    main()