# Camera Service
## Overview
This service is responsible for managing the camera. It is a ROS service that

## 使用说明
1、启动相机发布节点
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480 -p framerate:=30.0 -p pixel_format:=mjpeg2rgb
2、ros2 run camera_service service_object_server 
