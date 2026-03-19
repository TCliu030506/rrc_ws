# ui_test
## Overview
创建UI窗口，并提供相关软件功能集成

## 使用说明
1、启动内窥镜相机发布节点
 ros2 run usb_cam usb_cam_node_exe --ros-args -r __node:=camera1_node -p video_device:=/dev/video0 -p image_width:=400 -p image_height:=400 -p framerate:=30.0 -p pixel_format:=mjpeg2rgb -r image_raw:=camera1/image_raw
2、启动usb相机发布节点
 ros2 run usb_cam usb_cam_node_exe --ros-args -r __node:=camera2_node -p video_device:=/dev/video3 -p image_width:=640 -p image_height:=480 -p framerate:=30.0 -p pixel_format:=mjpeg2rgb -r image_raw:=camera2/image_raw


2、ros2 run camera_service service_object_server 
3、ros2 run ui_test ui_node

## 注意事项
一定要把两个摄像头分别插在电脑的USB插口和拓展坞的USB插口上，否则会出现摄像头相互占用的问题！！！

## 如何查看当前摄像头端口号
ls /dev/video*

## 图像化显示当前节点和话题
rqt_graph