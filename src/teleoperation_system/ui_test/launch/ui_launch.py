import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # 启动UI界面节点
    ui_node = Node(
        package='ui_test',
        executable='ui_node',
        name='ui_node'
    )
    return LaunchDescription([
        # 启动内窥镜相机发布节点
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera1_node',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 400},
                {'image_height': 400},
                {'framerate': 30.0},
                {'pixel_format': 'yuyv2rgb'}
            ],
            remappings=[
                ('image_raw', 'camera1/image_raw')
            ]
        ),
        # 启动 USB 相机发布节点
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera2_node',
            parameters=[
                {'video_device': '/dev/video2'},
                {'image_width': 1280},
                {'image_height': 720},
                {'framerate': 15.0},
                {'pixel_format': 'mjpeg2rgb'}
            ],
            remappings=[
                ('image_raw', 'camera2/image_raw')
            ]
        ),
        # 启动图像处理节点
        Node(
            package='camera_service',
            executable='service_object_server',
            name='image_process_node'
        ),
        # 启动遥操作主端设备节点
        Node(
            package='teleoperation_control',
            executable='sigema7_getpos',
            name='teleoperation_master_node'
        ),
        # 启动遥操作从端设备节点
        Node(
            package='teleoperation_control',
            executable='system_control_node_ui',
            name='teleoperation_slave_node'
        ),
        # 启动机械臂状态记录节点
        Node(
            package='data_process_python',
            executable='xmate_state_record',
            name='xmate_state_record_node'
        ),

        # 启动UI节点
        ui_node,
        # UI节点退出时关闭所有节点
        RegisterEventHandler(
            OnProcessExit(
                target_action=ui_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),

    ])
