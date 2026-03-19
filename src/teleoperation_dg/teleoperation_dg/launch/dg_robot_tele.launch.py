import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    # 启动UI界面节点(待修改)
    ui_node = Node(
        package='teleoperation_dg',
        executable='dg_ui',
        name='dg_ui_node',
        output='screen',
    )
    return LaunchDescription([
        # 启动mt电机节点
        Node(
            package='mtactuator',
            executable='mtactuator_server_node_8',
            name='mtactuator_server_node_8',
        ),

        # 启动 USB 相机发布节点
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='dg_usb_camera_node',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 1920},
                {'image_height': 1080},
                {'framerate': 30.0},
                {'pixel_format': 'mjpeg2rgb'}
            ],
            remappings=[
                ('image_raw', 'usb_images')
            ]
        ),

        # 启动遥操作主端设备节点
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            name='dg_teleoperation_master_node',
            parameters=[
                {"omni_name": "phantom"},
                {"publish_rate": 1000},
                {"reference_frame": "/map"},
                {"units": "mm"}
            ]
        ),

        # 启动遥操作从端设备节点
        Node(
            package='teleoperation_dg',
            executable='teleoperation_control_rpy',
            output='screen',
            name='dg_teleoperation_slave_node'
        ),


        # 启动dg_python节点
        Node(
            package='dg_python',
            executable='dg_omni_control_ui',
            name='dg_omni_control_node',
        ),

        # 启动UI界面节点（单独变量，供事件引用）
        ui_node,
        # UI 进程退出 -> 触发整个 launch Shutdown -> 其余节点会被一起 SIGINT 退出
        RegisterEventHandler(
            OnProcessExit(
                target_action=ui_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='dg_ui_node exited, shutting down launch'))
                ],
            )
        ),

    ])
