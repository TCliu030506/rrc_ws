import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    # 启动UI界面节点
    ui_node = Node(
        package='tus_ui',
        executable='tus_ui',
        name='tus_ui_node',
        output='screen',
    )
    return LaunchDescription([

        # 启动超声传输数据节点（需要后续修改）
        Node(
            package='tus_ui',
            executable='img_capture_node',
            name='img_capture_node',
            output='screen',
        ),

        # 启动 USB 相机发布节点
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_camera_node',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 1280},
                {'image_height': 720},
                {'framerate': 15.0},
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
            name='teleoperation_master_node',
            parameters=[
                {"omni_name": "phantom"},
                {"publish_rate": 1000},
                {"reference_frame": "/map"},
                {"units": "mm"}
            ]
        ),

        # 启动遥操作从端设备节点
        Node(
            package='ur5_rtde_control',
            executable='teleoperation_control_ui',
            output='screen',
            name='teleoperation_slave_node'
        ),

        # 启动UI界面节点（单独变量，供事件引用）
        ui_node,
        # UI 进程退出 -> 触发整个 launch Shutdown -> 其余节点会被一起 SIGINT 退出
        RegisterEventHandler(
            OnProcessExit(
                target_action=ui_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='tus_ui_node exited, shutting down launch'))
                ],
            )
        ),

    ])
