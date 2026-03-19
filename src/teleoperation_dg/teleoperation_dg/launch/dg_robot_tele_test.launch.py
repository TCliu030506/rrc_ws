import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    return LaunchDescription([
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
            executable='teleoperation_control',
            output='screen',
            name='dg_teleoperation_slave_node'
        ),

    ])
