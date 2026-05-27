from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    force_poll_hz = LaunchConfiguration('force_poll_hz')
    state_poll_hz = LaunchConfiguration('state_poll_hz')

    return LaunchDescription([
        # 频率参数
        DeclareLaunchArgument(
            'force_poll_hz',
            default_value='50.0',
            description='Force sensor publish frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'state_poll_hz',
            default_value='50.0',
            description='State sensor publish frequency (Hz)'
        ),

        # 服务端
        Node(
            package='xmate_cr7_script',
            executable='cr7_server',
            name='cr7_server',
            output='screen'
        ),

        # 力传感节点
        Node(
            package='xmate_cr7_script',
            executable='cr7_force_sensor',
            name='cr7_force_sensor',
            output='screen',
            parameters=[{'force_poll_hz': force_poll_hz}]
        ),

        # 状态传感节点（注意：你的源文件为 cr7_state_seneor.cpp，默认可执行名也应为 cr7_state_seneor）
        Node(
            package='xmate_cr7_script',
            executable='cr7_state_seneor',
            name='cr7_state_seneor',
            output='screen',
            parameters=[{'state_poll_hz': state_poll_hz}]
        ),
    ])