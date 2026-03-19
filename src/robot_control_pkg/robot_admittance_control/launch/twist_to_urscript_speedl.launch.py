from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic_arm_command', default_value='/UR5/desired_twist'),
        DeclareLaunchArgument('speedl_acc', default_value='0.5'),
        DeclareLaunchArgument('speedl_time', default_value='0.08'),
        DeclareLaunchArgument('publish_rate', default_value='125.0'),
        DeclareLaunchArgument('command_timeout', default_value='0.2'),
        DeclareLaunchArgument('zero_on_timeout', default_value='true'),
        DeclareLaunchArgument('send_stop_on_exit', default_value='true'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.5'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.6'),
        DeclareLaunchArgument('skip_repeated_zero_command', default_value='true'),
        DeclareLaunchArgument('zero_command_epsilon', default_value='1e-6'),
        Node(
            package='robot_admittance_control',
            executable='twist_to_urscript_speedl_node',
            name='twist_to_urscript_speedl_node',
            output='screen',
            parameters=[{
                'topic_arm_command': LaunchConfiguration('topic_arm_command'),
                'speedl_acc': LaunchConfiguration('speedl_acc'),
                'speedl_time': LaunchConfiguration('speedl_time'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'command_timeout': LaunchConfiguration('command_timeout'),
                'zero_on_timeout': LaunchConfiguration('zero_on_timeout'),
                'send_stop_on_exit': LaunchConfiguration('send_stop_on_exit'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'skip_repeated_zero_command': LaunchConfiguration('skip_repeated_zero_command'),
                'zero_command_epsilon': LaunchConfiguration('zero_command_epsilon'),
            }],
        ),
    ])
