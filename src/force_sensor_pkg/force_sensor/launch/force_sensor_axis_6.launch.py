from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('forcesensorport', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('pubrate', default_value='100'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='base_frame'),
        DeclareLaunchArgument('topic_name', default_value='external_force_torque_wrench'),
        DeclareLaunchArgument('auto_zero', default_value='false'),
        Node(
            package='force_sensor',
            executable='force_sensor_axis_6',
            name='force_sensor_axis_6',
            output='screen',
            parameters=[{
                'forcesensorport': LaunchConfiguration('forcesensorport'),
                'forcesensor_rate': LaunchConfiguration('pubrate'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic_name': LaunchConfiguration('topic_name'),
                'auto_zero': LaunchConfiguration('auto_zero'),
            }],
        ),
    ])
