from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic_control_wrench', default_value='/arm_admittance_control/control_wrench'),
        DeclareLaunchArgument('frame_id', default_value='base'),
        DeclareLaunchArgument('publish_rate', default_value='100.0'),
        DeclareLaunchArgument('force_x', default_value='0.0'),
        DeclareLaunchArgument('force_y', default_value='0.0'),
        DeclareLaunchArgument('force_z', default_value='-3.7'),
        DeclareLaunchArgument('torque_x', default_value='0.0'),
        DeclareLaunchArgument('torque_y', default_value='0.0'),
        DeclareLaunchArgument('torque_z', default_value='0.0'),
        Node(
            package='robot_admittance_control',
            executable='control_wrench_publisher_node',
            name='control_wrench_publisher_node',
            output='screen',
            parameters=[{
                'topic_control_wrench': LaunchConfiguration('topic_control_wrench'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'force_x': LaunchConfiguration('force_x'),
                'force_y': LaunchConfiguration('force_y'),
                'force_z': LaunchConfiguration('force_z'),
                'torque_x': LaunchConfiguration('torque_x'),
                'torque_y': LaunchConfiguration('torque_y'),
                'torque_z': LaunchConfiguration('torque_z'),
            }],
        ),
    ])
