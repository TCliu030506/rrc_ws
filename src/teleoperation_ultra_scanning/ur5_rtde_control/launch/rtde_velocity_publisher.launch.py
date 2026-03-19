from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    topic_cmd_vel = LaunchConfiguration('topic_cmd_vel')
    adaptive_acc_min = LaunchConfiguration('adaptive_acc_min')
    adaptive_acc_max = LaunchConfiguration('adaptive_acc_max')
    adaptive_acc_scale = LaunchConfiguration('adaptive_acc_scale')
    enable_debug_output = LaunchConfiguration('enable_debug_output')
    topic_debug_sent_velocity = LaunchConfiguration('topic_debug_sent_velocity')
    topic_debug_sent_acceleration = LaunchConfiguration('topic_debug_sent_acceleration')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('topic_cmd_vel', default_value='/UR5/desired_twist'),
        DeclareLaunchArgument('adaptive_acc_min', default_value='0.2'),
        DeclareLaunchArgument('adaptive_acc_max', default_value='3.0'),
        DeclareLaunchArgument('adaptive_acc_scale', default_value='0.5'),
        DeclareLaunchArgument('enable_debug_output', default_value='true'),
        DeclareLaunchArgument('topic_debug_sent_velocity', default_value='/UR5/debug/sent_velocity'),
        DeclareLaunchArgument('topic_debug_sent_acceleration', default_value='/UR5/debug/sent_acceleration'),
        Node(
            package='ur5_rtde_control',
            executable='rtde_velocity_publisher_node',
            name='rtde_velocity_publisher_node',
            output='screen',
            parameters=[
                {'robot_ip': robot_ip},
                {'topic_cmd_vel': topic_cmd_vel},
                {'adaptive_acc_min': adaptive_acc_min},
                {'adaptive_acc_max': adaptive_acc_max},
                {'adaptive_acc_scale': adaptive_acc_scale},
                {'enable_debug_output': enable_debug_output},
                {'topic_debug_sent_velocity': topic_debug_sent_velocity},
                {'topic_debug_sent_acceleration': topic_debug_sent_acceleration},
            ],
        ),
    ])
