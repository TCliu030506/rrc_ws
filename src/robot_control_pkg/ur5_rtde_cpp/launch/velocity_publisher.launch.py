from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5_rtde_cpp',
            executable='velocity_publisher_node',
            name='ur_velocity_publisher_node',
            output='screen',
            parameters=[
                {'robot_ip': '192.168.1.102'},
                {'topic_cmd_vel': '/ur_cmd_vel'},
                {'adaptive_acc_min': 0.3},
                {'adaptive_acc_max': 3.0},
                {'adaptive_acc_scale': 1.0},
                {'enable_debug_output': True},
                {'topic_debug_sent_velocity': '/UR5/debug/sent_velocity'},
                {'topic_debug_sent_acceleration': '/UR5/debug/sent_acceleration'},
                {'dt': 1.0/125.0},
            ]
        )
    ])
