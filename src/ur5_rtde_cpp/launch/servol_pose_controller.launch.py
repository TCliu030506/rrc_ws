from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5_rtde_cpp',
            executable='servol_pose_controller_node',
            name='ur_servol_pose_controller_node',
            output='screen',
            parameters=[
                {'robot_ip': '192.168.1.102'},
                {'topic_cmd_pose': '/ur_cmd_pose'},
                {'speed': 0.15},
                {'acceleration': 0.1},
                {'lookahead_time': 0.1},
                {'gain': 300.0},
                {'tcp_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'dt': 1.0/125.0},
            ]
        )
    ])
