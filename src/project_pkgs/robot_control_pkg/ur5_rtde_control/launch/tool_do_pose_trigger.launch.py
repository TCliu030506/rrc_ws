from launch import LaunchDescription
from launch_ros.actions import Node


ROBOT_IP = '192.168.1.102'
POSE_TOPIC = '/asm_ee_site/pose'
TOOL_DO_INDEX = 0
TRIGGER_DISTANCE = 0.0001
PULSE_WIDTH_SEC = 0.001
RECORDS_FILE = 'tool_do_pose_triggers.csv'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5_rtde_control',
            executable='tool_do_pose_trigger_node',
            name='tool_do_pose_trigger_node',
            output='screen',
            parameters=[{
                'robot_ip': ROBOT_IP,
                'pose_topic': POSE_TOPIC,
                'tool_do_index': TOOL_DO_INDEX,
                'trigger_distance': TRIGGER_DISTANCE,
                'pulse_width_sec': PULSE_WIDTH_SEC,
                'records_file': RECORDS_FILE,
            }],
        ),
    ])
