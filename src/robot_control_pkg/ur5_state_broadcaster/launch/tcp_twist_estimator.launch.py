from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_pose_topic', default_value='/tcp_pose_broadcaster/pose'),
        DeclareLaunchArgument('output_pose_topic', default_value='/UR5/ee_pose'),
        DeclareLaunchArgument('output_twist_topic', default_value='/UR5/ee_twist'),
        DeclareLaunchArgument('min_dt', default_value='0.0001'),
        DeclareLaunchArgument('max_angular_speed', default_value='10.0'),
        Node(
            package='ur5_state_broadcaster',
            executable='tcp_twist_estimator',
            name='tcp_twist_estimator',
            output='screen',
            parameters=[{
                'input_pose_topic': LaunchConfiguration('input_pose_topic'),
                'output_pose_topic': LaunchConfiguration('output_pose_topic'),
                'output_twist_topic': LaunchConfiguration('output_twist_topic'),
                'min_dt': LaunchConfiguration('min_dt'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            }],
        ),
    ])
