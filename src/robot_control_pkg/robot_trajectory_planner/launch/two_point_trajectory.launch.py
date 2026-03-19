from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_trajectory_planner',
            executable='two_point_trajectory_node',
            name='two_point_trajectory_node',
            output='screen',
            parameters=[{
                'topic_desired_pose': '/desired_pose',
                'topic_desired_twist': '/desired_twist',
                'topic_desired_accel': '/desired_accel',
                'publish_rate': 125.0,
                'cycle_period': 8.0,
                'point_a': [0.360, 0.100, 0.450],
                'point_b': [0.360, -0.200, 0.450],
                'orientation_xyzw': [0.0, 0.0, 0.0, 1.0],
            }],
        )
    ])
