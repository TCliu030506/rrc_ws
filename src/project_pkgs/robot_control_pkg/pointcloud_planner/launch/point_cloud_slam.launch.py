from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pcl_cloudslam_node = Node(
        package='pointcloud_planner',
        executable='pcl_cloudslam',
        output='screen',
        parameters=[{
            'planning_mode': 'cylinder_preplan',
            'cylinder_view_distance_m': 0.35,
            'probe_length_m': 0.18,
            'probe_width_m': 0.075,
            'probe_boundary_margin_m': 0.0,
        }],
    )

    return LaunchDescription([
        TimerAction(period=0.0, actions=[pcl_cloudslam_node]),
    ])
