from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_desired_pose = LaunchConfiguration('topic_desired_pose')
    topic_desired_twist = LaunchConfiguration('topic_desired_twist')
    topic_desired_accel = LaunchConfiguration('topic_desired_accel')

    return LaunchDescription([
        DeclareLaunchArgument('topic_desired_pose', default_value='/scan/desired_pose'),
        DeclareLaunchArgument('topic_desired_twist', default_value='/scan/desired_twist'),
        DeclareLaunchArgument('topic_desired_accel', default_value='/scan/desired_accel'),
        DeclareLaunchArgument('publish_rate', default_value='125.0'),
        DeclareLaunchArgument('scan_center', default_value='[0.45, 0.0, 0.45]'),
        DeclareLaunchArgument('scan_length_x', default_value='0.20'),
        DeclareLaunchArgument('scan_width_y', default_value='0.10'),
        DeclareLaunchArgument('scan_speed', default_value='0.001'),
        DeclareLaunchArgument('line_spacing', default_value='0.01'),
        DeclareLaunchArgument('line_turnaround_pause', default_value='0.15'),
        DeclareLaunchArgument('orientation_xyzw', default_value='[0.0, 0.0, 0.0, 1.0]'),
        Node(
            package='robot_trajectory_planner',
            executable='scan_raster_trajectory_node',
            name='scan_raster_trajectory_node',
            output='screen',
            parameters=[{
                'topic_desired_pose': topic_desired_pose,
                'topic_desired_twist': topic_desired_twist,
                'topic_desired_accel': topic_desired_accel,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'scan_center': LaunchConfiguration('scan_center'),
                'scan_length_x': LaunchConfiguration('scan_length_x'),
                'scan_width_y': LaunchConfiguration('scan_width_y'),
                'scan_speed': LaunchConfiguration('scan_speed'),
                'line_spacing': LaunchConfiguration('line_spacing'),
                'line_turnaround_pause': LaunchConfiguration('line_turnaround_pause'),
                'orientation_xyzw': LaunchConfiguration('orientation_xyzw'),
            }],
        ),
    ])
