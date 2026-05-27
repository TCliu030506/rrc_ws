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
        DeclareLaunchArgument('center_x', default_value='1.00'),
        DeclareLaunchArgument('center_z', default_value='0.50'),
        DeclareLaunchArgument('radius', default_value='0.50'),
        DeclareLaunchArgument('y_start', default_value='0.0'),
        DeclareLaunchArgument('y_end', default_value='0.20'),
        DeclareLaunchArgument('y_step', default_value='0.05'),
        DeclareLaunchArgument('arc_points', default_value='10000'),
        DeclareLaunchArgument('arc_start_deg', default_value='160.0'),
        DeclareLaunchArgument('arc_end_deg', default_value='200.0'),
        DeclareLaunchArgument('orientation_mode', default_value='fixed'), # options: 'fixed', 'normal'

        Node(
            package='robot_trajectory_planner',
            executable='scan_cylindrical_surface_node',
            name='scan_cylindrical_surface_node',
            output='screen',
            parameters=[{
                'topic_desired_pose': topic_desired_pose,
                'topic_desired_twist': topic_desired_twist,
                'topic_desired_accel': topic_desired_accel,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'center_x': LaunchConfiguration('center_x'),
                'center_z': LaunchConfiguration('center_z'),
                'radius': LaunchConfiguration('radius'),
                'y_start': LaunchConfiguration('y_start'),
                'y_end': LaunchConfiguration('y_end'),
                'y_step': LaunchConfiguration('y_step'),
                'arc_points': LaunchConfiguration('arc_points'),
                'arc_start_deg': LaunchConfiguration('arc_start_deg'),
                'arc_end_deg': LaunchConfiguration('arc_end_deg'),
                'orientation_mode': LaunchConfiguration('orientation_mode'),
            }],
        ),
    ])
