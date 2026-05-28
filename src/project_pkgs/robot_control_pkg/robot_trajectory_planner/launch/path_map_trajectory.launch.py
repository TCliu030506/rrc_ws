from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    current_pose_topic = LaunchConfiguration('current_pose_topic')
    topic_desired_pose = LaunchConfiguration('topic_desired_pose')
    topic_desired_twist = LaunchConfiguration('topic_desired_twist')
    topic_desired_accel = LaunchConfiguration('topic_desired_accel')
    path_file = LaunchConfiguration('path_file')
    publish_rate = LaunchConfiguration('publish_rate')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    min_segment_duration = LaunchConfiguration('min_segment_duration')
    loop_path = LaunchConfiguration('loop_path')
    enable_path_resampling = LaunchConfiguration('enable_path_resampling')
    max_path_linear_step = LaunchConfiguration('max_path_linear_step')
    max_path_angular_step = LaunchConfiguration('max_path_angular_step')

    return LaunchDescription([
        DeclareLaunchArgument('current_pose_topic', default_value='/asm_ee_site/pose'),
        DeclareLaunchArgument('topic_desired_pose', default_value='/scan/desired_pose'),
        DeclareLaunchArgument('topic_desired_twist', default_value='/scan/desired_twist'),
        DeclareLaunchArgument('topic_desired_accel', default_value='/scan/desired_accel'),
        DeclareLaunchArgument('path_file', default_value=''),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.005'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.05'),
        DeclareLaunchArgument('min_segment_duration', default_value='0.0'),
        DeclareLaunchArgument('loop_path', default_value='false'),
        DeclareLaunchArgument('enable_path_resampling', default_value='true'),
        DeclareLaunchArgument('max_path_linear_step', default_value='0.002'),
        DeclareLaunchArgument('max_path_angular_step', default_value='0.02'),
        Node(
            package='robot_trajectory_planner',
            executable='path_map_trajectory_node',
            name='path_map_trajectory_node',
            output='screen',
            parameters=[{
                'current_pose_topic': current_pose_topic,
                'topic_desired_pose': topic_desired_pose,
                'topic_desired_twist': topic_desired_twist,
                'topic_desired_accel': topic_desired_accel,
                'path_file': path_file,
                'publish_rate': ParameterValue(publish_rate, value_type=float),
                'max_linear_speed': ParameterValue(max_linear_speed, value_type=float),
                'max_angular_speed': ParameterValue(max_angular_speed, value_type=float),
                'min_segment_duration': ParameterValue(
                    min_segment_duration,
                    value_type=float,
                ),
                'loop_path': ParameterValue(loop_path, value_type=bool),
                'enable_path_resampling': ParameterValue(
                    enable_path_resampling,
                    value_type=bool,
                ),
                'max_path_linear_step': ParameterValue(
                    max_path_linear_step,
                    value_type=float,
                ),
                'max_path_angular_step': ParameterValue(
                    max_path_angular_step,
                    value_type=float,
                ),
            }],
        ),
    ])
