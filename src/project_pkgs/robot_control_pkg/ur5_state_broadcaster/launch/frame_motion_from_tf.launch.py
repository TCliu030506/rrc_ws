from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_frame', default_value='base'),
        DeclareLaunchArgument('target_frame', default_value='tool0'),
        DeclareLaunchArgument('output_pose_topic', default_value='/tool0/pose'),
        DeclareLaunchArgument('output_twist_topic', default_value='/tool0/twist'),
        DeclareLaunchArgument('output_accel_topic', default_value='/tool0/accel'),
        DeclareLaunchArgument('publish_rate', default_value='125.0'),
        DeclareLaunchArgument('tf_lookup_timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('express_in_target_frame', default_value='true'),
        DeclareLaunchArgument('min_dt', default_value='0.0001'),
        DeclareLaunchArgument('max_dt', default_value='0.2'),
        DeclareLaunchArgument('velocity_filter_tau', default_value='0.02'),
        DeclareLaunchArgument('accel_filter_tau', default_value='0.05'),
        DeclareLaunchArgument('max_linear_speed', default_value='5.0'),
        DeclareLaunchArgument('max_angular_speed', default_value='20.0'),
        DeclareLaunchArgument('max_linear_accel', default_value='30.0'),
        DeclareLaunchArgument('max_angular_accel', default_value='80.0'),
        Node(
            package='ur5_state_broadcaster',
            executable='frame_motion_from_tf',
            name='frame_motion_from_tf_node',
            output='screen',
            parameters=[{
                'source_frame': LaunchConfiguration('source_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
                'output_pose_topic': LaunchConfiguration('output_pose_topic'),
                'output_twist_topic': LaunchConfiguration('output_twist_topic'),
                'output_accel_topic': LaunchConfiguration('output_accel_topic'),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'),
                    value_type=float,
                ),
                'tf_lookup_timeout_sec': ParameterValue(
                    LaunchConfiguration('tf_lookup_timeout_sec'),
                    value_type=float,
                ),
                'express_in_target_frame': ParameterValue(
                    LaunchConfiguration('express_in_target_frame'),
                    value_type=bool,
                ),
                'min_dt': ParameterValue(LaunchConfiguration('min_dt'), value_type=float),
                'max_dt': ParameterValue(LaunchConfiguration('max_dt'), value_type=float),
                'velocity_filter_tau': ParameterValue(
                    LaunchConfiguration('velocity_filter_tau'),
                    value_type=float,
                ),
                'accel_filter_tau': ParameterValue(
                    LaunchConfiguration('accel_filter_tau'),
                    value_type=float,
                ),
                'max_linear_speed': ParameterValue(
                    LaunchConfiguration('max_linear_speed'),
                    value_type=float,
                ),
                'max_angular_speed': ParameterValue(
                    LaunchConfiguration('max_angular_speed'),
                    value_type=float,
                ),
                'max_linear_accel': ParameterValue(
                    LaunchConfiguration('max_linear_accel'),
                    value_type=float,
                ),
                'max_angular_accel': ParameterValue(
                    LaunchConfiguration('max_angular_accel'),
                    value_type=float,
                ),
            }],
        ),
    ])
