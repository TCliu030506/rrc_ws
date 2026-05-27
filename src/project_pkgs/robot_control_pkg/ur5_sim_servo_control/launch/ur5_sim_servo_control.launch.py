from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_pose_topic = LaunchConfiguration('input_pose_topic')
    servo_twist_topic = LaunchConfiguration('servo_twist_topic')
    planning_frame = LaunchConfiguration('planning_frame')
    ee_frame = LaunchConfiguration('ee_frame')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    linear_gain = LaunchConfiguration('linear_gain')
    angular_gain = LaunchConfiguration('angular_gain')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    position_deadband = LaunchConfiguration('position_deadband')
    orientation_deadband = LaunchConfiguration('orientation_deadband')
    publish_zero_when_idle = LaunchConfiguration('publish_zero_when_idle')

    bridge_node = Node(
        package='ur5_sim_servo_control',
        executable='ur5_pose_target_servo_bridge',
        name='ur5_pose_target_servo_bridge',
        output='screen',
        parameters=[{
            'input_pose_topic': input_pose_topic,
            'servo_twist_topic': servo_twist_topic,
            'planning_frame': planning_frame,
            'ee_frame': ee_frame,
            'control_rate_hz': control_rate_hz,
            'linear_gain': linear_gain,
            'angular_gain': angular_gain,
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
            'position_deadband': position_deadband,
            'orientation_deadband': orientation_deadband,
            'publish_zero_when_idle': publish_zero_when_idle,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_pose_topic', default_value='/admittance/cmd_pose'),
        DeclareLaunchArgument('servo_twist_topic', default_value='/servo_node/delta_twist_cmds'),
        DeclareLaunchArgument('planning_frame', default_value='base_link'),
        DeclareLaunchArgument('ee_frame', default_value='tool0'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('linear_gain', default_value='1.0'),
        DeclareLaunchArgument('angular_gain', default_value='1.0'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.15'),
        DeclareLaunchArgument('max_angular_velocity', default_value='0.75'),
        DeclareLaunchArgument('position_deadband', default_value='0.002'),
        DeclareLaunchArgument('orientation_deadband', default_value='0.01'),
        DeclareLaunchArgument('publish_zero_when_idle', default_value='true'),
        bridge_node,
    ])
