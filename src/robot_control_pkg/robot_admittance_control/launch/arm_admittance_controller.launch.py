from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_topic = LaunchConfiguration("pose_topic_arm")
    twist_topic = LaunchConfiguration("twist_topic_arm")
    wrench_ext_topic = LaunchConfiguration("wrench_ext_topic_arm")
    wrench_ctr_topic = LaunchConfiguration("wrench_ctr_topic_arm")
    cmd_topic = LaunchConfiguration("cmd_topic_arm")
    pose_cmd_topic = LaunchConfiguration("pose_cmd_topic_arm")
    desired_pose_topic = LaunchConfiguration("desired_pose_topic")
    desired_twist_topic = LaunchConfiguration("desired_twist_topic")
    desired_accel_topic = LaunchConfiguration("desired_accel_topic")
    enable_output_smoothing = LaunchConfiguration("enable_output_smoothing")
    twist_smoothing_alpha_linear = LaunchConfiguration("twist_smoothing_alpha_linear")
    twist_smoothing_alpha_angular = LaunchConfiguration("twist_smoothing_alpha_angular")
    pose_smoothing_alpha_linear = LaunchConfiguration("pose_smoothing_alpha_linear")
    pose_smoothing_alpha_angular = LaunchConfiguration("pose_smoothing_alpha_angular")
    params_file = LaunchConfiguration("admittance_params_file")

    return LaunchDescription([
        DeclareLaunchArgument("pose_topic_arm", default_value="/UR5/ee_pose"),
        DeclareLaunchArgument("twist_topic_arm", default_value="/UR5/ee_twist"),
        DeclareLaunchArgument("wrench_ext_topic_arm", default_value="/external_force_torque_wrench"),
        DeclareLaunchArgument("wrench_ctr_topic_arm", default_value="/arm_admittance_control/control_wrench"),
        DeclareLaunchArgument("cmd_topic_arm", default_value="/UR5/desired_twist"),
        DeclareLaunchArgument("pose_cmd_topic_arm", default_value="/arm_desired_pose"),
        DeclareLaunchArgument("desired_pose_topic", default_value="/desired_pose"),
        DeclareLaunchArgument("desired_twist_topic", default_value="/desired_twist"),
        DeclareLaunchArgument("desired_accel_topic", default_value="/desired_accel"),
        DeclareLaunchArgument("enable_output_smoothing", default_value="true"),
        DeclareLaunchArgument("twist_smoothing_alpha_linear", default_value="0.25"),
        DeclareLaunchArgument("twist_smoothing_alpha_angular", default_value="0.20"),
        DeclareLaunchArgument("pose_smoothing_alpha_linear", default_value="0.25"),
        DeclareLaunchArgument("pose_smoothing_alpha_angular", default_value="0.20"),
        DeclareLaunchArgument(
            "admittance_params_file",
            default_value="/home/liutiancheng/Lab_WS/zzrobot_ws/src/robot_control_pkg/robot_admittance_control/config/admittance_params_ros2.yaml",
        ),
        Node(
            package="robot_admittance_control",
            executable="admittance_controller_node",
            name="admittance_controller_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "topic_arm_pose": pose_topic,
                    "topic_arm_twist": twist_topic,
                    "topic_external_wrench": wrench_ext_topic,
                    "topic_control_wrench": wrench_ctr_topic,
                    "topic_arm_command": cmd_topic,
                    "topic_arm_pose_command": pose_cmd_topic,
                    "topic_desired_pose": desired_pose_topic,
                    "topic_desired_twist": desired_twist_topic,
                    "topic_desired_accel": desired_accel_topic,
                    "enable_output_smoothing": enable_output_smoothing,
                    "twist_smoothing_alpha_linear": twist_smoothing_alpha_linear,
                    "twist_smoothing_alpha_angular": twist_smoothing_alpha_angular,
                    "pose_smoothing_alpha_linear": pose_smoothing_alpha_linear,
                    "pose_smoothing_alpha_angular": pose_smoothing_alpha_angular,
                },
            ],
        ),
    ])
