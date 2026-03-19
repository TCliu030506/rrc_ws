from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    limited = LaunchConfiguration("limited")
    transmission_hw_interface = LaunchConfiguration("transmission_hw_interface")

    xacro_normal = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([FindPackageShare("robot_admittance_control"), "ur_description", "urdf", "ur5_ft_robot.urdf.xacro"]),
        " transmission_hw_interface:=", transmission_hw_interface,
    ])
    xacro_limited = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([FindPackageShare("robot_admittance_control"), "ur_description", "urdf", "ur5_joint_limited_robot.urdf.xacro"]),
        " transmission_hw_interface:=", transmission_hw_interface,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("limited", default_value="false"),
        DeclareLaunchArgument("transmission_hw_interface", default_value="hardware_interface/PositionJointInterface"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": ParameterValue(xacro_normal, value_type=str)}],
            condition=UnlessCondition(limited),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": ParameterValue(xacro_limited, value_type=str)}],
            condition=IfCondition(limited),
        ),
    ])
