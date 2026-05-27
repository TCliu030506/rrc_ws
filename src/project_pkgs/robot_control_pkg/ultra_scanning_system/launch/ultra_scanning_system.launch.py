from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Frames
BASE_FRAME = 'base'
TOOL_FRAME = 'tool0'
ASM_EE_FRAME = 'asm_ee_site'

# Topics
RAW_WRENCH_TOPIC = '/external_force_torque_wrench'
COMPENSATED_WRENCH_TOPIC = '/external_force_torque_wrench_compensated'
GRAVITY_WRENCH_TOPIC = '/external_gravity_compensation_wrench_model'

EE_POSE_TOPIC = '/asm_ee_site/pose'
EE_TWIST_TOPIC = '/asm_ee_site/twist'

DESIRED_POSE_TOPIC = '/scan/desired_pose'
DESIRED_TWIST_TOPIC = '/scan/desired_twist'
DESIRED_ACCEL_TOPIC = '/scan/desired_accel'

ASM_EE_CMD_POSE_TOPIC = '/admittance/asm_ee_cmd_pose'
TOOL_CMD_POSE_TOPIC = '/arm_desired_pose_tool0'
CONTROL_WRENCH_TOPIC = '/arm_admittance_control/control_wrench'


def _launch_file(package_name: str, *path_parts: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare(package_name),
            *path_parts,
        ])
    )


def generate_launch_description():
    global_log_level = LaunchConfiguration('global_log_level')
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    forcesensorport = LaunchConfiguration('forcesensorport')
    enable_admittance = LaunchConfiguration('enable_admittance')

    hardware_tf_launch = IncludeLaunchDescription(
        _launch_file(
            'ultra_scanning_system',
            'launch',
            'ultra_scanning_hardware_tf.launch.py',
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': 'false',
            'asm_parent_frame': TOOL_FRAME,
            'asm_tf_publish_rate': '50.0',
        }.items(),
    )

    force_sensor_launch = IncludeLaunchDescription(
        _launch_file('force_sensor', 'launch', 'force_sensor_axis_6.launch.py'),
        launch_arguments={
            'forcesensorport': forcesensorport,
            'pubrate': '200',
            'baudrate': '115200',
            'frame_id': 'asm_force_sensor_link',
            'topic_name': RAW_WRENCH_TOPIC,
            'auto_zero': 'false',
        }.items(),
    )

    gravity_compensation_node = Node(
        package='tool_gravity_compensation',
        executable='gravity_compensation_node',
        name='gravity_compensation_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('tool_gravity_compensation'),
                'config',
                'gravity_compensation_params_v2.yaml',
            ]),
            {
                'wrench_in_topic': RAW_WRENCH_TOPIC,
                'wrench_out_topic': COMPENSATED_WRENCH_TOPIC,
                'gravity_wrench_topic': GRAVITY_WRENCH_TOPIC,
            },
        ],
    )

    ee_state_from_tf_node = Node(
        package='ultra_scanning_sim',
        executable='ee_state_from_tf_node',
        name='asm_ee_state_from_tf_node',
        output='screen',
        parameters=[{
            'source_frame': BASE_FRAME,
            'target_frame': ASM_EE_FRAME,
            'output_pose_topic': EE_POSE_TOPIC,
            'output_twist_topic': EE_TWIST_TOPIC,
            'publish_rate': 50.0,
            'max_angular_speed': 10.0,
        }],
    )

    current_pose_hold_node = Node(
        package='robot_trajectory_planner',
        executable='current_pose_hold_node',
        name='current_pose_hold_node',
        output='screen',
        parameters=[{
            'current_pose_topic': EE_POSE_TOPIC,
            'topic_desired_pose': DESIRED_POSE_TOPIC,
            'topic_desired_twist': DESIRED_TWIST_TOPIC,
            'topic_desired_accel': DESIRED_ACCEL_TOPIC,
            'publish_rate': 20.0,
        }],
    )

    admittance_controller_launch = IncludeLaunchDescription(
        _launch_file(
            'robot_admittance_control',
            'launch',
            'arm_admittance_controller.launch.py',
        ),
        launch_arguments={
            'pose_topic_arm': EE_POSE_TOPIC,
            'twist_topic_arm': EE_TWIST_TOPIC,
            'wrench_ext_topic_arm': COMPENSATED_WRENCH_TOPIC,
            'wrench_ctr_topic_arm': CONTROL_WRENCH_TOPIC,
            'cmd_topic_arm': '/UR5/desired_twist',
            'pose_cmd_topic_arm': ASM_EE_CMD_POSE_TOPIC,
            'desired_pose_topic': DESIRED_POSE_TOPIC,
            'desired_twist_topic': DESIRED_TWIST_TOPIC,
            'desired_accel_topic': DESIRED_ACCEL_TOPIC,
            'enable_output_smoothing': 'true',
            'twist_smoothing_alpha_linear': '0.03',
            'twist_smoothing_alpha_angular': '0.01',
            'pose_smoothing_alpha_linear': '0.03',
            'pose_smoothing_alpha_angular': '0.01',
            'admittance_params_file': PathJoinSubstitution([
                FindPackageShare('robot_admittance_control'),
                'config',
                'admittance_params_ros2.yaml',
            ]),
        }.items(),
    )

    asm_ee_command_transform_node = Node(
        package='ultra_scanning_system',
        executable='asm_ee_command_transform',
        name='asm_ee_command_transform_node',
        output='screen',
        parameters=[{
            'input_pose_topic': ASM_EE_CMD_POSE_TOPIC,
            'output_pose_topic': TOOL_CMD_POSE_TOPIC,
            'control_base_frame': BASE_FRAME,
            'tool_frame': TOOL_FRAME,
            'controlled_frame': ASM_EE_FRAME,
            'tf_lookup_timeout_sec': 0.05,
        }],
    )

    rtde_servol_pose_controller_launch = IncludeLaunchDescription(
        _launch_file(
            'ur5_rtde_control',
            'launch',
            'rtde_servol_pose_controller.launch.py',
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'topic_cmd_pose': TOOL_CMD_POSE_TOPIC,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('global_log_level', default_value='WARN'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('forcesensorport', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('enable_admittance', default_value='true'),
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_SEVERITY_THRESHOLD',
            global_log_level,
        ),
        TimerAction(period=0.0, actions=[force_sensor_launch]),
        TimerAction(period=1.2, actions=[hardware_tf_launch]),
        TimerAction(
            period=2.5,
            condition=IfCondition(enable_admittance),
            actions=[gravity_compensation_node],
        ),
        TimerAction(period=3.0, actions=[ee_state_from_tf_node]),
        TimerAction(period=3.5, actions=[current_pose_hold_node]),
        TimerAction(
            period=3.2,
            condition=IfCondition(enable_admittance),
            actions=[asm_ee_command_transform_node],
        ),
        TimerAction(
            period=3.2,
            condition=IfCondition(enable_admittance),
            actions=[rtde_servol_pose_controller_launch],
        ),
        TimerAction(
            period=4.0,
            condition=IfCondition(enable_admittance),
            actions=[admittance_controller_launch],
        ),
    ])
