from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    trajectory_planner = LaunchConfiguration('trajectory_planner')
    path_map_file = LaunchConfiguration('path_map_file')
    path_map_publish_rate = LaunchConfiguration('path_map_publish_rate')
    path_map_max_linear_speed = LaunchConfiguration('path_map_max_linear_speed')
    path_map_max_angular_speed = LaunchConfiguration('path_map_max_angular_speed')
    path_map_loop_path = LaunchConfiguration('path_map_loop_path')
    path_map_enable_resampling = LaunchConfiguration('path_map_enable_resampling')
    path_map_max_linear_step = LaunchConfiguration('path_map_max_linear_step')
    path_map_max_angular_step = LaunchConfiguration('path_map_max_angular_step')
    contact_scan_force_axis = LaunchConfiguration('contact_scan_force_axis')
    contact_scan_force_axis_sign = LaunchConfiguration(
        'contact_scan_force_axis_sign',
    )
    contact_scan_approach_axis_sign = LaunchConfiguration(
        'contact_scan_approach_axis_sign',
    )
    contact_scan_contact_force_threshold = LaunchConfiguration(
        'contact_scan_contact_force_threshold',
    )
    contact_scan_target_contact_force = LaunchConfiguration(
        'contact_scan_target_contact_force',
    )
    contact_scan_max_contact_force = LaunchConfiguration(
        'contact_scan_max_contact_force',
    )
    contact_scan_max_search_distance = LaunchConfiguration(
        'contact_scan_max_search_distance',
    )
    contact_scan_pre_contact_speed = LaunchConfiguration(
        'contact_scan_pre_contact_speed',
    )
    contact_scan_retract_distance = LaunchConfiguration(
        'contact_scan_retract_distance',
    )

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

    dynamic_gravity_compensation_node = Node(
        package='tool_gravity_compensation',
        executable='dynamic_gravity_compensation_node',
        name='dynamic_gravity_compensation_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('tool_gravity_compensation'),
                'config',
                'gravity_compensation_dynamic_params.yaml',
            ]),
            {
                'wrench_in_topic': RAW_WRENCH_TOPIC,
                'wrench_out_topic': COMPENSATED_WRENCH_TOPIC,
                'gravity_wrench_topic': GRAVITY_WRENCH_TOPIC,
            },
        ],
    )

    force_sensor_motion_node = Node(
        package='ur5_state_broadcaster',
        executable='frame_motion_from_tf',
        name='force_sensor_motion_from_tf_node',
        output='screen',
        parameters=[{
            'source_frame': BASE_FRAME,
            'target_frame': 'asm_force_sensor_link',
            'output_pose_topic': '/asm_force_sensor_link/pose',
            'output_twist_topic': '/asm_force_sensor_link/twist',
            'output_accel_topic': '/asm_force_sensor_link/accel',
            'publish_rate': 125.0,
            'express_in_target_frame': True,
        }],
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

    path_map_trajectory_node = Node(
        package='robot_trajectory_planner',
        executable='path_map_trajectory_node',
        name='path_map_trajectory_node',
        output='screen',
        parameters=[{
            'current_pose_topic': EE_POSE_TOPIC,
            'topic_desired_pose': DESIRED_POSE_TOPIC,
            'topic_desired_twist': DESIRED_TWIST_TOPIC,
            'topic_desired_accel': DESIRED_ACCEL_TOPIC,
            'path_file': path_map_file,
            'publish_rate': ParameterValue(path_map_publish_rate, value_type=float),
            'max_linear_speed': ParameterValue(
                path_map_max_linear_speed,
                value_type=float,
            ),
            'max_angular_speed': ParameterValue(
                path_map_max_angular_speed,
                value_type=float,
            ),
            'loop_path': ParameterValue(path_map_loop_path, value_type=bool),
            'enable_path_resampling': ParameterValue(
                path_map_enable_resampling,
                value_type=bool,
            ),
            'max_path_linear_step': ParameterValue(
                path_map_max_linear_step,
                value_type=float,
            ),
            'max_path_angular_step': ParameterValue(
                path_map_max_angular_step,
                value_type=float,
            ),
        }],
    )

    contact_scan_trajectory_node = Node(
        package='ultra_scanning_system',
        executable='contact_scan_trajectory_node',
        name='contact_scan_trajectory_node',
        output='screen',
        parameters=[{
            'current_pose_topic': EE_POSE_TOPIC,
            'compensated_wrench_topic': COMPENSATED_WRENCH_TOPIC,
            'topic_desired_pose': DESIRED_POSE_TOPIC,
            'topic_desired_twist': DESIRED_TWIST_TOPIC,
            'topic_desired_accel': DESIRED_ACCEL_TOPIC,
            'topic_control_wrench': CONTROL_WRENCH_TOPIC,
            'control_wrench_frame': ASM_EE_FRAME,
            'path_file': path_map_file,
            'publish_rate': ParameterValue(
                path_map_publish_rate,
                value_type=float,
            ),
            'max_linear_speed': ParameterValue(
                path_map_max_linear_speed,
                value_type=float,
            ),
            'max_angular_speed': ParameterValue(
                path_map_max_angular_speed,
                value_type=float,
            ),
            'max_path_linear_step': ParameterValue(
                path_map_max_linear_step,
                value_type=float,
            ),
            'max_path_angular_step': ParameterValue(
                path_map_max_angular_step,
                value_type=float,
            ),
            'force_axis': contact_scan_force_axis,
            'force_axis_sign': ParameterValue(
                contact_scan_force_axis_sign,
                value_type=float,
            ),
            'approach_axis_sign': ParameterValue(
                contact_scan_approach_axis_sign,
                value_type=float,
            ),
            'contact_force_threshold': ParameterValue(
                contact_scan_contact_force_threshold,
                value_type=float,
            ),
            'target_contact_force': ParameterValue(
                contact_scan_target_contact_force,
                value_type=float,
            ),
            'max_contact_force': ParameterValue(
                contact_scan_max_contact_force,
                value_type=float,
            ),
            'max_search_distance': ParameterValue(
                contact_scan_max_search_distance,
                value_type=float,
            ),
            'pre_contact_speed': ParameterValue(
                contact_scan_pre_contact_speed,
                value_type=float,
            ),
            'retract_distance': ParameterValue(
                contact_scan_retract_distance,
                value_type=float,
            ),
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
        DeclareLaunchArgument(
            'trajectory_planner',
            default_value='current_pose_hold',
            description='Trajectory source: current_pose_hold, path_map, or contact_scan.',
        ),
        DeclareLaunchArgument(
            'path_map_file',
            default_value='',
            description=(
                'Path-map file for path_map_trajectory_node. Empty uses '
                'robot_trajectory_planner/share/data/path_map.txt.'
            ),
        ),
        DeclareLaunchArgument('path_map_publish_rate', default_value='50.0'),
        DeclareLaunchArgument('path_map_max_linear_speed', default_value='0.005'),
        DeclareLaunchArgument('path_map_max_angular_speed', default_value='0.05'),
        DeclareLaunchArgument('path_map_loop_path', default_value='false'),
        DeclareLaunchArgument('path_map_enable_resampling', default_value='true'),
        DeclareLaunchArgument('path_map_max_linear_step', default_value='0.001'),
        DeclareLaunchArgument('path_map_max_angular_step', default_value='0.01'),
        DeclareLaunchArgument('contact_scan_force_axis', default_value='z'),
        DeclareLaunchArgument('contact_scan_force_axis_sign', default_value='1.0'),
        DeclareLaunchArgument(
            'contact_scan_approach_axis_sign',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'contact_scan_contact_force_threshold',
            default_value='2.0',
        ),
        DeclareLaunchArgument(
            'contact_scan_target_contact_force',
            default_value='6.0',
        ),
        DeclareLaunchArgument(
            'contact_scan_max_contact_force',
            default_value='12.0',
        ),
        DeclareLaunchArgument(
            'contact_scan_max_search_distance',
            default_value='0.04',
        ),
        DeclareLaunchArgument(
            'contact_scan_pre_contact_speed',
            default_value='0.002',
        ),
        DeclareLaunchArgument(
            'contact_scan_retract_distance',
            default_value='0.04',
        ),
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_SEVERITY_THRESHOLD',
            global_log_level,
        ),
        TimerAction(period=0.0, actions=[force_sensor_launch]),
        TimerAction(period=1.2, actions=[hardware_tf_launch]),
        TimerAction(
            period=2.0,
            condition=IfCondition(enable_admittance),
            actions=[force_sensor_motion_node],
        ),
        # TimerAction(
        #     period=2.5,
        #     condition=IfCondition(enable_admittance),
        #     actions=[gravity_compensation_node],
        # ),
        TimerAction(
            period=2.5,
            condition=IfCondition(enable_admittance),
            actions=[dynamic_gravity_compensation_node],
        ),
        TimerAction(period=3.0, actions=[ee_state_from_tf_node]),
        TimerAction(
            period=4.5,
            condition=IfCondition(PythonExpression([
                "'", trajectory_planner, "' == 'current_pose_hold'",
            ])),
            actions=[current_pose_hold_node],
        ),
        TimerAction(
            period=4.5,
            condition=IfCondition(PythonExpression([
                "'", trajectory_planner, "' == 'path_map'",
            ])),
            actions=[path_map_trajectory_node],
        ),
        TimerAction(
            period=4.5,
            condition=IfCondition(PythonExpression([
                "'", trajectory_planner, "' == 'contact_scan'",
            ])),
            actions=[contact_scan_trajectory_node],
        ),
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
