from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 全局开关：后续调试时优先修改这里，避免通过 ros2 launch 参数层层传递。
GLOBAL_LOG_LEVEL = 'WARN'
ROBOT_IP = '192.168.1.102'
ENABLE_ADMITTANCE = True
TRAJECTORY_PLANNER = 'contact_scan'  # current_pose_hold, path_map, contact_scan

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
SERVO_ASM_EE_CMD_POSE_TOPIC = '/servo/asm_ee_cmd_pose'
TOOL_CMD_POSE_TOPIC = '/arm_desired_pose_tool0'
CONTROL_WRENCH_TOPIC = '/arm_admittance_control/control_wrench'

# Path-map trajectory 参数
PATH_MAP_PUBLISH_RATE = 200.0
PATH_MAP_MAX_LINEAR_SPEED = 0.01
PATH_MAP_MAX_ANGULAR_SPEED = 0.05
PATH_MAP_LOOP_PATH = False
PATH_MAP_ENABLE_RESAMPLING = True
PATH_MAP_MAX_LINEAR_STEP = 0.0005
PATH_MAP_MAX_ANGULAR_STEP = 0.01

# Contact scan 参数
CONTACT_SCAN_FORCE_AXIS = 'z'
CONTACT_SCAN_FORCE_AXIS_SIGN = -1.0
CONTACT_SCAN_APPROACH_AXIS_SIGN = 1.0
CONTACT_SCAN_CONTACT_FORCE_THRESHOLD = 5.0
CONTACT_SCAN_TARGET_CONTACT_FORCE = 25.0
CONTACT_SCAN_FORCE_RAMP_RATE = 0.5
CONTACT_SCAN_ZERO_TORQUE_RX_RY_ENABLED = True
CONTACT_SCAN_TARGET_TORQUE_RX = 0.0
CONTACT_SCAN_TARGET_TORQUE_RY = 0.0
CONTACT_SCAN_TORQUE_RAMP_RATE = 0.05
CONTACT_SCAN_SETTLE_DURATION = 0.5
CONTACT_SCAN_SETTLE_FORCE_TOLERANCE = 8.0
CONTACT_SCAN_MAX_CONTACT_FORCE = 50.0
CONTACT_SCAN_MAX_SEARCH_DISTANCE = 0.5
CONTACT_SCAN_APPROACH_LINEAR_SPEED = 0.05
CONTACT_SCAN_PRE_CONTACT_SPEED = 0.0001
CONTACT_SCAN_RETRACT_DISTANCE = 0.05
CONTACT_SCAN_STATE_TOPIC = '/contact_scan/state'
SCAN_POSE_MUX_ADMITTANCE_BLEND_DURATION = 0.001

# 一体化 servoL 执行节点参数：内部完成 asm_ee_site -> tool0 的 TF 转换。
SERVO_ENABLE_DEBUG_POSE_PUBLISH = True
SERVO_SPEED = 0.15
SERVO_ACCELERATION = 0.1
SERVO_LOOKAHEAD_TIME = 0.15
SERVO_GAIN = 200.0
SERVO_TF_LOOKUP_TIMEOUT_SEC = 0.05


def _launch_file(package_name: str, *path_parts: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare(package_name),
            *path_parts,
        ])
    )


def generate_launch_description():
    path_map_file = PathJoinSubstitution([
        # FindPackageShare('robot_trajectory_planner'),
        # 'data',
        # 'path_map_offset.txt',
        FindPackageShare('pointcloud_planner'),
        'data/path_planning',
        'path_map_concave.txt',
        # 'path_map.txt',
    ])

    # 节点定义和参数配置
    # 系统初始化节点
    system_init_launch = IncludeLaunchDescription(
        _launch_file(
            'ultra_scanning_system',
            'launch',
            'ultra_scanning_hardware_init.launch.py',
        ),
    )

    # 周期性切换工具 DO0，用于触发超声采集。
    periodic_tool_io_demo = Node(
        package='ur5_rtde_control',
        executable='periodic_tool_io_demo',
        name='periodic_tool_io_demo',
        output='screen',
    )

    # 动态重力补偿节点
    dynamic_gravity_compensation_node = Node(
        package='tool_gravity_compensation',
        executable='dynamic_gravity_compensation_node',
        name='dynamic_gravity_compensation_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('tool_gravity_compensation'),
                'config',
                # 'gravity_compensation_dynamic_params.yaml',
                'gravity_compensation_dynamic_v2_merged_params.yaml',
            ]),
            {
                'wrench_in_topic': RAW_WRENCH_TOPIC,
                'wrench_out_topic': COMPENSATED_WRENCH_TOPIC,
                'gravity_wrench_topic': GRAVITY_WRENCH_TOPIC,
            },
        ],
    )
    # 力传感器运动节点（从 TF 计算力传感器位姿\速度\加速度，供惯性力补偿使用）
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

    # 三个轨迹规划节点: 保持当前位置、路径图轨迹、接触扫查轨迹
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
            'publish_rate': 125.0,
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
            'publish_rate': PATH_MAP_PUBLISH_RATE,
            'max_linear_speed': PATH_MAP_MAX_LINEAR_SPEED,
            'max_angular_speed': PATH_MAP_MAX_ANGULAR_SPEED,
            'loop_path': PATH_MAP_LOOP_PATH,
            'enable_path_resampling': PATH_MAP_ENABLE_RESAMPLING,
            'max_path_linear_step': PATH_MAP_MAX_LINEAR_STEP,
            'max_path_angular_step': PATH_MAP_MAX_ANGULAR_STEP,
        }],
    )
    contact_scan_trajectory_node = Node(
        package='ultra_scanning_system',
        executable='contact_scan_trajectory_node',
        name='contact_scan_trajectory_node',
        output='screen',
        parameters=[{
            # contact_scan 节点负责上层阶段状态机：
            # 接近、预接触搜索、接触扫查、撤离和故障停机。
            'current_pose_topic': EE_POSE_TOPIC,
            'compensated_wrench_topic': COMPENSATED_WRENCH_TOPIC,
            'topic_desired_pose': DESIRED_POSE_TOPIC,
            'topic_desired_twist': DESIRED_TWIST_TOPIC,
            'topic_desired_accel': DESIRED_ACCEL_TOPIC,
            'topic_control_wrench': CONTROL_WRENCH_TOPIC,
            'state_topic': CONTACT_SCAN_STATE_TOPIC,
            'control_wrench_frame': ASM_EE_FRAME,
            'path_file': path_map_file,
            'publish_rate': PATH_MAP_PUBLISH_RATE,
            'max_linear_speed': PATH_MAP_MAX_LINEAR_SPEED,
            'approach_linear_speed': CONTACT_SCAN_APPROACH_LINEAR_SPEED,
            'max_angular_speed': PATH_MAP_MAX_ANGULAR_SPEED,
            'max_path_linear_step': PATH_MAP_MAX_LINEAR_STEP,
            'max_path_angular_step': PATH_MAP_MAX_ANGULAR_STEP,
            # 力轴和符号以当前项目实测为准，用于把“压紧力增大”统一为正。
            'force_axis': CONTACT_SCAN_FORCE_AXIS,
            'force_axis_sign': CONTACT_SCAN_FORCE_AXIS_SIGN,
            'approach_axis_sign': CONTACT_SCAN_APPROACH_AXIS_SIGN,
            # 预接触搜索和安全阈值。实机调试时优先降低目标力和搜索速度。
            'contact_force_threshold': CONTACT_SCAN_CONTACT_FORCE_THRESHOLD,
            'target_contact_force': CONTACT_SCAN_TARGET_CONTACT_FORCE,
            'force_ramp_rate': CONTACT_SCAN_FORCE_RAMP_RATE,
            'zero_torque_rx_ry_enabled': CONTACT_SCAN_ZERO_TORQUE_RX_RY_ENABLED,
            'target_torque_rx': CONTACT_SCAN_TARGET_TORQUE_RX,
            'target_torque_ry': CONTACT_SCAN_TARGET_TORQUE_RY,
            'torque_ramp_rate': CONTACT_SCAN_TORQUE_RAMP_RATE,
            'contact_settle_duration': CONTACT_SCAN_SETTLE_DURATION,
            'contact_settle_force_tolerance': CONTACT_SCAN_SETTLE_FORCE_TOLERANCE,
            'max_contact_force': CONTACT_SCAN_MAX_CONTACT_FORCE,
            'max_search_distance': CONTACT_SCAN_MAX_SEARCH_DISTANCE,
            'pre_contact_speed': CONTACT_SCAN_PRE_CONTACT_SPEED,
            'retract_distance': CONTACT_SCAN_RETRACT_DISTANCE,
        }],
    )

    # 动态 admittance 控制器节点
    admittance_controller_node = Node(
        package='robot_admittance_control',
        executable='admittance_controller_node',
        name='admittance_controller_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_admittance_control'),
                'config',
                'admittance_params_ros2.yaml',
            ]),
            {
                'topic_arm_pose': EE_POSE_TOPIC,
                'topic_arm_twist': EE_TWIST_TOPIC,
                'topic_external_wrench': COMPENSATED_WRENCH_TOPIC,
                'topic_control_wrench': CONTROL_WRENCH_TOPIC,
                'topic_arm_command': '/UR5/desired_twist',
                'topic_arm_pose_command': ASM_EE_CMD_POSE_TOPIC,
                'topic_desired_pose': DESIRED_POSE_TOPIC,
                'topic_desired_twist': DESIRED_TWIST_TOPIC,
                'topic_desired_accel': DESIRED_ACCEL_TOPIC,
                'admittance_freeze_state_topic': CONTACT_SCAN_STATE_TOPIC,
                'admittance_freeze_states': ['approach', 'pre_contact'],
            },
        ],
    )
    # 扫描位姿混合节点
    scan_pose_mux_node = Node(
        package='ultra_scanning_system',
        executable='scan_pose_mux',
        name='scan_pose_mux',
        output='screen',
        parameters=[{
            'direct_pose_topic': DESIRED_POSE_TOPIC,
            'admittance_pose_topic': ASM_EE_CMD_POSE_TOPIC,
            'output_pose_topic': SERVO_ASM_EE_CMD_POSE_TOPIC,
            'state_topic': CONTACT_SCAN_STATE_TOPIC,
            'direct_states': ['approach', 'pre_contact'],
            'admittance_blend_duration': SCAN_POSE_MUX_ADMITTANCE_BLEND_DURATION,
        }],
    )
    # 一体化 servoL 执行节点
    rtde_servol_frame_pose_controller_node = Node(  # noqa: F841
        package='ur5_rtde_control',
        executable='rtde_servol_frame_pose_controller_node',
        name='rtde_servol_frame_pose_controller_node',
        output='screen',
        parameters=[{
            'robot_ip': ROBOT_IP,
            'topic_cmd_pose': SERVO_ASM_EE_CMD_POSE_TOPIC,
            # 当前 /servo/asm_ee_cmd_pose 是 Pose，无 header，
            # 因此用参数声明它是 base 坐标系下 asm_ee_site 的目标位姿。
            'input_pose_is_stamped': False,
            'input_pose_frame': BASE_FRAME,
            'base_frame': BASE_FRAME,
            'tool_frame': TOOL_FRAME,
            'controlled_frame': ASM_EE_FRAME,
            'tf_lookup_timeout_sec': SERVO_TF_LOOKUP_TIMEOUT_SEC,
            'speed': SERVO_SPEED,
            'acceleration': SERVO_ACCELERATION,
            'lookahead_time': SERVO_LOOKAHEAD_TIME,
            'gain': SERVO_GAIN,
            'enable_debug_pose_publish': SERVO_ENABLE_DEBUG_POSE_PUBLISH,
            'debug_pose_topic': TOOL_CMD_POSE_TOPIC,
        }],
    )

    # 超声采集触发节点
    frequency_tool_do_trigger_node = Node(
        package='ur5_rtde_control',
        executable='frequency_tool_do_trigger_node',
        name='frequency_tool_do_trigger_node',
        output='screen',
        parameters=[{
            'robot_ip': ROBOT_IP,
            'pose_topic': EE_POSE_TOPIC,
            'state_topic': CONTACT_SCAN_STATE_TOPIC,
            'trigger_state': 'contact_scan',
            'tool_do_index': 0,
            'trigger_frequency_hz': 1.0,
            'pulse_width_sec': 0.4,
            'trigger_count': 1000,
            'records_file': 'frequency_tool_do_triggers.csv',
        }],
    )

    actions = [
        SetEnvironmentVariable(
            'RCUTILS_LOGGING_SEVERITY_THRESHOLD',
            GLOBAL_LOG_LEVEL,
        ),
        # TimerAction(period=0.0, actions=[system_init_launch]), # 启动系统初始化的硬件相关节点
        # TimerAction(period=2.5, actions=[periodic_tool_io_demo]), # 周期性切换工具 DO0，用于触发超声采集。
    ]

    if ENABLE_ADMITTANCE:
        actions.extend([
            # TimerAction(period=0.0, actions=[force_sensor_motion_node]), #暂时不需要惯性力补偿
            TimerAction(period=0.1, actions=[dynamic_gravity_compensation_node]),
            TimerAction(period=0.1, actions=[scan_pose_mux_node]),
            TimerAction(period=0.1, actions=[rtde_servol_frame_pose_controller_node]),
            TimerAction(period=0.5, actions=[admittance_controller_node]),
            TimerAction(period=0.1, actions=[frequency_tool_do_trigger_node]),
        ])

    if TRAJECTORY_PLANNER == 'current_pose_hold':
        actions.append(TimerAction(period=1.5, actions=[current_pose_hold_node]))
    elif TRAJECTORY_PLANNER == 'path_map':
        actions.append(TimerAction(period=1.5, actions=[path_map_trajectory_node]))
    elif TRAJECTORY_PLANNER == 'contact_scan':
        actions.append(TimerAction(period=1.5, actions=[contact_scan_trajectory_node]))
    else:
        raise ValueError(
            "TRAJECTORY_PLANNER must be 'current_pose_hold', 'path_map', or 'contact_scan'"
        )

    return LaunchDescription(actions)
