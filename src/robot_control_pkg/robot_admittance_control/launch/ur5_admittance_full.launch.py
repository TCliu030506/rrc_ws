from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    global_log_level = LaunchConfiguration('global_log_level')

    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    forcesensorport = LaunchConfiguration('forcesensorport')
    force_pubrate = LaunchConfiguration('force_pubrate')
    force_baudrate = LaunchConfiguration('force_baudrate')
    force_frame_id = LaunchConfiguration('force_frame_id')
    force_topic_name = LaunchConfiguration('force_topic_name')
    force_auto_zero = LaunchConfiguration('force_auto_zero')

    tcp_pose_topic = LaunchConfiguration('tcp_pose_topic')
    ee_pose_topic = LaunchConfiguration('ee_pose_topic')
    ee_twist_topic = LaunchConfiguration('ee_twist_topic')
    twist_min_dt = LaunchConfiguration('twist_min_dt')
    twist_max_angular_speed = LaunchConfiguration('twist_max_angular_speed')

    topic_control_wrench = LaunchConfiguration('topic_control_wrench')
    topic_arm_command = LaunchConfiguration('topic_arm_command')
    topic_arm_pose_command = LaunchConfiguration('topic_arm_pose_command')
    topic_desired_pose = LaunchConfiguration('topic_desired_pose')
    topic_desired_twist = LaunchConfiguration('topic_desired_twist')
    topic_desired_accel = LaunchConfiguration('topic_desired_accel')
    enable_output_smoothing = LaunchConfiguration('enable_output_smoothing')
    twist_smoothing_alpha_linear = LaunchConfiguration('twist_smoothing_alpha_linear')
    twist_smoothing_alpha_angular = LaunchConfiguration('twist_smoothing_alpha_angular')
    pose_smoothing_alpha_linear = LaunchConfiguration('pose_smoothing_alpha_linear')
    pose_smoothing_alpha_angular = LaunchConfiguration('pose_smoothing_alpha_angular')
    enable_topic_monitor = LaunchConfiguration('enable_topic_monitor')

    control_wrench_frame_id = LaunchConfiguration('control_wrench_frame_id')
    control_wrench_publish_rate = LaunchConfiguration('control_wrench_publish_rate')
    control_force_x = LaunchConfiguration('control_force_x')
    control_force_y = LaunchConfiguration('control_force_y')
    control_force_z = LaunchConfiguration('control_force_z')
    control_torque_x = LaunchConfiguration('control_torque_x')
    control_torque_y = LaunchConfiguration('control_torque_y')
    control_torque_z = LaunchConfiguration('control_torque_z')

    speedl_acc = LaunchConfiguration('speedl_acc')
    speedl_time = LaunchConfiguration('speedl_time')
    speedl_publish_rate = LaunchConfiguration('speedl_publish_rate')
    speedl_command_timeout = LaunchConfiguration('speedl_command_timeout')
    speedl_zero_on_timeout = LaunchConfiguration('speedl_zero_on_timeout')
    speedl_send_stop_on_exit = LaunchConfiguration('speedl_send_stop_on_exit')
    speedl_max_linear_speed = LaunchConfiguration('speedl_max_linear_speed')
    speedl_max_angular_speed = LaunchConfiguration('speedl_max_angular_speed')
    speedl_skip_repeated_zero_command = LaunchConfiguration('speedl_skip_repeated_zero_command')
    speedl_zero_command_epsilon = LaunchConfiguration('speedl_zero_command_epsilon')

    admittance_params_file = LaunchConfiguration('admittance_params_file')

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': launch_rviz,
        }.items()
    )

    force_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('force_sensor'),
                'launch',
                'force_sensor_axis_6.launch.py'
            ])
        ),
        launch_arguments={
            'forcesensorport': forcesensorport,
            'pubrate': force_pubrate,
            'baudrate': force_baudrate,
            'frame_id': force_frame_id,
            'topic_name': force_topic_name,
            'auto_zero': force_auto_zero,
        }.items()
    )

    tcp_twist_estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5_state_broadcaster'),
                'launch',
                'tcp_twist_estimator.launch.py'
            ])
        ),
        launch_arguments={
            'input_pose_topic': tcp_pose_topic,
            'output_pose_topic': ee_pose_topic,
            'output_twist_topic': ee_twist_topic,
            'min_dt': twist_min_dt,
            'max_angular_speed': twist_max_angular_speed,
        }.items()
    )

    # desired_trajectory_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('robot_trajectory_planner'),
    #             'launch',
    #             'two_point_trajectory.launch.py'
    #         ])
    #     )
    # )

    desired_trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_trajectory_planner'),
                'launch',
                'teleoperation_desired_trajectory.launch.py'
            ])
        ),
        launch_arguments={
            'topic_master_state': '/phantom/state',
            'topic_ui_control': 'tus_control',
            'topic_desired_pose': topic_desired_pose,
            'topic_desired_twist': topic_desired_twist,
            'topic_desired_accel': topic_desired_accel,
        }.items()
    )

    speedl_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_admittance_control'),
                'launch',
                'twist_to_urscript_speedl.launch.py'
            ])
        ),
        launch_arguments={
            'topic_arm_command': topic_arm_command,
            'speedl_acc': speedl_acc,
            'speedl_time': speedl_time,
            'publish_rate': speedl_publish_rate,
            'command_timeout': speedl_command_timeout,
            'zero_on_timeout': speedl_zero_on_timeout,
            'send_stop_on_exit': speedl_send_stop_on_exit,
            'max_linear_speed': speedl_max_linear_speed,
            'max_angular_speed': speedl_max_angular_speed,
            'skip_repeated_zero_command': speedl_skip_repeated_zero_command,
            'zero_command_epsilon': speedl_zero_command_epsilon,
        }.items()
    )

    control_wrench_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_admittance_control'),
                'launch',
                'control_wrench_publisher.launch.py'
            ])
        ),
        launch_arguments={
            'topic_control_wrench': topic_control_wrench,
            'frame_id': control_wrench_frame_id,
            'publish_rate': control_wrench_publish_rate,
            'force_x': control_force_x,
            'force_y': control_force_y,
            'force_z': control_force_z,
            'torque_x': control_torque_x,
            'torque_y': control_torque_y,
            'torque_z': control_torque_z,
        }.items()
    )

    admittance_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_admittance_control'),
                'launch',
                'arm_admittance_controller.launch.py'
            ])
        ),
        launch_arguments={
            'pose_topic_arm': ee_pose_topic,
            'twist_topic_arm': ee_twist_topic,
            'wrench_ext_topic_arm': force_topic_name,
            'wrench_ctr_topic_arm': topic_control_wrench,
            'cmd_topic_arm': topic_arm_command,
            'pose_cmd_topic_arm': topic_arm_pose_command,
            'desired_pose_topic': topic_desired_pose,
            'desired_twist_topic': topic_desired_twist,
            'desired_accel_topic': topic_desired_accel,
            'enable_output_smoothing': enable_output_smoothing,
            'twist_smoothing_alpha_linear': twist_smoothing_alpha_linear,
            'twist_smoothing_alpha_angular': twist_smoothing_alpha_angular,
            'pose_smoothing_alpha_linear': pose_smoothing_alpha_linear,
            'pose_smoothing_alpha_angular': pose_smoothing_alpha_angular,
            'admittance_params_file': admittance_params_file,
        }.items()
    )

    rtde_servol_pose_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5_rtde_control'),
                'launch',
                'rtde_servol_pose_controller.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'topic_cmd_pose': topic_arm_pose_command,
        }.items()
    )

    flange_to_sensor_tf_node = Node(
        package='robot_admittance_control',
        executable='flange_to_sensor_static_tf',
        name='flange_to_sensor_static_tf'
    )

    rtde_velocity_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5_rtde_control'),
                'launch',
                'rtde_velocity_publisher.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'topic_cmd_vel': topic_arm_command,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('global_log_level', default_value='WARN'),
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', global_log_level),

        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        DeclareLaunchArgument('forcesensorport', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('force_pubrate', default_value='100'),
        DeclareLaunchArgument('force_baudrate', default_value='115200'),
        # DeclareLaunchArgument('force_frame_id', default_value='base'),
        DeclareLaunchArgument('force_frame_id', default_value='sensor_frame'),
        DeclareLaunchArgument('force_topic_name', default_value='/external_force_torque_wrench'),
        DeclareLaunchArgument('force_auto_zero', default_value='true'),

        DeclareLaunchArgument('tcp_pose_topic', default_value='/tcp_pose_broadcaster/pose'),
        DeclareLaunchArgument('ee_pose_topic', default_value='/UR5/ee_pose'),
        DeclareLaunchArgument('ee_twist_topic', default_value='/UR5/ee_twist'),
        DeclareLaunchArgument('twist_min_dt', default_value='0.0001'),
        DeclareLaunchArgument('twist_max_angular_speed', default_value='10.0'),

        DeclareLaunchArgument('topic_control_wrench', default_value='/arm_admittance_control/control_wrench'),
        DeclareLaunchArgument('topic_arm_command', default_value='/UR5/desired_twist'),
        DeclareLaunchArgument('topic_arm_pose_command', default_value='/arm_desired_pose'),
        DeclareLaunchArgument('topic_desired_pose', default_value='/desired_pose'),
        DeclareLaunchArgument('topic_desired_twist', default_value='/desired_twist'),
        DeclareLaunchArgument('topic_desired_accel', default_value='/desired_accel'),
        DeclareLaunchArgument('enable_topic_monitor', default_value='true'),
        DeclareLaunchArgument('enable_output_smoothing', default_value='true'),
        DeclareLaunchArgument('twist_smoothing_alpha_linear', default_value='0.03'),
        DeclareLaunchArgument('twist_smoothing_alpha_angular', default_value='0.01'),
        DeclareLaunchArgument('pose_smoothing_alpha_linear', default_value='0.03'),
        DeclareLaunchArgument('pose_smoothing_alpha_angular', default_value='0.01'),

        DeclareLaunchArgument('control_wrench_frame_id', default_value='base'),
        DeclareLaunchArgument('control_wrench_publish_rate', default_value='125.0'),
        DeclareLaunchArgument('control_force_x', default_value='0.0'),
        DeclareLaunchArgument('control_force_y', default_value='0.0'),
        DeclareLaunchArgument('control_force_z', default_value='0.0'),
        DeclareLaunchArgument('control_torque_x', default_value='0.0'),
        DeclareLaunchArgument('control_torque_y', default_value='0.0'),
        DeclareLaunchArgument('control_torque_z', default_value='0.0'),

        DeclareLaunchArgument('speedl_acc', default_value='3.0'),
        DeclareLaunchArgument('speedl_time', default_value='0.08'),
        DeclareLaunchArgument('speedl_publish_rate', default_value='125.0'),
        DeclareLaunchArgument('speedl_command_timeout', default_value='0.2'),
        DeclareLaunchArgument('speedl_zero_on_timeout', default_value='true'),
        DeclareLaunchArgument('speedl_send_stop_on_exit', default_value='true'),
        DeclareLaunchArgument('speedl_max_linear_speed', default_value='0.5'),
        DeclareLaunchArgument('speedl_max_angular_speed', default_value='0.6'),
        DeclareLaunchArgument('speedl_skip_repeated_zero_command', default_value='true'),
        DeclareLaunchArgument('speedl_zero_command_epsilon', default_value='1e-6'),
        DeclareLaunchArgument(
            'admittance_params_file',
            default_value='/home/liutiancheng/Lab_WS/zzrobot_ws/src/robot_control_pkg/robot_admittance_control/config/admittance_params_ros2.yaml'
        ),

        TimerAction(period=0.0, actions=[ur_driver_launch]),
        TimerAction(period=1.0, actions=[flange_to_sensor_tf_node]),
        TimerAction(period=2.0, actions=[force_sensor_launch]),
        TimerAction(period=3.0, actions=[tcp_twist_estimator_launch]),
        TimerAction(period=3.5, actions=[desired_trajectory_launch]),
        TimerAction(period=4.0, actions=[control_wrench_publisher_launch]),
        # TimerAction(period=4.5, actions=[speedl_bridge_launch]),
        TimerAction(period=4.5, actions=[rtde_servol_pose_controller_launch]),
        # TimerAction(period=4.5, actions=[rtde_velocity_publisher_launch]),
        TimerAction(period=5.0, actions=[admittance_controller_launch]),

    ])
