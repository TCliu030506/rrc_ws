from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ee_pose_topic = LaunchConfiguration('ee_pose_topic')
    ee_twist_topic = LaunchConfiguration('ee_twist_topic')
    wrench_topic = LaunchConfiguration('wrench_topic')
    desired_pose_topic = LaunchConfiguration('desired_pose_topic')
    desired_twist_topic = LaunchConfiguration('desired_twist_topic')
    desired_accel_topic = LaunchConfiguration('desired_accel_topic')
    admittance_cmd_pose_topic = LaunchConfiguration('admittance_cmd_pose_topic')
    use_servo_bridge = LaunchConfiguration('use_servo_bridge')
    servo_twist_topic = LaunchConfiguration('servo_twist_topic')
    ee_state_source_frame = LaunchConfiguration('ee_state_source_frame')
    ee_state_target_frame = LaunchConfiguration('ee_state_target_frame')
    use_moveit = LaunchConfiguration('use_moveit')
    ur_type = LaunchConfiguration('ur_type')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            FindPackageShare('asm_description'),
            'worlds',
            'myworld.world',
        ])),
        DeclareLaunchArgument('use_moveit', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('moveit_config_package', default_value='ur_moveit_config'),
        DeclareLaunchArgument('moveit_config_file', default_value='ur.srdf.xacro'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        DeclareLaunchArgument('ee_pose_topic', default_value='/gazebo/ee_pose'),
        DeclareLaunchArgument('ee_twist_topic', default_value='/gazebo/ee_twist'),
        DeclareLaunchArgument('wrench_topic', default_value='/wrench_compensated'),
        DeclareLaunchArgument('desired_pose_topic', default_value='/scan/desired_pose'),
        DeclareLaunchArgument('desired_twist_topic', default_value='/scan/desired_twist'),
        DeclareLaunchArgument('desired_accel_topic', default_value='/scan/desired_accel'),
        DeclareLaunchArgument('admittance_cmd_pose_topic', default_value='/admittance/cmd_pose'),
        DeclareLaunchArgument('use_servo_bridge', default_value='false'),
        DeclareLaunchArgument('servo_twist_topic', default_value='/servo_node/delta_twist_cmds'),
        DeclareLaunchArgument('ee_state_source_frame', default_value='base_link'),
        DeclareLaunchArgument('ee_state_target_frame', default_value='tool0'),

        # ========== Phase 1: 启动 Gazebo 和机器人控制 ==========
        # 启动 Gazebo + 重力补偿
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ultra_scanning_sim'),
                'launch',
                'sim_base.launch.py',
            ])),
            condition=UnlessCondition(use_moveit),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                # 添加初始位置文件参数
                'initial_positions_file': PathJoinSubstitution([
                    FindPackageShare('asm_description'),
                    'config',
                    'my_initial_positions.yaml',
                ]),
            }.items(),
        ),

        # UR5控制 (use_moveit=true)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ur_simulation_gazebo'),
                'launch',
                'ur_sim_control.launch.py',
            ])),
            condition=IfCondition(use_moveit),
            launch_arguments={
                'ur_type': ur_type,
                'runtime_config_package': 'asm_description',
                'controllers_file': 'ur5_with_asm_controllers.yaml',
                'description_package': 'asm_description',
                'description_file': 'ur5_with_asm.urdf.xacro',
                'launch_rviz': 'false',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'world': LaunchConfiguration('world'),
            }.items(),
        ),

        # ========== Phase 2: 延迟 1 秒后启动 MoveIt ==========
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg='[LAUNCH] Waiting for joint_states topic...'),
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'echo', '/joint_states', '--once'],
                    output='screen',
                ),
                LogInfo(msg='[LAUNCH] joint_states topic is ready'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'),
                        'launch',
                        'ur_moveit.launch.py',
                    ])),
                    condition=IfCondition(use_moveit),
                    launch_arguments={
                        'ur_type': ur_type,
                        'description_package': 'asm_description',
                        'description_file': 'ur5_with_asm.urdf.xacro',
                        'moveit_config_package': moveit_config_package,
                        'moveit_config_file': moveit_config_file,
                        'use_sim_time': 'true',
                        'launch_rviz': LaunchConfiguration('launch_rviz'),
                        'use_fake_hardware': 'true',
                    }.items(),
                ),
            ],
        ),

        # ========== Phase 3: 延迟 2 秒后启动控制器和辅助节点 ==========
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['asm_arm_controller', '-c', '/controller_manager'],
                    output='screen',
                    condition=IfCondition(use_moveit),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('tool_gravity_compensation'),
                        'launch',
                        'gravity_compensation_sim_dynamic.launch.py',
                    ])),
                    condition=IfCondition(use_moveit),
                ),
                # 启动ur5_joints_controller节点
                Node(
                    package='ur5_sim_servo_control',
                    executable='ur5_joints_controller',
                    name='ur5_joints_controller',
                    output='screen',
                ),
            ],
        ),

        # ========== Phase 4: 延迟 3 秒后启动 TF 状态节点 和 ur5_ik服务节点 ==========
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[LAUNCH] Waiting for /compute_ik service...'),
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/compute_ik', 'moveit_msgs/srv/GetPositionIK',
                         '{ik_request: {group_name: "ur_manipulator", ik_link_name: "tool0", '
                         'pose_stamped: {header: {frame_id: "base_link"}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, '
                         'orientation: {w: 1.0}}}, timeout: {sec: 1}}}'],
                    output='screen',
                ),
                LogInfo(msg='[LAUNCH] /compute_ik service is ready'),
                Node(
                    package='ultra_scanning_sim',
                    executable='ee_state_from_tf_node',
                    name='ee_state_from_tf_node',
                    output='screen',
                    parameters=[{
                        'source_frame': ee_state_source_frame,
                        'target_frame': ee_state_target_frame,
                        'output_pose_topic': ee_pose_topic,
                        'output_twist_topic': ee_twist_topic,
                        'publish_rate': 125.0,
                    }],
                ),
                # 启动ur5_ik服务节点
                Node(
                    package='ur5_sim_servo_control',
                    executable='ur5_ik_service',
                    name='ur5_ik_service',
                    output='screen',
                ),
            ],
        ),

        # ========== Phase 5: 延迟 4 秒后启动轨迹规划节点 ==========
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('robot_trajectory_planner'),
                        'launch',
                        'scan_raster_trajectory.launch.py',
                    ])),
                    launch_arguments={
                        'topic_desired_pose': desired_pose_topic,
                        'topic_desired_twist': desired_twist_topic,
                        'topic_desired_accel': desired_accel_topic,
                    }.items(),
                ),
            ],
        ),

        # ========== Phase 6: 延迟 5 秒后启动导纳控制器 ==========
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[LAUNCH] Waiting for desired trajectory topics...'),
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'echo', desired_pose_topic, '--once'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'echo', desired_twist_topic, '--once'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'echo', desired_accel_topic, '--once'],
                    output='screen',
                ),
                LogInfo(msg='[LAUNCH] Desired trajectory topics are ready'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('robot_admittance_control'),
                        'launch',
                        'arm_admittance_controller_sim.launch.py',
                    ])),
                    launch_arguments={
                        'pose_topic_arm': ee_pose_topic,
                        'twist_topic_arm': ee_twist_topic,
                        'wrench_ext_topic_arm': wrench_topic,
                        'desired_pose_topic': desired_pose_topic,
                        'desired_twist_topic': desired_twist_topic,
                        'desired_accel_topic': desired_accel_topic,
                    }.items(),
                ),
            ],
        ),
    ])