from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rviz = LaunchConfiguration('launch_rviz')

    cmd_pose_topic = LaunchConfiguration('cmd_pose_topic')
    wrench_topic = LaunchConfiguration('wrench_topic')
    wrench_comp_topic = LaunchConfiguration('wrench_comp_topic')
    wrench_base_topic = LaunchConfiguration('wrench_base_topic')
    wrench_comp_base_topic = LaunchConfiguration('wrench_comp_base_topic')

    source_frame = LaunchConfiguration('source_frame')
    target_frame = LaunchConfiguration('target_frame')

    position_x = LaunchConfiguration('position_x')
    position_y = LaunchConfiguration('position_y')
    position_z = LaunchConfiguration('position_z')

    hold_seconds_per_pose = LaunchConfiguration('hold_seconds_per_pose')
    start_delay_sec = LaunchConfiguration('start_delay_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('asm_description'),
                'worlds',
                'myworld.world',
            ]),
        ),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        DeclareLaunchArgument('cmd_pose_topic', default_value='/admittance/cmd_pose'),
        DeclareLaunchArgument('wrench_topic', default_value='/wrench'),
        DeclareLaunchArgument('wrench_comp_topic', default_value='/wrench_compensated'),
        DeclareLaunchArgument('wrench_base_topic', default_value='/wrench_base_link'),
        DeclareLaunchArgument('wrench_comp_base_topic', default_value='/wrench_compensated_base_link'),

        DeclareLaunchArgument('source_frame', default_value='asm_force_sensor_link'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),

        DeclareLaunchArgument('position_x', default_value='0.15'),
        DeclareLaunchArgument('position_y', default_value='0.00'),
        DeclareLaunchArgument('position_z', default_value='0.65'),
        DeclareLaunchArgument('hold_seconds_per_pose', default_value='20.0'),
        DeclareLaunchArgument('start_delay_sec', default_value='8.0'),

        # 1) Start Gazebo simulation base and gravity compensation.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ultra_scanning_sim'),
                    'launch',
                    'sim_base.launch.py',
                ])
            ),
            launch_arguments={
                'world': world,
                'gazebo_gui': gazebo_gui,
                'launch_rviz': launch_rviz,
            }.items(),
        ),

        # Required for ur5_ik.py (/compute_ik service).
        TimerAction(
            period=1.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('ur_moveit_config'),
                            'launch',
                            'ur_moveit.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'ur_type': 'ur5',
                        'description_package': 'asm_description',
                        'description_file': 'ur5_with_asm.urdf.xacro',
                        'moveit_config_package': 'ur_moveit_config',
                        'moveit_config_file': 'ur.srdf.xacro',
                        'use_sim_time': 'true',
                        'launch_rviz': launch_rviz,
                        'use_fake_hardware': 'true',
                    }.items(),
                ),
            ],
        ),

        # 2) Start IK and joint trajectory bridge for robot motion.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ur5_sim_servo_control',
                    executable='ur5_joints_controller',
                    name='ur5_joints_controller',
                    output='screen',
                ),
                Node(
                    package='ur5_sim_servo_control',
                    executable='ur5_ik_service',
                    name='ur5_ik_service',
                    output='screen',
                    parameters=[{
                        'input_pose_topic': cmd_pose_topic,
                        'output_joints_topic': '/ur5_target_joints',
                        'ik_service': '/compute_ik',
                        'group_name': 'ur_manipulator',
                        'ik_link_name': 'tool0',
                        'planning_frame': 'base_link',
                    }],
                ),
            ],
        ),

        # 3) Wrench frame transform node.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ultra_scanning_sim',
                    executable='wrench_frame_transform_node',
                    name='wrench_frame_transform_node',
                    output='screen',
                    parameters=[{
                        'input_wrench_topic': wrench_topic,
                        'input_wrench_comp_topic': wrench_comp_topic,
                        'output_wrench_topic': wrench_base_topic,
                        'output_wrench_comp_topic': wrench_comp_base_topic,
                        'source_frame': source_frame,
                        'target_frame': target_frame,
                    }],
                ),
            ],
        ),

        # 4) Standard pose test node.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ultra_scanning_sim',
                    executable='force_sensor_pose_test_node',
                    name='force_sensor_pose_test_node',
                    output='screen',
                    parameters=[{
                        'cmd_pose_topic': cmd_pose_topic,
                        'wrench_base_topic': wrench_base_topic,
                        'wrench_comp_base_topic': wrench_comp_base_topic,
                        'position_x': position_x,
                        'position_y': position_y,
                        'position_z': position_z,
                        'hold_seconds_per_pose': hold_seconds_per_pose,
                        'start_delay_sec': start_delay_sec,
                    }],
                ),
            ],
        ),
    ])
