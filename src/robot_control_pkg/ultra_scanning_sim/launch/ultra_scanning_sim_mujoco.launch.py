from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== 1. MuJoCo 仿真节点参数 ====================
    keyframe_name = LaunchConfiguration('keyframe_name')
    use_viewer = LaunchConfiguration('use_viewer')
    publish_rgb = LaunchConfiguration('publish_rgb')
    publish_depth = LaunchConfiguration('publish_depth')
    camera_names = LaunchConfiguration('camera_names')
    use_camera_visualizer = LaunchConfiguration('use_camera_visualizer')
    publish_end_effector_pose = LaunchConfiguration('publish_end_effector_pose')
    end_effector_pose_topic = LaunchConfiguration('end_effector_pose_topic')
    cartesian_command_topic = LaunchConfiguration('cartesian_command_topic')

    # ==================== 2. 机械臂状态反馈参数 ====================
    ee_pose_topic = LaunchConfiguration('ee_pose_topic')
    ee_twist_topic = LaunchConfiguration('ee_twist_topic')
    wrench_topic = LaunchConfiguration('wrench_topic')
    compensated_wrench_topic = LaunchConfiguration('compensated_wrench_topic')

    # ==================== 3. 重力补偿参数 ====================
    gravity_comp_params_file = LaunchConfiguration('gravity_comp_params_file')

    # ==================== 4. 导纳控制参数 ====================
    desired_pose_topic = LaunchConfiguration('desired_pose_topic')
    admittance_cmd_pose_topic = LaunchConfiguration('admittance_cmd_pose_topic')
    enable_admittance = LaunchConfiguration('enable_admittance')
    desired_twist_topic = LaunchConfiguration('desired_twist_topic')
    desired_accel_topic = LaunchConfiguration('desired_accel_topic')

    # 扫查轨迹输出话题与导纳控制目标话题保持一致
    topic_desired_pose = desired_pose_topic
    topic_desired_twist = desired_twist_topic
    topic_desired_accel = desired_accel_topic

    # ==================== 5. 扫描轨迹规划参数 ====================
    # 5.1 轨迹发布公共参数
    publish_rate = LaunchConfiguration('publish_rate')

    # 5.2 起始过渡参数：机械臂当前位姿 -> 扫查起点
    current_pose_topic = LaunchConfiguration('current_pose_topic')
    initial_blend_duration = LaunchConfiguration('initial_blend_duration')

    # 5.3 圆柱扫查几何参数
    scan_center_x = LaunchConfiguration('scan_center_x')
    scan_center_z = LaunchConfiguration('scan_center_z')
    scan_radius = LaunchConfiguration('scan_radius')
    scan_y_start = LaunchConfiguration('scan_y_start')
    scan_y_end = LaunchConfiguration('scan_y_end')
    scan_y_step = LaunchConfiguration('scan_y_step')

    # 5.4 圆弧采样与姿态参数
    scan_arc_points = LaunchConfiguration('scan_arc_points')
    scan_arc_start_deg = LaunchConfiguration('scan_arc_start_deg')
    scan_arc_end_deg = LaunchConfiguration('scan_arc_end_deg')
    scan_orientation_mode = LaunchConfiguration('scan_orientation_mode')
    scan_orientation_xyzw = LaunchConfiguration('scan_orientation_xyzw')
    scan_contact_offset = LaunchConfiguration('scan_contact_offset')

    # 5.5 层间过渡参数：y 切换时平滑过渡到下一层
    y_transition_duration = LaunchConfiguration('y_transition_duration')

    return LaunchDescription([
        # ========== 1. MujoCo 仿真节点参数 ==========
        DeclareLaunchArgument('keyframe_name', default_value='home'),
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('publish_rgb', default_value='true'),
        DeclareLaunchArgument('publish_depth', default_value='false'),
        DeclareLaunchArgument('camera_names', default_value="camera_fixed,camera_targetbody"),
        DeclareLaunchArgument('show_depth', default_value='false'),
        DeclareLaunchArgument('use_camera_visualizer', default_value='false'),
        DeclareLaunchArgument('publish_end_effector_pose', default_value='true'),
        DeclareLaunchArgument('end_effector_pose_topic', default_value='/end_effector_pose'),
        DeclareLaunchArgument('cartesian_command_topic', default_value='/cartesian_target_pose'),

        # ========== 2. 机械臂状态反馈参数 ==========
        DeclareLaunchArgument('ee_pose_topic', default_value='/mujoco/ee_pose'),
        DeclareLaunchArgument('ee_twist_topic', default_value='/mujoco/ee_twist'),
        DeclareLaunchArgument('wrench_topic', default_value='/sensors/wrench'),
        DeclareLaunchArgument('compensated_wrench_topic', default_value='/wrench_compensated'),

        # ========== 3. 重力补偿参数 ==========
        DeclareLaunchArgument(
            'gravity_comp_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('tool_gravity_compensation'),
                'config',
                'gravity_compensation_mujoco_dynamic_params.yaml',
            ]),
        ),

        # ========== 4. 导纳控制参数 ==========
        DeclareLaunchArgument('desired_pose_topic', default_value='/scan/desired_pose'),
        DeclareLaunchArgument('admittance_cmd_pose_topic', default_value='/admittance/cmd_pose'),
        DeclareLaunchArgument('enable_admittance', default_value='true'),
        DeclareLaunchArgument('desired_twist_topic', default_value='/scan/desired_twist'),
        DeclareLaunchArgument('desired_accel_topic', default_value='/scan/desired_accel'),

        # ========== 5. 扫描轨迹规划参数（圆柱扫查） ==========
        # 5.1 轨迹发布公共参数
        DeclareLaunchArgument('publish_rate', default_value='20.0'),

        # 5.2 起始过渡参数：机械臂当前位姿 -> 扫查起点
        # initial_blend_duration 控制从当前末端位姿平滑过渡到扫查起点的时长
        DeclareLaunchArgument('current_pose_topic', default_value='/end_effector_pose'),
        DeclareLaunchArgument('initial_blend_duration', default_value='40.0'),

        # 5.3 圆柱扫查几何参数
        DeclareLaunchArgument('scan_center_x', default_value='0.80'),
        DeclareLaunchArgument('scan_center_z', default_value='0.55'),
        DeclareLaunchArgument('scan_radius', default_value='0.30'),
        DeclareLaunchArgument('scan_y_start', default_value='0.0'),
        DeclareLaunchArgument('scan_y_end', default_value='0.20'),
        DeclareLaunchArgument('scan_y_step', default_value='0.05'),

        # 5.4 圆弧采样与姿态参数
        DeclareLaunchArgument('scan_arc_points', default_value='1000'),
        DeclareLaunchArgument('scan_arc_start_deg', default_value='170.0'),
        DeclareLaunchArgument('scan_arc_end_deg', default_value='190.0'),
        DeclareLaunchArgument('scan_orientation_mode', default_value='normal'),
        DeclareLaunchArgument('scan_orientation_xyzw', default_value='[0.0, 0.7071, 0.0, 0.7071]'),
        DeclareLaunchArgument('scan_contact_offset', default_value='0.0'),

        # 5.5 层间过渡参数：y 切换时平滑过渡到下一层
        # 该参数控制每次完成一层圆弧后，从当前 y 平滑移动到下一层 y 的过渡时长
        DeclareLaunchArgument('y_transition_duration', default_value='4.0'),

        # ========== 节点启动配置 ==========
        # 1. MujoCo 仿真节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('asm_description_mujoco'),
                    'launch',
                    'asm_ros2_node.launch.py',
                ])
            ),
            launch_arguments={
                'keyframe_name': keyframe_name,
                'use_viewer': use_viewer,
                'publish_rgb': publish_rgb,
                'publish_depth': publish_depth,
                'camera_names': camera_names,
                'publish_end_effector_pose': publish_end_effector_pose,
                'end_effector_pose_topic': end_effector_pose_topic,
                'cartesian_command_topic': cartesian_command_topic,
                'external_wrench_topic': wrench_topic,
                'use_camera_visualizer': use_camera_visualizer,
                'show_depth': LaunchConfiguration('show_depth'),
            }.items(),
        ),

        # 2. 重力补偿节点（将 /sensors/wrench 补偿为 /wrench_compensated）
        TimerAction(
            period=1.2,
            condition=IfCondition(enable_admittance),
            actions=[
                Node(
                    package='tool_gravity_compensation',
                    executable='mujoco_dynamic_gravity_compensation_node',
                    name='mujoco_dynamic_gravity_compensation_node',
                    output='screen',
                    parameters=[
                        gravity_comp_params_file,
                        {
                            'wrench_in_topic': wrench_topic,
                            'wrench_out_topic': compensated_wrench_topic,
                        },
                    ],
                ),
            ],
        ),
        
        # 3. 圆柱面扫描轨迹规划器
        TimerAction(
            period=2.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('robot_trajectory_planner'),
                            'launch',
                            'scan_cylindrical_surface_mujoco.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'topic_desired_pose': topic_desired_pose,
                        'topic_desired_twist': topic_desired_twist,
                        'topic_desired_accel': topic_desired_accel,
                        'current_pose_topic': current_pose_topic,
                        'initial_blend_duration': initial_blend_duration,
                        'publish_rate': publish_rate,
                        'center_x': scan_center_x,
                        'center_z': scan_center_z,
                        'radius': scan_radius,
                        'y_start': scan_y_start,
                        'y_end': scan_y_end,
                        'y_step': scan_y_step,
                        'arc_points': scan_arc_points,
                        'arc_start_deg': scan_arc_start_deg,
                        'arc_end_deg': scan_arc_end_deg,
                        'orientation_mode': scan_orientation_mode,
                        'orientation_xyzw': scan_orientation_xyzw,
                        'contact_offset': scan_contact_offset,
                        'y_transition_duration': y_transition_duration,
                    }.items(),
                ),
            ],
        ),

        # # ========== 已弃用 ==========
        # # 原栅栏式扫描轨迹规划器（已被 scan_cylindrical_surface 替代）
        # TimerAction(
        #     period=1.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 PathJoinSubstitution([
        #                     FindPackageShare('robot_trajectory_planner'),
        #                     'launch',
        #                     'scan_raster_trajectory.launch.py',
        #                 ])
        #             ),
        #             launch_arguments={
        #                 'topic_desired_pose': desired_pose_topic,
        #                 'topic_desired_twist': desired_twist_topic,
        #                 'topic_desired_accel': desired_accel_topic,
        #                 'publish_rate': publish_rate,
        #                 'scan_center': scan_center,
        #                 'scan_length_x': scan_length_x,
        #                 'scan_width_y': scan_width_y,
        #                 'scan_speed': scan_speed,
        #                 'line_spacing': line_spacing,
        #                 'line_turnaround_pause': line_turnaround_pause,
        #                 'orientation_xyzw': orientation_xyzw,
        #             }.items(),
        #         ),
        #     ],
        # ),

        # 4. PoseStamped 转换桥接节点（将 MuJoCo PoseStamped 转换为导纳控制所需的 Pose/Twist）
        TimerAction(
            period=1.5,
            condition=IfCondition(enable_admittance),
            actions=[
                Node(
                    package='ultra_scanning_sim',
                    executable='pose_stamped_to_pose_bridge',
                    name='mujoco_pose_stamped_to_pose_bridge',
                    output='screen',
                    parameters=[{
                        'input_pose_stamped_topic': end_effector_pose_topic,
                        'output_pose_topic': ee_pose_topic,
                        'output_twist_topic': ee_twist_topic,
                    }],
                ),
            ],
        ),

        # 5. 导纳控制器（阻抗控制）
        TimerAction(
            period=2.0,
            condition=IfCondition(enable_admittance),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('robot_admittance_control'),
                            'launch',
                            'arm_admittance_controller_sim.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'pose_topic_arm': ee_pose_topic,
                        'twist_topic_arm': ee_twist_topic,
                        'wrench_ext_topic_arm': compensated_wrench_topic,
                        'pose_cmd_topic_arm': admittance_cmd_pose_topic,
                        'desired_pose_topic': desired_pose_topic,
                        'desired_twist_topic': desired_twist_topic,
                        'desired_accel_topic': desired_accel_topic,
                    }.items(),
                ),
            ],
        ),

        # 6. Pose 转换桥接节点（将导纳输出 Pose 转为 MuJoCo 控制输入 PoseStamped）
        TimerAction(
            period=2.2,
            condition=IfCondition(enable_admittance),
            actions=[
                Node(
                    package='ultra_scanning_sim',
                    executable='pose_to_pose_stamped_bridge',
                    name='admittance_pose_bridge',
                    output='screen',
                    parameters=[{
                        'input_pose_topic': admittance_cmd_pose_topic,
                        'output_pose_stamped_topic': cartesian_command_topic,
                        'frame_id': 'world',
                    }],
                ),
            ],
        ),

        # # 7. 控制力发布节点
        # TimerAction(
        #     period=2.5,
        #     condition=IfCondition(enable_admittance),
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 PathJoinSubstitution([
        #                     FindPackageShare('robot_admittance_control'),
        #                     'launch',
        #                     'adaptive_control_wrench_publisher.launch.py',
        #                 ])
        #             ),
        #             launch_arguments={
        #                 'topic_control_wrench': '/arm_admittance_control/control_wrench',
        #                 'topic_compensated_wrench': compensated_wrench_topic,
        #                 'frame_id': 'asm_ee_site',
        #                 'publish_rate': '100.0',
        #                 'tf_lookup_timeout_sec': '0.05',
        #                 'normal_axis': 'z',
        #                 'normal_axis_sign': '-1.0',
        #                 'contact_force_threshold': '2.0',
        #                 'release_force_threshold': '2.0',
        #                 'force_target': '10.0',
        #                 'force_rate_limit': '20.0',
        #                 'force_tracking_epsilon': '0.5',
        #             }.items(),
        #         ),
        #     ],
        # ),


    ])