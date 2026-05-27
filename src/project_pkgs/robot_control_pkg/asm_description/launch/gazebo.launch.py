#!/usr/bin/env python3
"""
Gazebo 仿真环境启动文件

功能：启动 Gazebo 模拟器，加载 ASM（超声扫描模块）机器人模型，
配置控制器，并可选启动 RViz 可视化。

特点：自包含模式，直接处理 XACRO 文件，不依赖外部启动文件。
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """生成启动描述"""
    # ========== 获取包路径 ==========
    robot_package_dir = get_package_share_directory('asm_description')
    robot_package_parent_dir = os.path.dirname(robot_package_dir)
    xacro_file_path = os.path.join(robot_package_dir, 'urdf', 'asm_description.xacro')
    gazebo_control_fragment_path = os.path.join(robot_package_dir, 'urdf', 'asm_description_gazebo.xacro')
    ros2_controllers_file_path = os.path.join(robot_package_dir, 'config', 'ros2_controllers.yaml')
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    
    # ========== 定义启动参数 ==========
    use_sim_time = LaunchConfiguration('use_sim_time')          # 是否使用仿真时间
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')  # 是否启动关节状态发布GUI
    use_rviz = LaunchConfiguration('use_rviz')                  # 是否启动RViz
    world = LaunchConfiguration('world')                        # Gazebo世界文件路径
    entity_name = LaunchConfiguration('entity_name')            # 机器人实体名称
    spawn_x = LaunchConfiguration('spawn_x')                    # 生成位置X
    spawn_y = LaunchConfiguration('spawn_y')                    # 生成位置Y
    spawn_z = LaunchConfiguration('spawn_z')                    # 生成位置Z
    spawn_yaw = LaunchConfiguration('spawn_yaw')                # 生成偏航角
    rviz_config_file = LaunchConfiguration('rviz_config_file')  # RViz配置文件

    # ========== 处理 XACRO 文件 ==========
    # 1. 处理主 URDF 文件
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = robot_description_config.toxml()
    
    # 2. 移除 XML 声明（避免解析错误）
    robot_description = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', robot_description, count=1)
    
    # 3. 移除注释（减少文件大小）
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.DOTALL)
    
    # 4. 合并 Gazebo 控制片段
    with open(gazebo_control_fragment_path, 'r', encoding='utf-8') as gazebo_control_fragment_file:
        gazebo_control_fragment = gazebo_control_fragment_file.read()
    
    # 替换控制器文件路径占位符
    gazebo_control_fragment = gazebo_control_fragment.replace('__ROS2_CONTROLLERS_FILE__', ros2_controllers_file_path)
    
    # 将控制片段插入到 URDF 的 </robot> 标签前
    robot_description = robot_description.replace('</robot>', f'{gazebo_control_fragment}\n</robot>', 1)

    # ========== 声明启动参数 ==========
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间',
    )

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='false',
        description='是否启动关节状态发布GUI',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz可视化',
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_ros_package_dir, 'worlds', 'empty.world'),
        description='Gazebo世界文件完整路径',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(robot_package_dir, 'rviz', 'asm_description.rviz'),
        description='RViz配置文件完整路径',
    )

    declare_entity_name_cmd = DeclareLaunchArgument(
        'entity_name',
        default_value='asm_description',
        description='在Gazebo中生成机器人时使用的实体名称',
    )

    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='机器人在Gazebo世界坐标系中的生成位置X',
    )

    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='机器人在Gazebo世界坐标系中的生成位置Y',
    )

    declare_spawn_z_cmd = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.0',
        description='机器人在Gazebo世界坐标系中的生成位置Z',
    )

    declare_spawn_yaw_cmd = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='机器人在Gazebo世界坐标系中的生成偏航角',
    )

    # ========== 设置环境变量 ==========
    # 设置 Gazebo 模型路径，让 Gazebo 能够找到自定义模型
    gazebo_model_path_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [
            robot_package_dir,
            ':',
            robot_package_parent_dir,
            ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ],
    )

    # ========== 启动 Gazebo ==========
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # ========== 启动 robot_state_publisher ==========
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,  # 使用处理后的 URDF
            }
        ],
    )

    # ========== 生成机器人模型 ==========
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', entity_name,      # 实体名称
            '-topic', 'robot_description',  # 从该话题获取机器人描述
            '-x', spawn_x,              # X位置
            '-y', spawn_y,              # Y位置
            '-z', spawn_z,              # Z位置
            '-Y', spawn_yaw,            # 偏航角
        ],
        output='screen',
    )

    # 延迟生成模型（等待 Gazebo 服务初始化）
    spawn_entity_delay = TimerAction(period=1.0, actions=[spawn_entity_node])

    # ========== 启动控制器 ==========
    # 关节状态广播器：发布关节状态到 /joint_states 话题
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_controllers_file_path,
        ],
        output='screen',
    )

    # 手臂控制器：控制机器人关节运动
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_controllers_file_path,
        ],
        output='screen',
    )

    # 事件处理：模型生成完成后再启动控制器
    spawn_controllers_after_entity = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
            ],
        )
    )

    # ========== 可选节点 ==========
    # 关节状态发布GUI（条件启动）
    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz可视化（条件启动）
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ========== 返回启动描述 ==========
    return LaunchDescription([
        # 参数声明
        declare_use_sim_time_cmd,
        declare_use_joint_state_pub_cmd,
        declare_use_rviz_cmd,
        declare_world_cmd,
        declare_rviz_config_file_cmd,
        declare_entity_name_cmd,
        declare_spawn_x_cmd,
        declare_spawn_y_cmd,
        declare_spawn_z_cmd,
        declare_spawn_yaw_cmd,
        
        # 环境变量
        gazebo_model_path_env,
        
        # Gazebo启动
        gazebo_launch,
        
        # 机器人描述和生成
        robot_state_publisher_node, 
        joint_state_publisher_node,
        spawn_entity_delay,  # 延迟生成模型
        spawn_controllers_after_entity,
        
        # 可视化
        rviz_node,
    ])