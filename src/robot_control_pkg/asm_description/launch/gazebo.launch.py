import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_package_dir = get_package_share_directory('asm_description')
    robot_package_parent_dir = os.path.dirname(robot_package_dir)
    xacro_file_path = os.path.join(robot_package_dir, 'urdf', 'asm_description.xacro')
    gazebo_control_fragment_path = os.path.join(robot_package_dir, 'urdf', 'asm_description_gazebo.xacro')
    ros2_controllers_file_path = os.path.join(robot_package_dir, 'config', 'ros2_controllers.yaml')
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    entity_name = LaunchConfiguration('entity_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = robot_description_config.toxml()
    robot_description = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', robot_description, count=1)
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.DOTALL)
    with open(gazebo_control_fragment_path, 'r', encoding='utf-8') as gazebo_control_fragment_file:
        gazebo_control_fragment = gazebo_control_fragment_file.read()
    gazebo_control_fragment = gazebo_control_fragment.replace('__ROS2_CONTROLLERS_FILE__', ros2_controllers_file_path)
    robot_description = robot_description.replace('</robot>', f'{gazebo_control_fragment}\n</robot>', 1)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation clock',
    )

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='false',
        description='Whether to launch joint_state_publisher_gui',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz',
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_ros_package_dir, 'worlds', 'empty.world'),
        description='Full path to world file to load in Gazebo',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(robot_package_dir, 'rviz', 'asm_description.rviz'),
        description='Full path to the RViz config file to use',
    )

    declare_entity_name_cmd = DeclareLaunchArgument(
        'entity_name',
        default_value='asm_description',
        description='Entity name used when spawning robot in Gazebo',
    )

    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Robot spawn x position in Gazebo world frame',
    )

    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn y position in Gazebo world frame',
    )

    declare_spawn_z_cmd = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.0',
        description='Robot spawn z position in Gazebo world frame',
    )

    declare_spawn_yaw_cmd = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Robot spawn yaw in Gazebo world frame',
    )

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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }
        ],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity',
            entity_name,
            '-topic',
            'robot_description',
            '-x',
            spawn_x,
            '-y',
            spawn_y,
            '-z',
            spawn_z,
            '-Y',
            spawn_yaw,
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            ros2_controllers_file_path,
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            ros2_controllers_file_path,
        ],
        output='screen',
    )

    spawn_controllers_after_entity = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
            ],
        )
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
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
        gazebo_model_path_env,
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        spawn_controllers_after_entity,
        rviz_node,
    ])