from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            FindPackageShare('asm_description'),
            'worlds',
            'myworld.world',
        ])),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('asm_description'),
                'launch',
                'ur5_with_asm_gazebo.launch.py',
            ])),
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('tool_gravity_compensation'),
                'launch',
                'gravity_compensation_sim_dynamic.launch.py',
            ])),
        ),
    ])
