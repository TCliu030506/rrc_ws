from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # 主窗口UI节点
    main_window_ui_node = Node(
        package='ultra_scanning_ui',
        executable='main_window_ui',
        name='main_window_ui',
        output='screen',
    )
    # UI命令管理节点
    ui_command_manager_node = Node(
        package='ultra_scanning_ui',
        executable='ui_command_manager_node',
        name='ui_command_manager_node',
        output='screen',
    )

    return LaunchDescription([
        main_window_ui_node,
        ui_command_manager_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=main_window_ui_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='main_window_ui exited',
                        )
                    )
                ],
            )
        ),
    ])
