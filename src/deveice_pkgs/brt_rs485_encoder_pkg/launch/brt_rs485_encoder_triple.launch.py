import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_encoder_params(config_name: str) -> dict:
    config_path = os.path.join(
        get_package_share_directory('brt_rs485_encoder_pkg'),
        'config',
        config_name,
    )
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    return config['brt_rs485_encoder_node']['ros__parameters']


def generate_launch_description() -> LaunchDescription:
    encoder1_params = _load_encoder_params('brt_rs485_encoder1.yaml')
    encoder2_params = _load_encoder_params('brt_rs485_encoder2.yaml')
    encoder3_params = _load_encoder_params('brt_rs485_encoder3.yaml')
    
    return LaunchDescription([
        Node(
            package='brt_rs485_encoder_pkg',
            executable='brt_rs485_encoder_node',
            name='brt_rs485_encoder1_node',
            output='screen',
            parameters=[encoder1_params],
        ),
        Node(
            package='brt_rs485_encoder_pkg',
            executable='brt_rs485_encoder_node',
            name='brt_rs485_encoder2_node',
            output='screen',
            parameters=[encoder2_params],
        ),
        Node(
            package='brt_rs485_encoder_pkg',
            executable='brt_rs485_encoder_node',
            name='brt_rs485_encoder3_node',
            output='screen',
            parameters=[encoder3_params],
        )
    ])
