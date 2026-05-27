import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'brt_rs485_encoder_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'README_BRT_RS485.md',
            'README_ROS2_TOPICS.md',
        ]),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'pyserial>=3.5'],
    zip_safe=True,
    maintainer='liutiancheng',
    maintainer_email='14011673+liu-tiancheng0506@user.noreply.gitee.com',
    description='ROS 2 driver and CLI for BRT RS485 Modbus encoders.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'brt_rs485_encoder = brt_rs485_encoder_pkg.brt_rs485_encoder:main',
            'brt_rs485_encoder_node = brt_rs485_encoder_pkg.brt_rs485_encoder_node:main',
            'reset_zero_client_node = brt_rs485_encoder_pkg.reset_zero_client:main',
        ],
    },
)
