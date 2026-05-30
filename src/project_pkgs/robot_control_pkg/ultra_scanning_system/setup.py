import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ultra_scanning_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liutiancheng',
    maintainer_email='14011673+liu-tiancheng0506@user.noreply.gitee.com',
    description='Launch files for the real ultrasound scanning system.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'asm_tool_tf_broadcaster = ultra_scanning_system.asm_tool_tf_broadcaster:main',
            'asm_ee_command_transform = ultra_scanning_system.asm_ee_command_transform:main',
            (
                'contact_scan_trajectory_node = '
                'ultra_scanning_system.contact_scan_trajectory_node:main'
            ),
        ],
    },
)
