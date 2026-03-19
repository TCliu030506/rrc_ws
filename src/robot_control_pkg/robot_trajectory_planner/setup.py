from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_trajectory_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ltc',
    maintainer_email='3210101491@zju.edu.cn',
    description='Simple two-point cyclic linear trajectory planner node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'two_point_trajectory_node = robot_trajectory_planner.two_point_trajectory_node:main',
            'teleoperation_trajectory_node = robot_trajectory_planner.teleoperation_trajectory_node:main',
        ],
    },
)
