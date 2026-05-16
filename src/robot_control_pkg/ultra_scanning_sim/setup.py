from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ultra_scanning_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liutiancheng',
    maintainer_email='14011673+liu-tiancheng0506@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ee_state_from_tf_node = ultra_scanning_sim.ee_state_from_tf_node:main',
            'wrench_frame_transform_node = ultra_scanning_sim.wrench_frame_transform_node:main',
            'force_sensor_pose_test_node = ultra_scanning_sim.force_sensor_pose_test_node:main',
            'pose_to_pose_stamped_bridge = ultra_scanning_sim.pose_to_pose_stamped_bridge:main',
            'pose_stamped_to_pose_bridge = ultra_scanning_sim.pose_stamped_to_pose_bridge:main',
        ],
    },
)
