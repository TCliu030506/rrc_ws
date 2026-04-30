from glob import glob
from setuptools import find_packages, setup


package_name = 'ur5_sim_servo_control'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liutiancheng',
    maintainer_email='14011673+liu-tiancheng0506@user.noreply.gitee.com',
    description='UR5 simulation pose-to-Servo bridge package.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ur5_pose_target_servo_bridge = ur5_sim_servo_control.pose_target_to_servo_twist:main',
            'ur5_joints_controller = ur5_sim_servo_control.ur5_joints_controller:main',
            'ur5_ik_service = ur5_sim_servo_control.ur5_ik:main',
        ],
    },
)
