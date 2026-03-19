from setuptools import find_packages, setup
from glob import glob

package_name = 'ur5_rtde_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            'ur5_rtde_control = ur5_rtde_control.ur5_rtde_control:main',
            'teleoperation_control = ur5_rtde_control.teleoperation_control:main',
            'teleoperation_control_ui = ur5_rtde_control.teleoperation_control_ui:main',
            'my_demo = ur5_rtde_control.my_demo:main',
            'omni_pub_test_node = ur5_rtde_control.omni_pub_test_node:main',
            'rtde_velocity_publisher_node = ur5_rtde_control.rtde_velocity_publisher_node:main',
            'rtde_servol_pose_controller_node = ur5_rtde_control.rtde_servol_pose_controller_node:main',
        ],
    },
)
