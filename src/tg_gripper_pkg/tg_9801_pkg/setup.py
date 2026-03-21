"""tg_9801_pkg 的 setuptools 安装配置。

用于声明包元数据、安装文件和可执行入口，使该功能包可以被 colcon 正常构建和安装。
"""

from setuptools import setup

package_name = 'tg_9801_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tg_9801.launch.py', 'launch/tg_9801_quick_test.launch.py']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='liutiancheng',
    maintainer_email='liutiancheng@example.com',
    description='TG-9801 gripper control package via serial protocol.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tg_9801_node = tg_9801_pkg.tg_9801_node:main',
            'tg_9801_cli = tg_9801_pkg.tg_9801_cli:main',
            'tg_9801_quick_test = tg_9801_pkg.tg_9801_quick_test:main',
            'tg_9801_test_client = tg_9801_pkg.tg_9801_test_client:main',
            'tg_state_broadcaster = tg_9801_pkg.tg_state_broadcaster:main',
            'tg_9801_ros_demo = tg_9801_pkg.tg_9801_ros_demo:main',
            'tg_9801_teleoperation_node = tg_9801_pkg.tg_9801_teleoperation_node:main',
        ],
    },
)
