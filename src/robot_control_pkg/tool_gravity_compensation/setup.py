from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tool_gravity_compensation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ltc',
    maintainer_email='3210101491@zju.edu.cn',
    description='Gravity calibration and compensation for wrist F/T sensor mounted tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gravity_calibration_node = tool_gravity_compensation.gravity_calibration_node:main',
            'gravity_compensation_node = tool_gravity_compensation.gravity_compensation_node:main',
            'auto_gravity_calibration_node = tool_gravity_compensation.auto_gravity_calibration_node:main',
            'sim_auto_gravity_calibration_node = tool_gravity_compensation.sim_auto_gravity_calibration_node:main',
            'sim_dynamic_gravity_calibration_node = tool_gravity_compensation.sim_dynamic_gravity_calibration_node:main',
            'sim_dynamic_gravity_compensation_node = tool_gravity_compensation.sim_dynamic_gravity_compensation_node:main',
            'mujoco_dynamic_gravity_calibration_node = tool_gravity_compensation.mujoco_dynamic_gravity_calibration_node:main',
            'mujoco_dynamic_gravity_compensation_node = tool_gravity_compensation.mujoco_dynamic_gravity_compensation_node:main',
        ],
    },
)
