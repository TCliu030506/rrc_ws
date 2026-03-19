from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'x6_control_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'),  glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyc',
    maintainer_email='1085954163@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x6_server = x6_control_python.x6_server:main',
            'move_demo = x6_control_python.move_demo:main',
            'aim_to_yaml = x6_control_python.aim_to_yaml:main',
            'x6_record_demo = x6_control_python.x6_record_demo:main',
            'x6_freq_record_demo = x6_control_python.x6_freq_record_demo:main',
            'interpo_test = x6_control_python.interpolation_test:main',
            'interpo_time_control = x6_control_python.interpo_time_control:main',
            'lk_filter_demo = x6_control_python.lk_filter_demo:main',
        ],
    },
)
