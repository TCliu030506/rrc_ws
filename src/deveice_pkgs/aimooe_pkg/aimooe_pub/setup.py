from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aimooe_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        '':[]
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rus',
    maintainer_email='3150103428@zju.edu.cn',
    description='communicate to aimooe_tracker',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'tool_add = {package_name}.tool_add:main',
            f'tool_calib = {package_name}.tool_calib:main',
            f'tool_tip = {package_name}.tool_tip:main',
            f'tool_transfer = {package_name}.tool_transfer:main',
            f'pub_trackers = {package_name}.pub_trackers:main',
            f'pub_test = {package_name}.pub_test:main',
            f'pub_coords = {package_name}.pub_coords:main',
            f'aim_demo = {package_name}.aim:main',
            f'pub_marker = {package_name}.pub_marker:main',
            f'pub_trackers_marker = {package_name}.pub_trackers_marker:main',
            f'pub_trackers_8markers = {package_name}.pub_trackers_8markers:main',
            f'szmd_record_demo = {package_name}.szmd_record_demo:main',
            f'szmd_filter_demo = {package_name}.szmd_filter_demo:main',
        ],
    },
)
