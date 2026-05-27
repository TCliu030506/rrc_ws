from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dg_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.10/site-packages/' + package_name, glob('dg_python/*.txt')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyx',
    maintainer_email='1667599165@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dg_control = dg_python.dg_control:main',
            'dg_rokae = dg_python.dg_rokae:main',
            'dg_ur5 = dg_python.dg_ur5:main',
            'dg_force_record = dg_python.dg_force_record:main',
            'dg_keyboard_control = dg_python.dg_keyboard_control:main',
            'dg_omni_control = dg_python.dg_omni_control:main',
            'dg_omni_control_ui = dg_python.dg_omni_control_ui:main',
        ],
    },
)
