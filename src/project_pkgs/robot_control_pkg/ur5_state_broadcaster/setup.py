from setuptools import find_packages, setup
from glob import glob

package_name = 'ur5_state_broadcaster'

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
            'tcp_twist_estimator = ur5_state_broadcaster.tcp_twist_estimator:main',
        ],
    },
)
