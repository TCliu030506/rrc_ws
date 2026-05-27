from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'interfaces_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'pandas',
        ],
    zip_safe=True,
    maintainer='rus',
    maintainer_email='3150103428@zju.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'msg_filter_demo = interfaces_py.msg_filter_demo:main',
        ],
    },
)
