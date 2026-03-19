from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tus_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 目录下的所有 launch 文件
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.*'))),
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
            'tus_ui = tus_ui.tus_ui:main',
            'img_capture_node = tus_ui.img_capture_node:main'
        ],
    },
)
