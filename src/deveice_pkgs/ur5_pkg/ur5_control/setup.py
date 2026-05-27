from setuptools import find_packages, setup

package_name = 'ur5_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ltc',
    maintainer_email='3210101491@zju.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'move_demo = ur5_control.move_demo:main',
        'linearmotion_demo = ur5_control.linearmotion_demo:main',
        'ur5_pos_init = ur5_control.ur5_pos_init:main',
        'ur5_kinematics = ur5_control.ur5_kinematics:main',
        'ur5_node_test_kin = ur5_control.ur5_node_test_kin:main',
        'my_demo = ur5_control.my_demo:main',
        ],
    },
)
