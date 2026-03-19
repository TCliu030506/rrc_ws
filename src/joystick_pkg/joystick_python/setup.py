from setuptools import find_packages, setup

package_name = 'joystick_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'
                      'inspire_python'
                      'mtactuator_python'
                      'ur5_control'],
    zip_safe=True,
    maintainer='lzc',
    maintainer_email='493237542@qq.com',
    description='joystick communicate with jy&mt actuator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_control = joystick_python.joystick_control:main',
            'joystick_control_extended = joystick_python.joystick_control_extended:main'
        ],
    },
)
