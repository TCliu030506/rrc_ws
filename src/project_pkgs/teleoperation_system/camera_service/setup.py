from setuptools import find_packages, setup

package_name = 'camera_service'

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
    maintainer='liutiancheng',
    maintainer_email='3210101491@zju.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_object_server = camera_service.service_object_server:main',
            'blood_image_detect = camera_service.blood_image_detect:main',
        ],
    },
)
