from setuptools import find_packages, setup

package_name = 'lkactuator_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='hyc',
    maintainer_email='hyc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lk_demo = lkactuator_python.lk_demo:main',
            'service = lkactuator_python.lk_service:main',
            'service_time = lkactuator_python.lk_service_timecontrol:main'
        ],
    },
)
#逗号