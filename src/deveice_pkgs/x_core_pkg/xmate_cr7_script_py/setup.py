from setuptools import setup

package_name = 'xmate_cr7_script_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='ROS 2 Python client for CR7 script service (compatible with coordinate/srv/StringScript).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cr7_script_client_node = xmate_cr7_script_py.cr7_script:main',
            'cr7_script_demo_node = xmate_cr7_script_py.cr7_script_demo:main',
        ],
    },
)
