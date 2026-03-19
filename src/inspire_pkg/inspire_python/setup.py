from setuptools import find_packages, setup

package_name = 'inspire_python'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=['inspire_python'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rus',
    maintainer_email='3150103428@zju.edu.cn',
    description='python module to send command',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'demo = {package_name}.demo:main',
            f'echo = {package_name}.echo:main',
            f'echo_copy = {package_name}.echo_copy:main',
            f'szmd_fc = {package_name}.szmd_forcecontrol:main',
            f'szmd = {package_name}.szmd:main',
            f'record = {package_name}.record:main',
            f'publish = {package_name}.publish:main'
        ],
    },
)
