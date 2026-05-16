from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'asm_description_mujoco'


def _find_package_root() -> Path:
    here = Path(__file__).resolve()
    candidates = [here.parent, *here.parents]
    for base in candidates:
        if (base / 'ur5e_with_asm' / 'assets').is_dir():
            return base
        src_candidate = base / 'src' / 'robot_control_pkg' / package_name
        if (src_candidate / 'ur5e_with_asm' / 'assets').is_dir():
            return src_candidate
        src_candidate = base / 'src' / package_name
        if (src_candidate / 'ur5e_with_asm' / 'assets').is_dir():
            return src_candidate
    return here.parent


def _rel_glob(base: Path, pattern: str) -> list:
    return [str(path.relative_to(base)) for path in base.glob(pattern)]


package_root = _find_package_root()
ur5e_root = package_root / 'ur5e_with_asm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/ur5e_with_asm',
            _rel_glob(package_root, 'ur5e_with_asm/*.xml')
            + _rel_glob(package_root, 'ur5e_with_asm/*.png'),
        ),
        (
            'share/' + package_name + '/ur5e_with_asm/assets',
            _rel_glob(package_root, 'ur5e_with_asm/assets/*'),
        ),
        (
            'share/' + package_name + '/launch',
            _rel_glob(package_root, 'launch/*.py'),
        ),
    ],
    install_requires=['setuptools', 'numpy', 'mujoco', 'glfw'],
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
            'asm_ros2_node = asm_description_mujoco.asm_ros2_node:main',
            'asm_keyboard_teleop = asm_description_mujoco.keyboard_teleop:main',
            'mujoco_tf_broadcaster = asm_description_mujoco.mujoco_tf_broadcaster:main',
        ],
    },
)
