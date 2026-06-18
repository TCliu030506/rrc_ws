import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def load_aruco_node_module():
    package_root = Path(__file__).resolve().parents[1]
    module_path = package_root / "ros2_aruco" / "aruco_node.py"
    spec = importlib.util.spec_from_file_location("aruco_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rotation_matrix_to_quaternion_identity():
    aruco_node = load_aruco_node_module()

    quat = aruco_node.rotation_matrix_to_quaternion(np.eye(3))

    np.testing.assert_allclose(quat, [0.0, 0.0, 0.0, 1.0])


def test_image_msg_to_mono8_converts_rgb8():
    aruco_node = load_aruco_node_module()
    image = np.array([[[10, 20, 30], [100, 110, 120]]], dtype=np.uint8)
    msg = SimpleNamespace(
        height=1,
        width=2,
        encoding="rgb8",
        data=image.tobytes(),
    )

    mono = aruco_node.image_msg_to_mono8(msg)

    assert mono.shape == (1, 2)
    assert mono.dtype == np.uint8
    assert mono[0, 1] > mono[0, 0]


def test_marker_object_points_are_centered_on_marker():
    aruco_node = load_aruco_node_module()

    points = aruco_node.marker_object_points(0.04)

    np.testing.assert_allclose(
        points,
        [
            [-0.02, 0.02, 0.0],
            [0.02, 0.02, 0.0],
            [0.02, -0.02, 0.0],
            [-0.02, -0.02, 0.0],
        ],
    )
