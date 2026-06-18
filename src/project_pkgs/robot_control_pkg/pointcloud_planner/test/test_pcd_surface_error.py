import importlib.util
from pathlib import Path

import numpy as np


def load_module():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "pcd_surface_error.py"
    )
    spec = importlib.util.spec_from_file_location("pcd_surface_error", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_load_ascii_xyz_pcd(tmp_path):
    module = load_module()
    pcd_path = tmp_path / "cloud.pcd"
    pcd_path.write_text(
        "# .PCD v0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH 2\n"
        "HEIGHT 1\n"
        "POINTS 2\n"
        "DATA ascii\n"
        "0 1 2\n"
        "3 4 5\n",
        encoding="ascii",
    )

    points = module.load_ascii_xyz_pcd(pcd_path)

    np.testing.assert_allclose(points, [[0, 1, 2], [3, 4, 5]])


def test_compute_nearest_neighbor_errors_and_statistics():
    module = load_module()
    model = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    measured = np.array([[0.0, 0.0, 0.1], [1.0, 0.0, 0.2]])

    errors = module.compute_nearest_neighbor_errors(model, measured)
    stats = module.compute_statistics(errors)

    np.testing.assert_allclose(errors, [0.1, 0.2])
    assert np.isclose(stats["mean"], 0.15)
    assert np.isclose(stats["rmse"], np.sqrt(0.025))
    assert np.isclose(stats["max"], 0.2)
