import math

import numpy as np

from tool_gravity_compensation.dynamic_gravity_model import (
    predict_dynamic_gravity_wrench,
    solve_dynamic_gravity_params,
)


def _rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ])


def _rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ])


def _rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _sample(g_sensor, transforms, masses, coms, force_bias, torque_bias):
    force = force_bias.copy()
    torque = torque_bias.copy()

    for transform, mass, com in zip(transforms, masses, coms):
        rot = transform["rot"]
        trans = transform["trans"]
        com_sensor = trans + rot @ com
        force_i = mass * g_sensor
        torque_i = np.cross(com_sensor, force_i)
        force += force_i
        torque += torque_i

    return {
        "g_sensor": g_sensor,
        "transforms": transforms,
        "force": force,
        "torque": torque,
    }


def test_solve_dynamic_gravity_params_recovers_link_masses_coms_and_biases():
    masses = np.array([0.42, 1.15])
    coms = np.array([
        [0.012, -0.004, 0.031],
        [-0.018, 0.009, 0.046],
    ])
    force_bias = np.array([0.22, -0.13, 0.31])
    torque_bias = np.array([0.011, -0.007, 0.019])

    samples = []
    for index, angles in enumerate([
        (0.0, 0.0, 0.0),
        (0.25, -0.2, 0.1),
        (-0.35, 0.15, -0.25),
        (0.45, 0.3, 0.2),
        (-0.15, -0.45, 0.35),
        (0.32, -0.28, -0.4),
    ]):
        world_to_sensor = _rot_z(angles[2]) @ _rot_y(angles[1]) @ _rot_x(angles[0])
        g_sensor = world_to_sensor.T @ np.array([0.0, 0.0, -9.81])
        transforms = [
            {
                "rot": _rot_y(0.2 + 0.05 * index),
                "trans": np.array([0.0, 0.0, 0.024]),
            },
            {
                "rot": _rot_x(-0.3 + 0.08 * index),
                "trans": np.array([0.0, 0.0, 0.105]),
            },
        ]
        samples.append(_sample(g_sensor, transforms, masses, coms, force_bias, torque_bias))

    result = solve_dynamic_gravity_params(samples, link_count=2)

    np.testing.assert_allclose(result["link_masses"], masses, atol=1e-9)
    np.testing.assert_allclose(result["link_coms"], coms, atol=1e-9)
    np.testing.assert_allclose(result["force_bias"], force_bias, atol=1e-9)
    np.testing.assert_allclose(result["torque_bias"], torque_bias, atol=1e-9)
    assert result["rank"] == 14
    assert result["rms_residual"] < 1e-10


def test_predict_dynamic_gravity_wrench_matches_link_sum_model():
    params = {
        "link_masses": [0.42, 1.15],
        "link_coms": [
            np.array([0.012, -0.004, 0.031]),
            np.array([-0.018, 0.009, 0.046]),
        ],
        "force_bias": np.array([0.22, -0.13, 0.31]),
        "torque_bias": np.array([0.011, -0.007, 0.019]),
    }
    g_sensor = np.array([1.2, -3.4, -9.1])
    transforms = [
        {"rot": _rot_y(0.3), "trans": np.array([0.0, 0.0, 0.024])},
        {"rot": _rot_x(-0.2), "trans": np.array([0.0, 0.0, 0.105])},
    ]
    expected = _sample(
        g_sensor,
        transforms,
        np.asarray(params["link_masses"]),
        np.asarray(params["link_coms"]),
        params["force_bias"],
        params["torque_bias"],
    )

    force, torque = predict_dynamic_gravity_wrench(g_sensor, transforms, params)

    np.testing.assert_allclose(force, expected["force"])
    np.testing.assert_allclose(torque, expected["torque"])
