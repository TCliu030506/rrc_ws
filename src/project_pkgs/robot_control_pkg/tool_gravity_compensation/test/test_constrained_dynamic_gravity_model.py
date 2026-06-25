import math

import numpy as np
import pytest

from tool_gravity_compensation.constrained_dynamic_gravity_model import (
    solve_constrained_dynamic_gravity_params,
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


def _transforms(index):
    phase = 0.11 * index
    return [
        {
            "rot": _rot_z(0.13 + phase) @ _rot_y(-0.08 + 0.03 * index),
            "trans": np.array([0.012, -0.006, 0.025]),
        },
        {
            "rot": _rot_x(-0.21 + 0.04 * index) @ _rot_z(0.07 - phase),
            "trans": np.array([-0.018, 0.011, 0.092]),
        },
        {
            "rot": _rot_y(0.28 - 0.02 * index) @ _rot_x(0.09 + phase),
            "trans": np.array([0.023, 0.016, 0.151]),
        },
        {
            "rot": _rot_z(-0.17 + 0.05 * index) @ _rot_y(0.19 - phase),
            "trans": np.array([-0.014, -0.021, 0.224]),
        },
    ]


def _sample(index, masses, first_moments, force_bias, torque_bias):
    sensor_rotation = (
        _rot_z(-0.19 + 0.07 * index)
        @ _rot_y(0.31 * math.sin(0.37 * index))
        @ _rot_x(-0.27 * math.cos(0.23 * index))
    )
    g_sensor = sensor_rotation.T @ np.array([0.0, 0.0, -9.81])
    transforms = _transforms(index)
    force = force_bias.copy()
    torque = torque_bias.copy()

    for transform, mass, first_moment in zip(
        transforms, masses, first_moments
    ):
        force_i = mass * g_sensor
        force += force_i
        torque += np.cross(transform["trans"], force_i)
        torque += np.cross(transform["rot"] @ first_moment, g_sensor)

    return {
        "g_sensor": g_sensor,
        "transforms": transforms,
        "force": force,
        "torque": torque,
    }


def _solve(samples, **overrides):
    arguments = {
        "samples": samples,
        "base_mass": 0.8,
        "moving_total_mass": 1.2,
        "force_bias": np.array([0.21, -0.14, 0.32]),
        "torque_bias": np.array([0.012, -0.008, 0.017]),
        "com_xy_prior_std_m": 0.03,
        "moving_mass_prior": np.array([0.3, 0.4, 0.5]),
        "moving_mass_prior_std_kg": 0.2,
        "first_moment_min_norm_std_kg_m": 1e6,
        "min_link_mass_kg": 0.05,
    }
    arguments.update(overrides)
    return solve_constrained_dynamic_gravity_params(**arguments)


def _fixture():
    masses = np.array([0.8, 0.3, 0.4, 0.5])
    first_moments = np.array([
        [0.0, 0.0, 0.016],
        [0.0, 0.0, -0.006],
        [0.0, 0.0, 0.012],
        [0.0, 0.0, -0.015],
    ])
    force_bias = np.array([0.21, -0.14, 0.32])
    torque_bias = np.array([0.012, -0.008, 0.017])
    samples = [
        _sample(index, masses, first_moments, force_bias, torque_bias)
        for index in range(18)
    ]
    return masses, first_moments, force_bias, torque_bias, samples


def test_fixed_mass_prior_and_hard_constraints_are_preserved():
    masses, _, force_bias, torque_bias, samples = _fixture()

    result = _solve(samples)

    np.testing.assert_allclose(result["force_bias"], force_bias)
    np.testing.assert_allclose(result["torque_bias"], torque_bias)
    np.testing.assert_allclose(result["link_masses"], masses, atol=1e-10)
    np.testing.assert_allclose(
        result["link_first_moments"][:, :2], 0.0, atol=1e-10
    )
    np.testing.assert_allclose(
        result["link_coms"],
        result["link_first_moments"] / result["link_masses"][:, None],
    )
    assert result["sample_count"] == len(samples)
    assert result["data_rank"] > 0
    assert result["reduced_data_rank"] > 0
    assert result["augmented_rank"] == 14
    assert np.isfinite(result["condition_number"])
    assert result["rms_residual"] < 1e-9
    assert result["mass_constraint_error"] <= 1e-9


def test_holdout_pose_wrench_prediction_matches_equivalent_model():
    masses, first_moments, force_bias, torque_bias, samples = _fixture()
    result = _solve(samples)
    holdout = _sample(
        41, masses, first_moments, force_bias, torque_bias
    )

    predicted = _sample(
        41,
        result["link_masses"],
        result["link_first_moments"],
        result["force_bias"],
        result["torque_bias"],
    )

    np.testing.assert_allclose(predicted["force"], holdout["force"], atol=1e-8)
    np.testing.assert_allclose(
        predicted["torque"], holdout["torque"], atol=1e-8
    )


def test_rejects_sample_without_exactly_four_transforms():
    _, _, _, _, samples = _fixture()
    samples[0] = dict(samples[0], transforms=samples[0]["transforms"][:3])

    with pytest.raises(ValueError, match="transform count must be 4"):
        _solve(samples)


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"base_mass": 0.0}, "base_mass"),
        ({"moving_total_mass": -1.0}, "moving_total_mass"),
        ({"moving_mass_prior": [0.3, 0.4, 0.6]}, "sum"),
        ({"moving_mass_prior": [0.3, 0.0, 0.9]}, "moving_mass_prior"),
        ({"moving_mass_prior_std_kg": 0.0}, "moving_mass_prior_std_kg"),
        ({"com_xy_prior_std_m": np.inf}, "com_xy_prior_std_m"),
        ({"force_bias": [0.0, np.nan, 0.0]}, "force_bias"),
        ({"min_link_mass_kg": 0.0}, "min_link_mass_kg"),
    ],
)
def test_rejects_invalid_solver_inputs(override, message):
    _, _, _, _, samples = _fixture()

    with pytest.raises(ValueError, match=message):
        _solve(samples, **override)
