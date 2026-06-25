import math

import numpy as np
import pytest

from tool_gravity_compensation.constrained_dynamic_gravity_model import (
    solve_constrained_dynamic_gravity_params,
)
from tool_gravity_compensation.dynamic_gravity_model import (
    predict_dynamic_gravity_wrench,
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


def _v2_transforms(prismatic_z, link2_y, link3_x):
    link2_rotation = _rot_y(link2_y)
    link3_rotation = link2_rotation @ _rot_x(link3_x)
    link2_origin = np.array([0.0, 0.0, 0.08 + prismatic_z])
    link3_origin = link2_origin + link2_rotation @ np.array(
        [0.0, 0.0, 0.12]
    )
    return [
        {
            "rot": np.eye(3),
            "trans": np.zeros(3),
        },
        {
            "rot": np.eye(3),
            "trans": np.array([0.0, 0.0, prismatic_z]),
        },
        {
            "rot": link2_rotation,
            "trans": link2_origin,
        },
        {
            "rot": link3_rotation,
            "trans": link3_origin,
        },
    ]


def _direct_wrench(
    g_sensor,
    transforms,
    masses,
    first_moments,
    force_bias,
    torque_bias,
):
    force = np.asarray(force_bias, dtype=float).copy()
    torque = np.asarray(torque_bias, dtype=float).copy()
    for transform, mass, first_moment in zip(
        transforms, masses, first_moments
    ):
        link_force = mass * g_sensor
        link_com_sensor = (
            transform["trans"]
            + transform["rot"] @ (first_moment / mass)
        )
        force += link_force
        torque += np.cross(link_com_sensor, link_force)
    return force, torque


def _sample(
    g_sensor,
    transforms,
    masses,
    first_moments,
    force_bias,
    torque_bias,
):
    force, torque = _direct_wrench(
        g_sensor,
        transforms,
        masses,
        first_moments,
        force_bias,
        torque_bias,
    )
    return {
        "g_sensor": g_sensor,
        "transforms": transforms,
        "force": force,
        "torque": torque,
    }


def _pose(index):
    sensor_rotation = (
        _rot_z(-0.19 + 0.07 * index)
        @ _rot_y(0.31 * math.sin(0.37 * index))
        @ _rot_x(-0.27 * math.cos(0.23 * index))
    )
    g_sensor = sensor_rotation.T @ np.array([0.0, 0.0, -9.81])
    transforms = _v2_transforms(
        prismatic_z=0.025 + 0.002 * (index % 5),
        link2_y=-0.45 + 0.09 * (index % 9),
        link3_x=0.35 * math.sin(0.29 * index),
    )
    return g_sensor, transforms


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


def _holdout_fixture():
    masses = np.array([0.8, 0.3, 0.4, 0.5])
    first_moments = np.array([
        [0.0, 0.0, 0.016],
        [0.0, 0.0, -0.006],
        [0.0, 0.0, 0.012],
        [0.0, 0.0, -0.015],
    ])
    force_bias = np.array([0.21, -0.14, 0.32])
    torque_bias = np.array([0.012, -0.008, 0.017])
    samples = []
    for index in range(18):
        g_sensor, transforms = _pose(index)
        samples.append(_sample(
            g_sensor,
            transforms,
            masses,
            first_moments,
            force_bias,
            torque_bias,
        ))
    return masses, first_moments, force_bias, torque_bias, samples


def _degenerate_fixture():
    true_masses = np.array([1.05, 0.22, 0.48, 0.65])
    first_moments = np.array([
        [0.012, -0.009, 0.016],
        [-0.008, 0.006, -0.006],
        [0.007, 0.011, 0.012],
        [-0.006, -0.008, -0.015],
    ])
    force_bias = np.array([0.21, -0.14, 0.32])
    torque_bias = np.array([0.012, -0.008, 0.017])
    g_sensor = np.array([0.0, 0.0, -9.81])
    transforms = _v2_transforms(0.03, 0.0, 0.0)
    sample = _sample(
        g_sensor,
        transforms,
        true_masses,
        first_moments,
        force_bias,
        torque_bias,
    )
    return true_masses, [sample]


def test_hard_mass_constraints_hold_for_conflicting_degenerate_data():
    true_masses, samples = _degenerate_fixture()

    result = _solve(samples)

    assert true_masses[0] != pytest.approx(0.8)
    assert np.sum(true_masses[1:]) != pytest.approx(1.2)
    assert result["link_masses"][0] == pytest.approx(0.8, abs=1e-12)
    assert np.sum(result["link_masses"][1:]) == pytest.approx(
        1.2, abs=1e-12
    )
    assert result["rms_residual"] > 0.1
    assert result["mass_constraint_error"] <= 1e-9


def test_mass_prior_and_xy_regularization_control_degenerate_solution():
    _, samples = _degenerate_fixture()
    alternate_prior = np.array([0.5, 0.4, 0.3])

    default = _solve(samples)
    repeated = _solve(samples)
    alternate = _solve(samples, moving_mass_prior=alternate_prior)
    tight_xy = _solve(samples, com_xy_prior_std_m=1e-4)
    loose_xy = _solve(samples, com_xy_prior_std_m=10.0)

    np.testing.assert_allclose(
        default["link_masses"][1:], [0.3, 0.4, 0.5], atol=1e-10
    )
    np.testing.assert_allclose(
        alternate["link_masses"][1:], alternate_prior, atol=1e-10
    )
    np.testing.assert_allclose(
        repeated["link_first_moments"],
        default["link_first_moments"],
        atol=0.0,
    )
    assert np.all(np.isfinite(default["link_first_moments"]))
    assert np.linalg.norm(
        tight_xy["link_first_moments"][:, :2]
    ) < np.linalg.norm(loose_xy["link_first_moments"][:, :2])


def test_first_moment_min_norm_regularization_shrinks_z_moments():
    masses = np.array([0.8, 0.3, 0.4, 0.5])
    first_moments = np.array([
        [0.0, 0.0, 0.01],
        [0.0, 0.0, 0.01],
        [0.0, 0.0, 0.01],
        [0.0, 0.0, 0.01],
    ])
    g_sensor = np.array([1e-6, 0.0, 0.0])
    transforms = _v2_transforms(0.0, 0.0, 0.0)
    sample = _sample(
        g_sensor,
        transforms,
        masses,
        first_moments,
        np.array([0.21, -0.14, 0.32]),
        np.array([0.012, -0.008, 0.017]),
    )

    weak = _solve(
        [sample], first_moment_min_norm_std_kg_m=1e6
    )
    strong = _solve(
        [sample], first_moment_min_norm_std_kg_m=1.0
    )

    weak_z_norm = np.linalg.norm(weak["link_first_moments"][:, 2])
    strong_z_norm = np.linalg.norm(strong["link_first_moments"][:, 2])
    assert np.isfinite(weak_z_norm)
    assert weak_z_norm > 1e3 * strong_z_norm


def test_returns_rank_compatibility_alias():
    _, _, _, _, samples = _holdout_fixture()

    result = _solve(samples)

    assert result["rank"] == result["augmented_rank"]


def test_holdout_pose_wrench_prediction_matches_equivalent_model():
    masses, first_moments, force_bias, torque_bias, samples = (
        _holdout_fixture()
    )
    result = _solve(samples)
    g_sensor, transforms = _pose(41)
    expected_force, expected_torque = _direct_wrench(
        g_sensor,
        transforms,
        masses,
        first_moments,
        force_bias,
        torque_bias,
    )
    predicted_force, predicted_torque = predict_dynamic_gravity_wrench(
        g_sensor, transforms, result
    )

    np.testing.assert_allclose(predicted_force, expected_force, atol=1e-8)
    np.testing.assert_allclose(predicted_torque, expected_torque, atol=1e-6)


@pytest.mark.parametrize("active_bias", ["force", "torque"])
def test_fixed_bias_is_removed_from_physical_fit_and_restored_in_prediction(
    active_bias,
):
    masses = np.array([0.8, 0.3, 0.4, 0.5])
    first_moments = np.array([
        [0.0, 0.0, 0.016],
        [0.0, 0.0, -0.006],
        [0.0, 0.0, 0.012],
        [0.0, 0.0, -0.015],
    ])
    force_bias = np.zeros(3)
    torque_bias = np.zeros(3)
    if active_bias == "force":
        force_bias = np.array([1.7, -2.1, 0.9])
    else:
        torque_bias = np.array([0.4, -0.3, 0.2])

    samples = []
    for index in range(18):
        g_sensor, transforms = _pose(index)
        samples.append(_sample(
            g_sensor,
            transforms,
            masses,
            first_moments,
            force_bias,
            torque_bias,
        ))

    result = _solve(
        samples,
        force_bias=force_bias,
        torque_bias=torque_bias,
    )
    holdout_g, holdout_transforms = _pose(41)
    expected_force, expected_torque = _direct_wrench(
        holdout_g,
        holdout_transforms,
        masses,
        first_moments,
        force_bias,
        torque_bias,
    )
    predicted_force, predicted_torque = predict_dynamic_gravity_wrench(
        holdout_g, holdout_transforms, result
    )

    assert result["rms_residual"] < 1e-9
    np.testing.assert_allclose(predicted_force, expected_force, atol=1e-8)
    np.testing.assert_allclose(predicted_torque, expected_torque, atol=1e-6)


def test_rejects_sample_without_exactly_four_transforms():
    _, _, _, _, samples = _holdout_fixture()
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
    _, _, _, _, samples = _holdout_fixture()

    with pytest.raises(ValueError, match=message):
        _solve(samples, **override)
