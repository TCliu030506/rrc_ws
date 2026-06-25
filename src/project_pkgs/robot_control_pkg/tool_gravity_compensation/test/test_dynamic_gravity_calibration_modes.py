import pytest

from tool_gravity_compensation import dynamic_gravity_calibration_node


V2_LINK_FRAMES = [
    'asm_tool_base_link',
    'asm_tool_link1',
    'asm_tool_link2',
    'asm_tool_link3',
]


def test_unconstrained_calls_legacy_solver_with_unchanged_arguments(monkeypatch):
    samples = [object()]
    expected = object()
    calls = []

    def fake_solver(actual_samples, *, link_count):
        calls.append((actual_samples, link_count))
        return expected

    monkeypatch.setattr(
        dynamic_gravity_calibration_node,
        'solve_dynamic_gravity_params',
        fake_solver,
    )

    result = dynamic_gravity_calibration_node.solve_calibration_samples(
        samples,
        ['link_a', 'link_b'],
        'unconstrained',
        {'ignored': 'legacy mode must not consume options'},
    )

    assert result is expected
    assert calls == [(samples, 2)]


def test_constrained_v2_rejects_wrong_link_frame_order():
    with pytest.raises(ValueError, match='link_frames'):
        dynamic_gravity_calibration_node.solve_calibration_samples(
            [],
            list(reversed(V2_LINK_FRAMES)),
            'constrained_v2',
            {},
        )


def test_constrained_v2_calls_new_solver_with_options(monkeypatch):
    samples = [object()]
    expected = object()
    options = {
        'known_base_mass': 0.8,
        'known_moving_total_mass': 1.2,
        'fixed_force_bias': [0.1, 0.2, 0.3],
        'fixed_torque_bias': [0.01, 0.02, 0.03],
        'com_xy_prior_std_m': 0.03,
        'moving_mass_prior': [0.3, 0.4, 0.5],
        'moving_mass_prior_std_kg': 0.2,
        'first_moment_min_norm_std_kg_m': 1000000.0,
        'min_link_mass_kg': 0.05,
        'max_abs_com_m': 0.25,
    }
    calls = []

    def fake_solver(**kwargs):
        calls.append(kwargs)
        return expected

    monkeypatch.setattr(
        dynamic_gravity_calibration_node,
        'solve_constrained_dynamic_gravity_params',
        fake_solver,
    )

    result = dynamic_gravity_calibration_node.solve_calibration_samples(
        samples,
        V2_LINK_FRAMES,
        'constrained_v2',
        options,
    )

    assert result is expected
    assert calls == [{
        'samples': samples,
        'base_mass': 0.8,
        'moving_total_mass': 1.2,
        'force_bias': [0.1, 0.2, 0.3],
        'torque_bias': [0.01, 0.02, 0.03],
        'com_xy_prior_std_m': 0.03,
        'moving_mass_prior': [0.3, 0.4, 0.5],
        'moving_mass_prior_std_kg': 0.2,
        'first_moment_min_norm_std_kg_m': 1000000.0,
        'min_link_mass_kg': 0.05,
    }]


def test_unknown_solver_mode_is_rejected():
    with pytest.raises(ValueError, match='solver_mode'):
        dynamic_gravity_calibration_node.solve_calibration_samples(
            [],
            [],
            'future_solver',
            {},
        )
