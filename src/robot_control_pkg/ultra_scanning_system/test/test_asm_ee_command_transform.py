import math

from ultra_scanning_system.asm_ee_command_math import (
    IDENTITY_QUAT,
    compose_transform,
    compute_base_to_tool_command,
    invert_transform,
)


def assert_vector_close(actual, expected, tol=1e-9):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert math.isclose(actual_value, expected_value, abs_tol=tol)


def test_compute_base_to_tool_command_removes_tool_to_ee_offset():
    base_to_ee = ((1.0, 2.0, 3.0), IDENTITY_QUAT)
    tool_to_ee = ((0.0, 0.0, 0.2), IDENTITY_QUAT)

    base_to_tool = compute_base_to_tool_command(base_to_ee, tool_to_ee)

    assert_vector_close(base_to_tool[0], (1.0, 2.0, 2.8))
    assert_vector_close(base_to_tool[1], IDENTITY_QUAT)


def test_compute_base_to_tool_command_preserves_desired_ee_after_recompose():
    sqrt_half = math.sqrt(0.5)
    base_to_ee = ((0.4, -0.1, 0.5), (0.0, 0.0, sqrt_half, sqrt_half))
    tool_to_ee = ((0.1, 0.0, 0.0), (sqrt_half, 0.0, 0.0, sqrt_half))

    base_to_tool = compute_base_to_tool_command(base_to_ee, tool_to_ee)
    recomposed = compose_transform(base_to_tool, tool_to_ee)

    assert_vector_close(recomposed[0], base_to_ee[0])
    assert_vector_close(recomposed[1], base_to_ee[1])


def test_invert_transform_is_inverse_under_composition():
    sqrt_half = math.sqrt(0.5)
    transform = ((0.1, -0.2, 0.3), (0.0, sqrt_half, 0.0, sqrt_half))

    identity = compose_transform(transform, invert_transform(transform))

    assert_vector_close(identity[0], (0.0, 0.0, 0.0))
    assert_vector_close(identity[1], IDENTITY_QUAT)
