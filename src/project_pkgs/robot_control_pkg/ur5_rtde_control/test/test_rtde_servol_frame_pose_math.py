import math

import pytest

from ur5_rtde_control.rtde_servol_frame_pose_math import (
    IDENTITY_QUAT,
    compose_transform,
    compute_base_to_tool_command_from_frame,
    finite_transform,
)


def assert_vector_close(actual, expected, tol=1e-9):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert math.isclose(actual_value, expected_value, abs_tol=tol)


def test_compute_base_to_tool_command_from_base_frame():
    base_to_controlled = ((1.0, 2.0, 3.0), IDENTITY_QUAT)
    tool_to_controlled = ((0.0, 0.0, 0.2), IDENTITY_QUAT)

    base_to_tool = compute_base_to_tool_command_from_frame(
        command_to_controlled=base_to_controlled,
        command_frame='base',
        base_frame='base',
        controlled_frame='asm_ee_site',
        tool_frame='tool0',
        lookup_transform=lambda target, source: tool_to_controlled,
    )

    assert_vector_close(base_to_tool[0], (1.0, 2.0, 2.8))
    assert_vector_close(base_to_tool[1], IDENTITY_QUAT)


def test_compute_base_to_tool_command_uses_command_frame_transform():
    command_to_controlled = ((0.0, 0.0, 0.1), IDENTITY_QUAT)
    base_to_camera = ((1.0, 0.0, 0.0), IDENTITY_QUAT)
    tool_to_controlled = ((0.0, 0.0, 0.2), IDENTITY_QUAT)

    def lookup_transform(target, source):
        if (target, source) == ('base', 'camera'):
            return base_to_camera
        if (target, source) == ('tool0', 'asm_ee_site'):
            return tool_to_controlled
        raise AssertionError((target, source))

    base_to_tool = compute_base_to_tool_command_from_frame(
        command_to_controlled=command_to_controlled,
        command_frame='camera',
        base_frame='base',
        controlled_frame='asm_ee_site',
        tool_frame='tool0',
        lookup_transform=lookup_transform,
    )

    recomposed = compose_transform(base_to_tool, tool_to_controlled)
    assert_vector_close(recomposed[0], (1.0, 0.0, 0.1))
    assert_vector_close(recomposed[1], IDENTITY_QUAT)


def test_finite_transform_rejects_nan_values():
    assert not finite_transform(((math.nan, 0.0, 0.0), IDENTITY_QUAT))


def test_compute_base_to_tool_command_rejects_nan_command():
    with pytest.raises(ValueError, match='finite'):
        compute_base_to_tool_command_from_frame(
            command_to_controlled=((math.nan, 0.0, 0.0), IDENTITY_QUAT),
            command_frame='base',
            base_frame='base',
            controlled_frame='asm_ee_site',
            tool_frame='tool0',
            lookup_transform=lambda target, source: ((0.0, 0.0, 0.0), IDENTITY_QUAT),
        )
