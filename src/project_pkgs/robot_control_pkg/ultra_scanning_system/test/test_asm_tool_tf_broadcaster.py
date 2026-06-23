from ultra_scanning_system.asm_tool_kinematics import (
    has_required_joint_states,
)


def test_v1_requires_first_two_joint_states_only():
    assert has_required_joint_states(1, True, True, False)
    assert not has_required_joint_states(1, True, False, True)


def test_v2_requires_three_joint_states():
    assert has_required_joint_states(2, True, True, True)
    assert not has_required_joint_states(2, True, True, False)


def test_v3_requires_first_two_joint_states_only():
    assert has_required_joint_states(3, True, True, False)
    assert not has_required_joint_states(3, True, False, True)
