import math

from ultra_scanning_system.contact_scan_trajectory_logic import (
    ContactScanState,
    align_contact_path_start,
    apply_contact_offset,
    compute_approach_axis,
    compute_retract_pose,
    normal_force,
    ramp_toward,
    should_enter_contact,
    should_fault_by_force,
    should_fault_by_search_distance,
)


def assert_vector_close(actual, expected, tol=1e-9):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert math.isclose(actual_value, expected_value, abs_tol=tol)


def test_compute_approach_axis_uses_tool_z_axis_with_sign():
    quat_identity = (0.0, 0.0, 0.0, 1.0)

    assert_vector_close(
        compute_approach_axis(quat_identity, approach_axis_sign=1.0),
        (0.0, 0.0, 1.0),
    )
    assert_vector_close(
        compute_approach_axis(quat_identity, approach_axis_sign=-1.0),
        (0.0, 0.0, -1.0),
    )


def test_apply_contact_offset_moves_each_path_point_along_its_tool_axis():
    pose = ((0.4, -0.1, 0.2), (0.0, 0.0, 0.0, 1.0))

    shifted = apply_contact_offset([pose], delta_c=0.03, approach_axis_sign=1.0)

    assert_vector_close(shifted[0][0], (0.4, -0.1, 0.23))
    assert_vector_close(shifted[0][1], pose[1])


def test_align_contact_path_start_matches_stable_contact_pose():
    path = [
        ((0.10, 0.20, 0.30), (0.0, 0.0, 0.0, 1.0)),
        ((0.20, 0.20, 0.31), (0.0, 0.0, 0.0, 1.0)),
    ]
    stable_pose = (
        (0.11, 0.18, 0.305),
        (0.1, 0.2, 0.3, 0.9),
    )

    aligned = align_contact_path_start(path, stable_pose=stable_pose)

    assert_vector_close(aligned[0][0], stable_pose[0])
    assert_vector_close(aligned[0][1], stable_pose[1])
    assert_vector_close(aligned[1][0], (0.21, 0.18, 0.315))
    assert_vector_close(aligned[1][1], path[1][1])


def test_normal_force_uses_configured_axis_and_sign():
    wrench = (0.5, -1.0, -4.2, 0.0, 0.0, 0.0)

    assert math.isclose(normal_force(wrench, axis='z', force_axis_sign=-1.0), 4.2)
    assert math.isclose(normal_force(wrench, axis='y', force_axis_sign=1.0), -1.0)


def test_contact_and_fault_predicates_are_threshold_based():
    assert should_enter_contact(2.1, contact_force_threshold=2.0)
    assert not should_enter_contact(1.9, contact_force_threshold=2.0)
    assert should_fault_by_force(12.0, max_contact_force=10.0)
    assert not should_fault_by_force(8.0, max_contact_force=10.0)
    assert should_fault_by_search_distance(0.051, max_search_distance=0.05)
    assert not should_fault_by_search_distance(0.049, max_search_distance=0.05)


def test_ramp_toward_limits_force_reference_change_per_cycle():
    assert math.isclose(ramp_toward(1.0, 5.0, max_rate=2.0, dt=0.5), 2.0)
    assert math.isclose(ramp_toward(4.8, 5.0, max_rate=2.0, dt=0.5), 5.0)
    assert math.isclose(ramp_toward(5.0, 1.0, max_rate=2.0, dt=0.5), 4.0)
    assert math.isclose(ramp_toward(5.0, 1.0, max_rate=0.0, dt=0.5), 1.0)
    assert math.isclose(ramp_toward(1.0, 5.0, max_rate=2.0, dt=0.0), 1.0)


def test_compute_retract_pose_moves_opposite_final_approach_axis():
    final_pose = ((0.4, -0.1, 0.2), (0.0, 0.0, 0.0, 1.0))

    retract_pose = compute_retract_pose(
        final_pose,
        retract_distance=0.04,
        approach_axis_sign=1.0,
    )

    assert_vector_close(retract_pose[0], (0.4, -0.1, 0.16))
    assert_vector_close(retract_pose[1], final_pose[1])


def test_state_enum_has_expected_order():
    assert ContactScanState.APPROACH.value == 'approach'
    assert ContactScanState.PRE_CONTACT.value == 'pre_contact'
    assert ContactScanState.CONTACT_SETTLE.value == 'contact_settle'
    assert ContactScanState.CONTACT_SCAN.value == 'contact_scan'
    assert ContactScanState.RETRACT.value == 'retract'
    assert ContactScanState.FINISHED.value == 'finished'
    assert ContactScanState.FAULT.value == 'fault'
