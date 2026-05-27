from types import SimpleNamespace

from robot_trajectory_planner.current_pose_hold_logic import (
    copy_pose,
    latch_initial_pose,
    set_zero_accel,
    set_zero_twist,
    validate_publish_rate,
)


def make_pose():
    return SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.4),
    )


def make_twist_like():
    return SimpleNamespace(
        linear=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        angular=SimpleNamespace(x=4.0, y=5.0, z=6.0),
    )


def test_copy_pose_copies_position_and_orientation():
    source = make_pose()
    target = make_pose()
    target.position.x = -1.0

    copy_pose(source, target)

    assert target.position.x == 1.0
    assert target.position.y == 2.0
    assert target.position.z == 3.0
    assert target.orientation.x == 0.1
    assert target.orientation.y == 0.2
    assert target.orientation.z == 0.3
    assert target.orientation.w == 0.4


def test_zero_motion_helpers_clear_linear_and_angular_fields():
    twist = make_twist_like()
    accel = make_twist_like()

    set_zero_twist(twist)
    set_zero_accel(accel)

    assert twist.linear.x == 0.0
    assert twist.linear.y == 0.0
    assert twist.linear.z == 0.0
    assert twist.angular.x == 0.0
    assert twist.angular.y == 0.0
    assert twist.angular.z == 0.0
    assert accel.linear.x == 0.0
    assert accel.linear.y == 0.0
    assert accel.linear.z == 0.0
    assert accel.angular.x == 0.0
    assert accel.angular.y == 0.0
    assert accel.angular.z == 0.0


def test_validate_publish_rate_rejects_non_positive_rate():
    try:
        validate_publish_rate(0.0)
    except ValueError as exc:
        assert "publish_rate" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_latch_initial_pose_keeps_first_pose_after_subsequent_updates():
    first = make_pose()
    second = make_pose()
    second.position.x = 9.0

    latched = latch_initial_pose(None, first, make_pose)
    latched_again = latch_initial_pose(latched, second, make_pose)

    assert latched_again is latched
    assert latched_again.position.x == 1.0
    assert latched_again.position.y == 2.0
