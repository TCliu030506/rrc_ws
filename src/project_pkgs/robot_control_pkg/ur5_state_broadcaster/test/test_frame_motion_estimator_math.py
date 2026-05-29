import math

import numpy as np

from ur5_state_broadcaster.frame_motion_estimator_math import FrameMotionEstimator


def assert_vector_close(actual, expected, tol=1e-9):
    assert np.allclose(np.asarray(actual), np.asarray(expected), atol=tol)


def test_estimates_linear_velocity_and_acceleration_in_source_frame():
    estimator = FrameMotionEstimator(
        min_dt=1e-4,
        velocity_filter_tau=0.0,
        accel_filter_tau=0.0,
        express_in_target_frame=False,
    )

    assert estimator.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)) is None

    first = estimator.update(0.1, (0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert_vector_close(first.linear_velocity, (1.0, 0.0, 0.0))
    assert_vector_close(first.linear_acceleration, (0.0, 0.0, 0.0))

    second = estimator.update(0.2, (0.3, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert_vector_close(second.linear_velocity, (2.0, 0.0, 0.0))
    assert_vector_close(second.linear_acceleration, (10.0, 0.0, 0.0))


def test_expresses_velocity_and_acceleration_in_target_frame():
    estimator = FrameMotionEstimator(
        min_dt=1e-4,
        velocity_filter_tau=0.0,
        accel_filter_tau=0.0,
        express_in_target_frame=True,
    )
    q_base_target = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))

    estimator.update(0.0, (0.0, 0.0, 0.0), q_base_target)
    first = estimator.update(0.1, (0.0, 0.1, 0.0), q_base_target)
    second = estimator.update(0.2, (0.0, 0.3, 0.0), q_base_target)

    assert_vector_close(first.linear_velocity, (1.0, 0.0, 0.0))
    assert_vector_close(second.linear_velocity, (2.0, 0.0, 0.0))
    assert_vector_close(second.linear_acceleration, (10.0, 0.0, 0.0))


def test_estimates_angular_velocity_and_acceleration():
    estimator = FrameMotionEstimator(
        min_dt=1e-4,
        velocity_filter_tau=0.0,
        accel_filter_tau=0.0,
        express_in_target_frame=False,
    )

    estimator.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    first = estimator.update(
        0.1,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(0.05), math.cos(0.05)),
    )
    second = estimator.update(
        0.2,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(0.15), math.cos(0.15)),
    )

    assert_vector_close(first.angular_velocity, (0.0, 0.0, 1.0))
    assert_vector_close(first.angular_acceleration, (0.0, 0.0, 0.0))
    assert_vector_close(second.angular_velocity, (0.0, 0.0, 2.0))
    assert_vector_close(second.angular_acceleration, (0.0, 0.0, 10.0))
