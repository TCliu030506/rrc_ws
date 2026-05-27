def validate_publish_rate(publish_rate: float) -> float:
    rate = float(publish_rate)
    if rate <= 0.0:
        raise ValueError("publish_rate must be > 0")
    return rate


def copy_pose(source, target) -> None:
    target.position.x = float(source.position.x)
    target.position.y = float(source.position.y)
    target.position.z = float(source.position.z)
    target.orientation.x = float(source.orientation.x)
    target.orientation.y = float(source.orientation.y)
    target.orientation.z = float(source.orientation.z)
    target.orientation.w = float(source.orientation.w)


def latch_initial_pose(current_latched_pose, incoming_pose, pose_factory):
    if current_latched_pose is not None:
        return current_latched_pose

    latched_pose = pose_factory()
    copy_pose(incoming_pose, latched_pose)
    return latched_pose


def set_zero_twist(msg) -> None:
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0


def set_zero_accel(msg) -> None:
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
