from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def make_control_qos(depth: int = 10) -> QoSProfile:
    """Reliable QoS for command/control topics."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
