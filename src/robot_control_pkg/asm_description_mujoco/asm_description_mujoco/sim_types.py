from dataclasses import dataclass
from typing import Optional, Type

import mujoco
import numpy as np
from geometry_msgs.msg import PoseStamped


@dataclass
class SensorPublisher:
    name: str
    sensor_id: int
    dim: int
    msg_type: Type
    publisher: object


@dataclass
class CameraPublisher:
    name: str
    camera: mujoco.MjvCamera
    rgb_pub: Optional[object]
    depth_pub: Optional[object]


@dataclass
class RenderedFrame:
    camera_name: str
    stamp: object
    rgb: Optional[np.ndarray]
    depth: Optional[np.ndarray]


@dataclass
class SimState:
    stamp: object
    sim_time: float
    qpos: np.ndarray
    qvel: np.ndarray
    sensordata: np.ndarray
    site_xpos: np.ndarray
    site_xmat: np.ndarray
    ext_force: Optional[np.ndarray]
    ext_torque: Optional[np.ndarray]


@dataclass
class CommandState:
    stamp: object
    received_time: float = 0.0
    raw_ctrl: Optional[np.ndarray] = None
    joint_target: Optional[np.ndarray] = None
    cartesian_target: Optional[PoseStamped] = None
