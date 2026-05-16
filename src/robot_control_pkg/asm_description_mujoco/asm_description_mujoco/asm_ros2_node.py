import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Type

import glfw
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
  PointStamped,
  PoseStamped,
  QuaternionStamped,
  Vector3Stamped,
  WrenchStamped,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray


@dataclass
class SensorPublisher:
  name: str
  sensor_id: int
  dim: int
  msg_type: Type
  publisher: object


class MujocoRos2Node(Node):
  def __init__(self) -> None:
    super().__init__("asm_mujoco_node")

    default_model_path = os.path.join(
      get_package_share_directory("asm_description_mujoco"),
      "ur5e_with_asm",
      "scene_all.xml",
    )

    self.declare_parameter("model_path", default_model_path)
    self.declare_parameter("keyframe_name", "home")
    self.declare_parameter("use_viewer", True)
    self.declare_parameter("publish_joint_states", True)
    self.declare_parameter("publish_end_effector_pose", True)
    self.declare_parameter("publish_raw_sensors", True)
    self.declare_parameter("sensor_names", [])
    self.declare_parameter("sensor_topic_prefix", "sensors")
    self.declare_parameter("end_effector_site_name", "asm_ee_site")
    self.declare_parameter("end_effector_pose_topic", "end_effector_pose")
    self.declare_parameter("cartesian_command_topic", "cartesian_target_pose")
    self.declare_parameter("publish_rgb", True)
    self.declare_parameter("publish_depth", False)
    self.declare_parameter("image_width", 640)
    self.declare_parameter("image_height", 480)
    self.declare_parameter("camera_name", "camera_fixed")
    self.declare_parameter("depth_near", 0.1)
    self.declare_parameter("depth_far", 50.0)
    self.declare_parameter("depth_max_m", 0.0)
    self.declare_parameter("step_rate_hz", 0.0)
    self.declare_parameter("publish_rate_hz", 50.0)
    self.declare_parameter("control_mode", "position")
    self.declare_parameter("raw_command_topic", "command_raw")
    self.declare_parameter("joint_command_topic", "command_joint")
    self.declare_parameter("joint_state_topic", "joint_states")
    self.declare_parameter("external_wrench_topic", "/wrench_compensated")
    self.declare_parameter("external_wrench_frame_id", "asm_force_sensor_link")
    self.declare_parameter("external_force_sensor_name", "asm_force_sensor_force")
    self.declare_parameter("external_torque_sensor_name", "asm_force_sensor_torque")

    model_path = str(self.get_parameter("model_path").value)
    self.get_logger().info(f"Loading MuJoCo model: {model_path}")
    self.model = mujoco.MjModel.from_xml_path(model_path)
    self.data = mujoco.MjData(self.model)
    self.sim_dt = float(self.model.opt.timestep)

    self.use_viewer = bool(self.get_parameter("use_viewer").value)
    self.publish_joint_states = bool(
      self.get_parameter("publish_joint_states").value
    )
    self.publish_end_effector_pose = bool(
      self.get_parameter("publish_end_effector_pose").value
    )
    self.publish_raw_sensors = bool(
      self.get_parameter("publish_raw_sensors").value
    )
    self.publish_rgb = bool(self.get_parameter("publish_rgb").value)
    self.publish_depth = bool(self.get_parameter("publish_depth").value)
    self.image_width = int(self.get_parameter("image_width").value)
    self.image_height = int(self.get_parameter("image_height").value)
    self.camera_name = str(self.get_parameter("camera_name").value)
    self.depth_near = float(self.get_parameter("depth_near").value)
    self.depth_far = float(self.get_parameter("depth_far").value)
    self.depth_max_m = float(self.get_parameter("depth_max_m").value)
    self.step_rate_hz = float(self.get_parameter("step_rate_hz").value)
    self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
    self.control_mode = str(self.get_parameter("control_mode").value)
    self.sensor_topic_prefix = str(
      self.get_parameter("sensor_topic_prefix").value
    )
    self.end_effector_site_name = str(
      self.get_parameter("end_effector_site_name").value
    )
    self.end_effector_pose_topic = str(
      self.get_parameter("end_effector_pose_topic").value
    )
    self.cartesian_command_topic = str(
      self.get_parameter("cartesian_command_topic").value
    )
    self.external_wrench_topic = str(
      self.get_parameter("external_wrench_topic").value
    )
    self.external_wrench_frame_id = str(
      self.get_parameter("external_wrench_frame_id").value
    )
    self.external_force_sensor_name = str(
      self.get_parameter("external_force_sensor_name").value
    )
    self.external_torque_sensor_name = str(
      self.get_parameter("external_torque_sensor_name").value
    )

    self.viewer = None
    if self.use_viewer:
      try:
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.get_logger().info("MuJoCo viewer started.")
      except Exception as exc:
        self.get_logger().error(f"Failed to start viewer: {exc}")
        self.viewer = None

    self.glfw_initialized = False
    self.window = None
    self.scene = None
    self.render_context = None
    self.camera = None
    if self.publish_rgb or self.publish_depth:
      self._init_renderer()

    self.joint_names, self.joint_qpos_adr, self.joint_qvel_adr = (
      self._build_joint_state_cache()
    )
    self.actuator_count = int(self.model.nu)
    self.controlled_joint_names, self.controlled_joint_qpos_adr, self.controlled_joint_qvel_adr = (
      self._build_controlled_joint_cache()
    )
    self.joint_name_to_ctrl_index = self._build_joint_control_map()
    self.last_ctrl = np.zeros(self.actuator_count, dtype=np.float64)
    self.pending_ctrl: Optional[np.ndarray] = None
    self.pending_cartesian_target: Optional[PoseStamped] = None

    self.end_effector_site_id = mujoco.mj_name2id(
      self.model, mujoco.mjtObj.mjOBJ_SITE, self.end_effector_site_name
    )
    if self.end_effector_site_id == -1:
      self.get_logger().warn(
        f"End-effector site '{self.end_effector_site_name}' not found; FK/IK disabled."
      )

    self.ik_max_iters = 20
    self.ik_tolerance = 1e-4
    self.ik_damping = 1e-3
    self.ik_step_size = 0.5

    keyframe_name = str(self.get_parameter("keyframe_name").value)
    self._load_keyframe(keyframe_name)

    self.sensor_publishers: List[SensorPublisher] = []
    if self.publish_raw_sensors:
      self.sensor_publishers = self._build_sensor_publishers()

    if self.publish_joint_states:
      joint_state_topic = str(self.get_parameter("joint_state_topic").value)
      self.joint_state_pub = self.create_publisher(
        JointState, joint_state_topic, qos_profile_sensor_data
      )
    else:
      self.joint_state_pub = None

    if self.publish_end_effector_pose:
      self.end_effector_pose_pub = self.create_publisher(
        PoseStamped, self.end_effector_pose_topic, qos_profile_sensor_data
      )
    else:
      self.end_effector_pose_pub = None

    if self.publish_rgb:
      self.rgb_pub = self.create_publisher(
        Image, "camera/rgb/image_raw", qos_profile_sensor_data
      )
    else:
      self.rgb_pub = None

    if self.publish_depth:
      self.depth_pub = self.create_publisher(
        Image, "camera/depth/image_raw", qos_profile_sensor_data
      )
    else:
      self.depth_pub = None

    # Publish external wrench in the exact message shape expected by admittance control.
    self.external_wrench_pub = self.create_publisher(
      WrenchStamped, self.external_wrench_topic, 10
    )
    self.external_force_sensor_id = mujoco.mj_name2id(
      self.model,
      mujoco.mjtObj.mjOBJ_SENSOR,
      self.external_force_sensor_name,
    )
    self.external_torque_sensor_id = mujoco.mj_name2id(
      self.model,
      mujoco.mjtObj.mjOBJ_SENSOR,
      self.external_torque_sensor_name,
    )
    if self.external_force_sensor_id == -1:
      self.get_logger().warn(
        f"External force sensor '{self.external_force_sensor_name}' not found."
      )
    if self.external_torque_sensor_id == -1:
      self.get_logger().warn(
        f"External torque sensor '{self.external_torque_sensor_name}' not found."
      )

    raw_command_topic = str(self.get_parameter("raw_command_topic").value)
    self.create_subscription(
      Float64MultiArray,
      raw_command_topic,
      self._on_raw_command,
      qos_profile_sensor_data,
    )

    joint_command_topic = str(self.get_parameter("joint_command_topic").value)
    self.create_subscription(
      JointState,
      joint_command_topic,
      self._on_joint_command,
      qos_profile_sensor_data,
    )

    self.create_subscription(
      PoseStamped,
      self.cartesian_command_topic,
      self._on_cartesian_command,
      qos_profile_sensor_data,
    )

    self.step_period = self._compute_step_period()
    self.step_timer = self.create_timer(self.step_period, self._step_simulation)

    if self.publish_rate_hz <= 0.0:
      self.publish_period = 0.0
    else:
      self.publish_period = 1.0 / self.publish_rate_hz
    self.publish_timer = self.create_timer(
      self.publish_period, self._publish_outputs
    )

    self.get_logger().info(
      f"Sim dt: {self.sim_dt:.4f}s, step period: {self.step_period:.4f}s"
    )

  def _compute_step_period(self) -> float:
    if self.step_rate_hz <= 0.0:
      return self.sim_dt
    requested = 1.0 / self.step_rate_hz
    if requested < self.sim_dt:
      self.get_logger().warn(
        "Requested step_rate_hz exceeds sim dt; clamping to sim dt."
      )
      return self.sim_dt
    return requested

  def _build_joint_state_cache(self) -> Tuple[List[str], List[int], List[int]]:
    joint_names: List[str] = []
    qpos_adrs: List[int] = []
    qvel_adrs: List[int] = []
    for j in range(self.model.njnt):
      j_type = self.model.jnt_type[j]
      if j_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
        continue
      name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
      if not name:
        continue
      joint_names.append(name)
      qpos_adrs.append(int(self.model.jnt_qposadr[j]))
      qvel_adrs.append(int(self.model.jnt_dofadr[j]))
    return joint_names, qpos_adrs, qvel_adrs

  def _build_joint_control_map(self) -> Dict[str, int]:
    mapping: Dict[str, int] = {}
    for i in range(self.model.nu):
      joint_id = int(self.model.actuator_trnid[i][0])
      if joint_id < 0:
        continue
      joint_name = mujoco.mj_id2name(
        self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
      )
      if joint_name:
        mapping[joint_name] = i
    return mapping

  def _build_controlled_joint_cache(self) -> Tuple[List[str], List[int], List[int]]:
    joint_names: List[str] = []
    qpos_adrs: List[int] = []
    qvel_adrs: List[int] = []
    for i in range(self.model.nu):
      joint_id = int(self.model.actuator_trnid[i][0])
      if joint_id < 0:
        continue
      joint_name = mujoco.mj_id2name(
        self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
      )
      if not joint_name:
        continue
      joint_names.append(joint_name)
      qpos_adrs.append(int(self.model.jnt_qposadr[joint_id]))
      qvel_adrs.append(int(self.model.jnt_dofadr[joint_id]))
    return joint_names, qpos_adrs, qvel_adrs

  def _on_cartesian_command(self, msg: PoseStamped) -> None:
    self.pending_cartesian_target = msg

  def _get_end_effector_pose(self) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.header.frame_id = "world"

    if self.end_effector_site_id == -1:
      return pose

    site = self.data.site(self.end_effector_site_id)
    position = np.asarray(site.xpos, dtype=np.float64)
    orientation = self._mat_to_quat(np.asarray(site.xmat, dtype=np.float64))

    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])
    pose.pose.orientation.w = float(orientation[0])
    pose.pose.orientation.x = float(orientation[1])
    pose.pose.orientation.y = float(orientation[2])
    pose.pose.orientation.z = float(orientation[3])
    return pose

  def _mat_to_quat(self, mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.asarray(mat, dtype=np.float64))
    return quat

  def _quat_multiply(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
    result = np.zeros(4, dtype=np.float64)
    mujoco.mju_mulQuat(
      result,
      np.asarray(left, dtype=np.float64),
      np.asarray(right, dtype=np.float64),
    )
    return result

  def _quat_error(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
    target = np.asarray(target, dtype=np.float64)
    current = np.asarray(current, dtype=np.float64)
    if np.dot(target, current) < 0.0:
      target = -target

    current_conj = np.array(
      [current[0], -current[1], -current[2], -current[3]], dtype=np.float64
    )
    quat_error = self._quat_multiply(target, current_conj)
    vector = quat_error[1:]
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm < 1e-12:
      return np.zeros(3, dtype=np.float64)
    scalar = float(np.clip(quat_error[0], -1.0, 1.0))
    angle = 2.0 * np.arctan2(vector_norm, scalar)
    return (vector / vector_norm) * angle

  def _solve_inverse_kinematics(
    self, target_position: np.ndarray, target_quaternion: np.ndarray
  ) -> Optional[np.ndarray]:
    if self.end_effector_site_id == -1:
      return None

    q = np.array(self.data.qpos, copy=True)
    target_position = np.asarray(target_position, dtype=np.float64)
    target_quaternion = np.asarray(target_quaternion, dtype=np.float64)
    control_dof_indices = np.asarray(self.controlled_joint_qvel_adr, dtype=np.int32)

    jac_pos = np.zeros((3, self.model.nv), dtype=np.float64)
    jac_rot = np.zeros((3, self.model.nv), dtype=np.float64)
    identity = np.eye(6, dtype=np.float64)

    for _ in range(self.ik_max_iters):
      self.data.qpos[:] = q
      mujoco.mj_forward(self.model, self.data)

      site = self.data.site(self.end_effector_site_id)
      current_position = np.asarray(site.xpos, dtype=np.float64)
      current_quaternion = self._mat_to_quat(np.asarray(site.xmat, dtype=np.float64))

      position_error = target_position - current_position
      orientation_error = self._quat_error(target_quaternion, current_quaternion)
      error = np.concatenate([position_error, orientation_error])

      if (
        np.linalg.norm(position_error) < self.ik_tolerance
        and np.linalg.norm(orientation_error) < self.ik_tolerance
      ):
        self.data.qpos[:] = q
        mujoco.mj_forward(self.model, self.data)
        return q

      mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, self.end_effector_site_id)
      jacobian = np.vstack((jac_pos, jac_rot))
      jacobian_reduced = jacobian[:, control_dof_indices]
      lhs = jacobian_reduced @ jacobian_reduced.T + (self.ik_damping**2) * identity
      delta_q = jacobian_reduced.T @ np.linalg.solve(lhs, error)
      q[control_dof_indices] = q[control_dof_indices] + self.ik_step_size * delta_q

    self.data.qpos[:] = q
    mujoco.mj_forward(self.model, self.data)
    return q

  def _load_keyframe(self, keyframe_name: str) -> bool:
    keyframe_id = mujoco.mj_name2id(
      self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name
    )
    if keyframe_id == -1:
      self.get_logger().warn(f"Keyframe '{keyframe_name}' not found in model.")
      return False

    keyframe = self.model.keyframe(keyframe_id)

    if len(keyframe.qpos) > 0:
      qpos_len = min(len(keyframe.qpos), self.data.qpos.size)
      self.data.qpos[:qpos_len] = keyframe.qpos[:qpos_len]
    if len(keyframe.qvel) > 0:
      qvel_len = min(len(keyframe.qvel), self.data.qvel.size)
      self.data.qvel[:qvel_len] = keyframe.qvel[:qvel_len]
    if len(keyframe.ctrl) > 0:
      ctrl_len = min(len(keyframe.ctrl), self.data.ctrl.size)
      self.data.ctrl[:ctrl_len] = keyframe.ctrl[:ctrl_len]
      self.last_ctrl = np.zeros(self.actuator_count, dtype=np.float64)
      self.last_ctrl[:ctrl_len] = np.asarray(keyframe.ctrl[:ctrl_len], dtype=np.float64)
    if len(keyframe.act) > 0 and self.data.act.size > 0:
      act_len = min(len(keyframe.act), self.data.act.size)
      self.data.act[:act_len] = keyframe.act[:act_len]

    mujoco.mj_forward(self.model, self.data)
    self.get_logger().info(f"Loaded keyframe '{keyframe_name}'.")
    return True

  def _init_renderer(self) -> None:
    if not glfw.init():
      raise RuntimeError("glfw.init() failed")
    self.glfw_initialized = True
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    self.window = glfw.create_window(
      self.image_width, self.image_height, "mujoco", None, None
    )
    if not self.window:
      raise RuntimeError("glfw.create_window() failed")
    glfw.make_context_current(self.window)

    self.camera = mujoco.MjvCamera()
    cam_id = mujoco.mj_name2id(
      self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
    )
    if cam_id == -1:
      self.get_logger().warn(
        f"Camera '{self.camera_name}' not found; using free camera."
      )
    else:
      self.camera.fixedcamid = cam_id
      self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    self.scene = mujoco.MjvScene(self.model, maxgeom=1000)
    self.render_context = mujoco.MjrContext(
      self.model, mujoco.mjtFontScale.mjFONTSCALE_150
    )
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.render_context)

  def _build_sensor_publishers(self) -> List[SensorPublisher]:
    sensor_names_param = self.get_parameter("sensor_names").value
    if sensor_names_param:
      names = list(sensor_names_param)
    else:
      names = [
        mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        for i in range(self.model.nsensor)
      ]
      names = [name for name in names if name]

    publishers: List[SensorPublisher] = []
    for name in names:
      sensor_id = mujoco.mj_name2id(
        self.model, mujoco.mjtObj.mjOBJ_SENSOR, name
      )
      if sensor_id == -1:
        self.get_logger().warn(f"Sensor '{name}' not found in model.")
        continue
      dim = int(self.model.sensor_dim[sensor_id])
      msg_type = self._infer_sensor_msg_type(name, dim)
      topic = f"{self.sensor_topic_prefix}/{name}"
      publisher = self.create_publisher(
        msg_type, topic, qos_profile_sensor_data
      )
      publishers.append(
        SensorPublisher(name=name, sensor_id=sensor_id, dim=dim, msg_type=msg_type, publisher=publisher)
      )
    return publishers

  def _infer_sensor_msg_type(self, name: str, dim: int) -> Type:
    if name.endswith("_pos") and dim == 3:
      return PointStamped
    if name.endswith("_quat") and dim == 4:
      return QuaternionStamped
    if name.endswith("_vel") and dim == 3:
      return Vector3Stamped
    if name.endswith("_force") and dim == 3:
      return Vector3Stamped
    if name.endswith("_torque") and dim == 3:
      return Vector3Stamped
    if name.endswith("_wrench") and dim == 6:
      return WrenchStamped
    return Float64MultiArray

  def _get_sensor_data(self, sensor_id: int) -> np.ndarray:
    start_idx = int(self.model.sensor_adr[sensor_id])
    dim = int(self.model.sensor_dim[sensor_id])
    return self.data.sensordata[start_idx : start_idx + dim]

  def _on_raw_command(self, msg: Float64MultiArray) -> None:
    if not msg.data:
      return
    ctrl = np.zeros(self.actuator_count, dtype=np.float64)
    length = min(len(msg.data), self.actuator_count)
    ctrl[:length] = np.asarray(msg.data[:length], dtype=np.float64)
    if len(msg.data) != self.actuator_count:
      self.get_logger().warn(
        "Raw command length mismatch; truncating or padding with zeros."
      )
    self.pending_ctrl = ctrl

  def _on_joint_command(self, msg: JointState) -> None:
    if not msg.name:
      return
    values: Sequence[float] = []
    if self.control_mode == "position":
      values = msg.position
    elif self.control_mode == "velocity":
      values = msg.velocity
    elif self.control_mode == "effort":
      values = msg.effort
    else:
      self.get_logger().warn(
        f"Unknown control_mode '{self.control_mode}', expected position|velocity|effort."
      )
      return

    if not values:
      return

    if self.control_mode == "position":
      # 位置控制模式：直接设置关节位置（响应更快）
      for name, value in zip(msg.name, values):
        # 查找关节 ID
        joint_id = mujoco.mj_name2id(
          self.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
          self.get_logger().warn(f"Joint '{name}' not found in model")
          continue
        # 获取关节位置在 qpos 数组中的索引
        qpos_adr = int(self.model.jnt_qposadr[joint_id])
        # 直接设置关节位置
        self.data.qpos[qpos_adr] = float(value)

        # 同时更新控制信号，防止执行器产生反向力矩
        ctrl_index = self.joint_name_to_ctrl_index.get(name)
        if ctrl_index is not None:
            self.data.ctrl[ctrl_index] = float(value)
            self.last_ctrl[ctrl_index] = float(value)
      
      # 更新动力学状态（必须调用，否则仿真状态不同步）
      mujoco.mj_forward(self.model, self.data)
      self.get_logger().debug(f"Directly set joint positions for: {msg.name}")
    else:
      # 速度/力控制模式：使用执行器控制
      ctrl = np.array(self.last_ctrl, copy=True)
      for name, value in zip(msg.name, values):
        ctrl_index = self.joint_name_to_ctrl_index.get(name)
        if ctrl_index is None:
          continue
        ctrl[ctrl_index] = float(value)
      self.pending_ctrl = ctrl

  def _step_simulation(self) -> None:
    if self.pending_cartesian_target is not None:
      target = self.pending_cartesian_target
      target_position = np.array(
        [
          target.pose.position.x,
          target.pose.position.y,
          target.pose.position.z,
        ],
        dtype=np.float64,
      )
      target_quaternion = np.array(
        [
          target.pose.orientation.w,
          target.pose.orientation.x,
          target.pose.orientation.y,
          target.pose.orientation.z,
        ],
        dtype=np.float64,
      )
      ik_solution = self._solve_inverse_kinematics(
        target_position, target_quaternion
      )
      if ik_solution is not None and self.actuator_count:
        self.last_ctrl = np.array(ik_solution[: self.actuator_count], copy=True)

    if self.pending_ctrl is not None:
      self.last_ctrl = self.pending_ctrl
      self.pending_ctrl = None
    if self.actuator_count:
      self.data.ctrl[:] = self.last_ctrl

    steps = max(1, int(round(self.step_period / self.sim_dt)))
    for _ in range(steps):
      mujoco.mj_step(self.model, self.data)

    if self.viewer is not None:
      if not self.viewer.is_running():
        self.get_logger().info("Viewer closed; disabling viewer sync.")
        self.viewer = None
        return
      with self.viewer.lock():
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(
          self.data.time % 2
        )
      self.viewer.sync()

  def _publish_outputs(self) -> None:
    now = self.get_clock().now().to_msg()

    if self.end_effector_pose_pub is not None:
      pose_msg = self._get_end_effector_pose()
      pose_msg.header.stamp = now
      self.end_effector_pose_pub.publish(pose_msg)

    if self.joint_state_pub is not None:
      msg = JointState()
      msg.header.stamp = now
      msg.name = list(self.joint_names)
      msg.position = [float(self.data.qpos[i]) for i in self.joint_qpos_adr]
      msg.velocity = [float(self.data.qvel[i]) for i in self.joint_qvel_adr]
      self.joint_state_pub.publish(msg)

    for sensor_pub in self.sensor_publishers:
      sensor_values = self._get_sensor_data(sensor_pub.sensor_id)
      if sensor_pub.msg_type is Float64MultiArray:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in sensor_values]
      elif sensor_pub.msg_type is PointStamped:
        msg = PointStamped()
        msg.header.stamp = now
        msg.point.x = float(sensor_values[0])
        msg.point.y = float(sensor_values[1])
        msg.point.z = float(sensor_values[2])
      elif sensor_pub.msg_type is QuaternionStamped:
        msg = QuaternionStamped()
        msg.header.stamp = now
        msg.quaternion.x = float(sensor_values[0])
        msg.quaternion.y = float(sensor_values[1])
        msg.quaternion.z = float(sensor_values[2])
        msg.quaternion.w = float(sensor_values[3])
      elif sensor_pub.msg_type is Vector3Stamped:
        msg = Vector3Stamped()
        msg.header.stamp = now
        msg.vector.x = float(sensor_values[0])
        msg.vector.y = float(sensor_values[1])
        msg.vector.z = float(sensor_values[2])
      elif sensor_pub.msg_type is WrenchStamped:
        msg = WrenchStamped()
        msg.header.stamp = now
        msg.wrench.force.x = float(sensor_values[0])
        msg.wrench.force.y = float(sensor_values[1])
        msg.wrench.force.z = float(sensor_values[2])
        msg.wrench.torque.x = float(sensor_values[3])
        msg.wrench.torque.y = float(sensor_values[4])
        msg.wrench.torque.z = float(sensor_values[5])
      else:
        continue
      sensor_pub.publisher.publish(msg)

    if self.external_force_sensor_id != -1 and self.external_torque_sensor_id != -1:
      force = self._get_sensor_data(self.external_force_sensor_id)
      torque = self._get_sensor_data(self.external_torque_sensor_id)
      if force.shape[0] >= 3 and torque.shape[0] >= 3:
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now
        wrench_msg.header.frame_id = self.external_wrench_frame_id
        wrench_msg.wrench.force.x = -1.0 * float(force[0])
        wrench_msg.wrench.force.y = -1.0 * float(force[1])
        wrench_msg.wrench.force.z = -1.0 * float(force[2])
        wrench_msg.wrench.torque.x = -1.0 * float(torque[0])
        wrench_msg.wrench.torque.y = -1.0 * float(torque[1])
        wrench_msg.wrench.torque.z = -1.0 * float(torque[2])
        self.external_wrench_pub.publish(wrench_msg)

    if (self.rgb_pub is not None) or (self.depth_pub is not None):
      rgb, depth = self._render_images(self.image_width, self.image_height)
      if self.rgb_pub is not None:
        rgb_msg = Image()
        rgb_msg.header.stamp = now
        rgb_msg.height = rgb.shape[0]
        rgb_msg.width = rgb.shape[1]
        rgb_msg.encoding = "rgb8"
        rgb_msg.step = rgb.shape[1] * 3
        rgb_msg.data = rgb.tobytes()
        self.rgb_pub.publish(rgb_msg)
      if self.depth_pub is not None:
        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.height = depth.shape[0]
        depth_msg.width = depth.shape[1]
        depth_msg.encoding = "32FC1"
        depth_msg.step = depth.shape[1] * 4
        depth_msg.data = depth.astype(np.float32).tobytes()
        self.depth_pub.publish(depth_msg)

  def _render_images(self, width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
    viewport = mujoco.MjrRect(0, 0, width, height)
    mujoco.mjv_updateScene(
      self.model,
      self.data,
      mujoco.MjvOption(),
      None,
      self.camera,
      mujoco.mjtCatBit.mjCAT_ALL,
      self.scene,
    )
    mujoco.mjr_render(viewport, self.scene, self.render_context)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.zeros((height, width), dtype=np.float64)
    mujoco.mjr_readPixels(rgb, depth, viewport, self.render_context)
    rgb = np.flipud(rgb)
    depth = np.flipud(depth)

    linear_depth = self._linearize_depth(depth)
    if self.depth_max_m > 0.0:
      linear_depth = np.clip(linear_depth, 0.0, self.depth_max_m)
    return rgb, linear_depth

  def _linearize_depth(self, depth: np.ndarray) -> np.ndarray:
    near = self.depth_near
    far = self.depth_far
    denom = far - (far - near) * depth
    return (far * near) / np.maximum(denom, 1e-6)

  def destroy_node(self) -> bool:
    if self.viewer is not None:
      try:
        self.viewer.close()
      except Exception:
        pass
    if self.window is not None:
      try:
        glfw.destroy_window(self.window)
      except Exception:
        pass
    if self.glfw_initialized:
      glfw.terminate()
    return super().destroy_node()


def main() -> None:
  rclpy.init()
  node = MujocoRos2Node()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()