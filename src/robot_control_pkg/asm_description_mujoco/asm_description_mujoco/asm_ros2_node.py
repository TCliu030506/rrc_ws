import ast
import os
import threading
import time
import traceback
from queue import Empty, Full, Queue
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Type

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
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from asm_description_mujoco.command_validation import CommandValidator
from asm_description_mujoco.ik_solver import DampedLeastSquaresIkSolver
from asm_description_mujoco.qos_profiles import make_control_qos
from asm_description_mujoco.sim_types import (
  CameraPublisher,
  CommandState,
  RenderedFrame,
  SensorPublisher,
  SimState,
)


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
    self.declare_parameter("sensor_names", "[]")
    self.declare_parameter("sensor_topic_prefix", "sensors")
    self.declare_parameter("end_effector_site_name", "asm_ee_site")
    self.declare_parameter("end_effector_pose_topic", "end_effector_pose")
    self.declare_parameter("cartesian_command_topic", "cartesian_target_pose")
    self.declare_parameter("publish_rgb", True)
    self.declare_parameter("publish_depth", False)
    self.declare_parameter("image_width", 640)
    self.declare_parameter("image_height", 480)
    self.declare_parameter("camera_name", "camera_fixed")
    self.declare_parameter("camera_names", "[]")
    self.declare_parameter("depth_near", 0.1)
    self.declare_parameter("depth_far", 50.0)
    self.declare_parameter("depth_max_m", 0.0)
    self.declare_parameter("image_packer_threads", 2)
    self.declare_parameter("render_queue_size", 2)
    self.declare_parameter("pack_queue_size", 4)
    float_descriptor = ParameterDescriptor(dynamic_typing=True)
    self.declare_parameter("step_rate_hz", 0.0, float_descriptor)
    self.declare_parameter("publish_rate_hz", 100.0, float_descriptor)
    self.declare_parameter("render_rate_hz", 15.0, float_descriptor)
    self.declare_parameter("viewer_rate_hz", 30.0, float_descriptor)
    self.declare_parameter("render_only_with_subscribers", True)
    self.declare_parameter("control_mode", "position")
    self.declare_parameter("raw_command_topic", "command_raw")
    self.declare_parameter("joint_command_topic", "command_joint")
    self.declare_parameter("joint_state_topic", "joint_states")
    self.declare_parameter("control_qos_depth", 10)
    self.declare_parameter("enable_command_clipping", True)
    self.declare_parameter("allow_partial_raw_commands", False)
    self.declare_parameter("command_timeout_sec", 0.0, float_descriptor)
    self.declare_parameter("world_frame", "world")
    self.declare_parameter("ik_max_iters", 50)
    self.declare_parameter("ik_position_tolerance", 1e-4, float_descriptor)
    self.declare_parameter("ik_orientation_tolerance", 1e-3, float_descriptor)
    self.declare_parameter("ik_damping", 1e-2, float_descriptor)
    self.declare_parameter("ik_step_size", 0.5, float_descriptor)
    self.declare_parameter("ik_max_position_step", 0.05, float_descriptor)
    self.declare_parameter("ik_max_orientation_step", 0.25, float_descriptor)
    self.declare_parameter("external_wrench_topic", "/wrench_compensated")
    self.declare_parameter("external_wrench_frame_id", "asm_force_sensor_link")
    self.declare_parameter("external_force_sensor_name", "asm_force_sensor_force")
    self.declare_parameter("external_torque_sensor_name", "asm_force_sensor_torque")

    model_path = str(self.get_parameter("model_path").value)
    self.get_logger().info(f"Loading MuJoCo model: {model_path}")
    self.model = mujoco.MjModel.from_xml_path(model_path)
    self.data = mujoco.MjData(self.model)
    self.viewer_data = mujoco.MjData(self.model)
    self.sim_dt = float(self.model.opt.timestep)

    self._stop_event = threading.Event()
    self._threads: List[threading.Thread] = []
    self._command_lock = threading.Lock()
    self._state_lock = threading.Lock()
    self._state_snapshot: Optional[SimState] = None
    self._latest_command = CommandState(stamp=None)

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
    self.camera_names = self._normalize_string_list(
      self.get_parameter("camera_names").value
    )
    self.depth_near = float(self.get_parameter("depth_near").value)
    self.depth_far = float(self.get_parameter("depth_far").value)
    self.depth_max_m = float(self.get_parameter("depth_max_m").value)
    self.image_packer_threads = int(
      self.get_parameter("image_packer_threads").value
    )
    self.render_queue_size = int(self.get_parameter("render_queue_size").value)
    self.pack_queue_size = int(self.get_parameter("pack_queue_size").value)
    self.step_rate_hz = float(self.get_parameter("step_rate_hz").value)
    self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
    self.render_rate_hz = float(self.get_parameter("render_rate_hz").value)
    self.viewer_rate_hz = float(self.get_parameter("viewer_rate_hz").value)
    self.render_only_with_subscribers = bool(
      self.get_parameter("render_only_with_subscribers").value
    )
    self.control_mode = str(self.get_parameter("control_mode").value)
    self.control_qos_depth = int(self.get_parameter("control_qos_depth").value)
    self.enable_command_clipping = bool(
      self.get_parameter("enable_command_clipping").value
    )
    self.allow_partial_raw_commands = bool(
      self.get_parameter("allow_partial_raw_commands").value
    )
    self.command_timeout_sec = float(
      self.get_parameter("command_timeout_sec").value
    )
    self.world_frame = str(self.get_parameter("world_frame").value)
    self.ik_max_iters = int(self.get_parameter("ik_max_iters").value)
    self.ik_position_tolerance = float(
      self.get_parameter("ik_position_tolerance").value
    )
    self.ik_orientation_tolerance = float(
      self.get_parameter("ik_orientation_tolerance").value
    )
    self.ik_damping = float(self.get_parameter("ik_damping").value)
    self.ik_step_size = float(self.get_parameter("ik_step_size").value)
    self.ik_max_position_step = float(
      self.get_parameter("ik_max_position_step").value
    )
    self.ik_max_orientation_step = float(
      self.get_parameter("ik_max_orientation_step").value
    )
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
        self.viewer = mujoco.viewer.launch_passive(self.model, self.viewer_data)
        self.get_logger().info("MuJoCo viewer started.")
      except Exception as exc:
        self.get_logger().error(f"Failed to start viewer: {exc}")
        self.viewer = None

    self.glfw_initialized = False
    self.window = None
    self.scene = None
    self.render_context = None
    self.camera_publishers: List[CameraPublisher] = []
    self.render_data: Optional[mujoco.MjData] = None
    self._render_queue: Optional[Queue] = None
    self._pack_queue: Optional[Queue] = None
    self._camera_pub_by_name: Dict[str, CameraPublisher] = {}
    if self.publish_rgb or self.publish_depth:
      self._init_renderer()
      self.camera_publishers = self._build_camera_publishers()
      self.render_data = mujoco.MjData(self.model)
      self._render_queue = Queue(maxsize=max(1, self.render_queue_size))
      self._pack_queue = Queue(maxsize=max(1, self.pack_queue_size))
      self._camera_pub_by_name = {
        publisher.name: publisher for publisher in self.camera_publishers
      }
      try:
        glfw.make_context_current(None)
      except Exception:
        pass

    self.joint_names, self.joint_qpos_adr, self.joint_qvel_adr = (
      self._build_joint_state_cache()
    )
    self.actuator_count = int(self.model.nu)
    self.controlled_joint_names, self.controlled_joint_ids, self.controlled_joint_qpos_adr, self.controlled_joint_qvel_adr = (
      self._build_controlled_joint_cache()
    )
    self.joint_name_to_ctrl_index = self._build_joint_control_map()
    self.last_ctrl = np.zeros(self.actuator_count, dtype=np.float64)

    self.end_effector_site_id = mujoco.mj_name2id(
      self.model, mujoco.mjtObj.mjOBJ_SITE, self.end_effector_site_name
    )
    if self.end_effector_site_id == -1:
      self.get_logger().warn(
        f"End-effector site '{self.end_effector_site_name}' not found; FK/IK disabled."
      )

    self.command_validator = CommandValidator(
      self.actuator_count,
      np.asarray(self.model.actuator_ctrllimited, dtype=bool),
      np.asarray(self.model.actuator_ctrlrange, dtype=np.float64),
      self.joint_name_to_ctrl_index,
      self.get_logger(),
      enable_clipping=self.enable_command_clipping,
      allow_partial_raw_commands=self.allow_partial_raw_commands,
      world_frame=self.world_frame,
    )
    self.ik_solver = DampedLeastSquaresIkSolver(
      self.model,
      self.end_effector_site_id,
      self.controlled_joint_ids,
      self.controlled_joint_qpos_adr,
      self.controlled_joint_qvel_adr,
      max_iters=self.ik_max_iters,
      position_tolerance=self.ik_position_tolerance,
      orientation_tolerance=self.ik_orientation_tolerance,
      damping=self.ik_damping,
      step_size=self.ik_step_size,
      max_position_step=self.ik_max_position_step,
      max_orientation_step=self.ik_max_orientation_step,
    )
    self._ik_warning_active = False
    self._command_timeout_warning_active = False

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

    control_qos = make_control_qos(self.control_qos_depth)

    raw_command_topic = str(self.get_parameter("raw_command_topic").value)
    self.create_subscription(
      Float64MultiArray,
      raw_command_topic,
      self._on_raw_command,
      control_qos,
    )

    joint_command_topic = str(self.get_parameter("joint_command_topic").value)
    self.create_subscription(
      JointState,
      joint_command_topic,
      self._on_joint_command,
      control_qos,
    )

    self.create_subscription(
      PoseStamped,
      self.cartesian_command_topic,
      self._on_cartesian_command,
      control_qos,
    )

    self.step_period = self._compute_step_period()
    if self.publish_rate_hz <= 0.0:
      self.publish_period = 0.0
      self._publish_enabled = False
    else:
      self.publish_period = 1.0 / self.publish_rate_hz
      self._publish_enabled = True
    if self.render_rate_hz > 0.0:
      self.render_period = 1.0 / self.render_rate_hz
    else:
      self.render_period = 1.0 / 15.0
    self.viewer_period = 1.0 / max(1.0, self.viewer_rate_hz)
    self._write_state_snapshot()

    self._start_threads()

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

  def _start_threads(self) -> None:
    self._stop_event.clear()

    self._simulation_thread = self._start_worker_thread(
      "mujoco_sim_thread", self._simulation_loop
    )

    if self.viewer is not None:
      self._viewer_thread = self._start_worker_thread(
        "mujoco_viewer_thread", self._viewer_loop
      )

    if self._publish_enabled:
      self._publish_thread = self._start_worker_thread(
        "ros2_publish_thread", self._publish_loop
      )

    if (self.publish_rgb or self.publish_depth) and self._publish_enabled:
      self._render_thread = self._start_worker_thread(
        "camera_render_thread", self._render_loop
      )

      packer_count = max(1, int(self.image_packer_threads))
      self._packer_threads: List[threading.Thread] = []
      for idx in range(packer_count):
        thread = self._start_worker_thread(
          f"image_pack_thread_{idx}", self._image_pack_loop
        )
        self._packer_threads.append(thread)

  def _start_worker_thread(
    self, name: str, target: Callable[[], None]
  ) -> threading.Thread:
    thread = threading.Thread(
      target=self._worker_thread_main,
      args=(name, target),
      name=name,
      daemon=True,
    )
    thread.start()
    self._threads.append(thread)
    return thread

  def _worker_thread_main(self, name: str, target: Callable[[], None]) -> None:
    try:
      target()
    except Exception as exc:
      self.get_logger().error(
        f"Worker thread '{name}' crashed: {exc}\n{traceback.format_exc()}"
      )
      self._stop_event.set()

  def _stop_threads(self) -> None:
    self._stop_event.set()
    current_thread = threading.current_thread()
    for thread in list(self._threads):
      if thread is not current_thread and thread.is_alive():
        thread.join(timeout=1.0)

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

  def _build_controlled_joint_cache(
    self,
  ) -> Tuple[List[str], List[int], List[int], List[int]]:
    joint_names: List[str] = []
    joint_ids: List[int] = []
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
      joint_ids.append(joint_id)
      qpos_adrs.append(int(self.model.jnt_qposadr[joint_id]))
      qvel_adrs.append(int(self.model.jnt_dofadr[joint_id]))
    return joint_names, joint_ids, qpos_adrs, qvel_adrs

  def _get_latest_state(self) -> Optional[SimState]:
    with self._state_lock:
      return self._state_snapshot

  def _write_state_snapshot(self) -> None:
    ext_force = self._get_sensor_data_from_array(
      self.external_force_sensor_id, self.data.sensordata
    )
    ext_torque = self._get_sensor_data_from_array(
      self.external_torque_sensor_id, self.data.sensordata
    )
    state = SimState(
      stamp=self.get_clock().now().to_msg(),
      sim_time=float(self.data.time),
      qpos=np.array(self.data.qpos, copy=True),
      qvel=np.array(self.data.qvel, copy=True),
      sensordata=np.array(self.data.sensordata, copy=True),
      site_xpos=np.array(self.data.site_xpos, copy=True),
      site_xmat=np.array(self.data.site_xmat, copy=True),
      ext_force=None if ext_force is None else np.array(ext_force, copy=True),
      ext_torque=None if ext_torque is None else np.array(ext_torque, copy=True),
    )
    with self._state_lock:
      self._state_snapshot = state

  def _get_latest_command(self) -> CommandState:
    with self._command_lock:
      return self._latest_command

  def _set_latest_command(self, command: CommandState) -> None:
    with self._command_lock:
      self._latest_command = command

  def _copy_state_to_data(self, state: SimState, data: mujoco.MjData) -> None:
    data.qpos[:] = state.qpos
    data.qvel[:] = state.qvel
    mujoco.mj_forward(self.model, data)

  def _qpos_solution_to_ctrl(self, q_solution: np.ndarray) -> np.ndarray:
    ctrl = np.array(self.last_ctrl, copy=True)
    for ctrl_index, qpos_adr in enumerate(self.controlled_joint_qpos_adr):
      if ctrl_index >= ctrl.size or qpos_adr >= q_solution.size:
        continue
      ctrl[ctrl_index] = float(q_solution[qpos_adr])
    return ctrl

  def _simulation_loop(self) -> None:
    next_time = time.perf_counter()
    while not self._stop_event.is_set():
      self._step_simulation_once()
      next_time += self.step_period
      sleep_time = next_time - time.perf_counter()
      if sleep_time > 0.0:
        time.sleep(sleep_time)
      else:
        next_time = time.perf_counter()

  def _viewer_loop(self) -> None:
    while not self._stop_event.is_set():
      if self.viewer is None:
        return
      if not self.viewer.is_running():
        self.get_logger().info("Viewer closed; disabling viewer sync.")
        self.viewer = None
        return
      state = self._get_latest_state()
      if state is not None:
        with self.viewer.lock():
          self._copy_state_to_data(state, self.viewer_data)
          self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(
            state.sim_time % 2
          )
      self.viewer.sync()
      time.sleep(self.viewer_period)

  def _render_loop(self) -> None:
    if self.window is None or self.render_data is None:
      return
    try:
      glfw.make_context_current(self.window)
    except Exception as exc:
      self.get_logger().error(f"Failed to bind GL context in render thread: {exc}")
      return

    next_time = time.perf_counter()
    while not self._stop_event.is_set():
      if self._render_queue is None:
        break
      if self._render_queue.full():
        time.sleep(self.render_period)
        continue

      if self.render_only_with_subscribers and not self._has_camera_subscribers():
        time.sleep(self.render_period)
        continue

      state = self._get_latest_state()
      if state is None:
        time.sleep(self.render_period)
        continue
      self._copy_state_to_data(state, self.render_data)

      for camera_pub in self.camera_publishers:
        if camera_pub.rgb_pub is None and camera_pub.depth_pub is None:
          continue
        if self.render_only_with_subscribers and not self._camera_has_subscribers(camera_pub):
          continue
        rgb, depth = self._render_images_raw(
          camera_pub.camera,
          self.image_width,
          self.image_height,
          self.render_data,
        )
        frame = RenderedFrame(
          camera_name=camera_pub.name,
          stamp=state.stamp,
          rgb=rgb if camera_pub.rgb_pub is not None else None,
          depth=depth if camera_pub.depth_pub is not None else None,
        )
        try:
          self._render_queue.put_nowait(frame)
        except Full:
          pass

      next_time += self.render_period
      sleep_time = next_time - time.perf_counter()
      if sleep_time > 0.0:
        time.sleep(sleep_time)
      else:
        next_time = time.perf_counter()

    try:
      glfw.make_context_current(None)
    except Exception:
      pass

  def _image_pack_loop(self) -> None:
    if self._render_queue is None or self._pack_queue is None:
      return
    while not self._stop_event.is_set():
      try:
        frame: RenderedFrame = self._render_queue.get(timeout=0.1)
      except Empty:
        continue

      camera_pub = self._camera_pub_by_name.get(frame.camera_name)
      if camera_pub is None:
        continue

      rgb_msg = None
      if camera_pub.rgb_pub is not None and frame.rgb is not None:
        rgb_msg = Image()
        rgb_msg.header.stamp = frame.stamp
        rgb_msg.height = frame.rgb.shape[0]
        rgb_msg.width = frame.rgb.shape[1]
        rgb_msg.encoding = "rgb8"
        rgb_msg.step = frame.rgb.shape[1] * 3
        rgb_msg.data = frame.rgb.tobytes()

      depth_msg = None
      if camera_pub.depth_pub is not None and frame.depth is not None:
        linear_depth = self._linearize_depth(frame.depth)
        if self.depth_max_m > 0.0:
          linear_depth = np.clip(linear_depth, 0.0, self.depth_max_m)
        depth_msg = Image()
        depth_msg.header.stamp = frame.stamp
        depth_msg.height = linear_depth.shape[0]
        depth_msg.width = linear_depth.shape[1]
        depth_msg.encoding = "32FC1"
        depth_msg.step = linear_depth.shape[1] * 4
        depth_msg.data = linear_depth.astype(np.float32).tobytes()

      if rgb_msg is None and depth_msg is None:
        continue
      try:
        self._pack_queue.put_nowait((camera_pub, rgb_msg, depth_msg))
      except Full:
        pass

  def _camera_has_subscribers(self, camera_pub: CameraPublisher) -> bool:
    if camera_pub.rgb_pub is not None and camera_pub.rgb_pub.get_subscription_count() > 0:
      return True
    if camera_pub.depth_pub is not None and camera_pub.depth_pub.get_subscription_count() > 0:
      return True
    return False

  def _has_camera_subscribers(self) -> bool:
    for camera_pub in self.camera_publishers:
      if self._camera_has_subscribers(camera_pub):
        return True
    return False

  def _publish_loop(self) -> None:
    next_time = time.perf_counter()
    while not self._stop_event.is_set():
      if self._publish_enabled:
        self._publish_outputs_once()
        self._publish_camera_frames()
      next_time += self.publish_period
      sleep_time = next_time - time.perf_counter()
      if sleep_time > 0.0:
        time.sleep(sleep_time)
      else:
        next_time = time.perf_counter()

  def _publish_camera_frames(self) -> None:
    if self._pack_queue is None:
      return
    while True:
      try:
        camera_pub, rgb_msg, depth_msg = self._pack_queue.get_nowait()
      except Empty:
        break
      if rgb_msg is not None and camera_pub.rgb_pub is not None:
        camera_pub.rgb_pub.publish(rgb_msg)
      if depth_msg is not None and camera_pub.depth_pub is not None:
        camera_pub.depth_pub.publish(depth_msg)

  def _on_cartesian_command(self, msg: PoseStamped) -> None:
    target = self.command_validator.validate_cartesian(msg)
    if target is None:
      return
    self._set_latest_command(
      CommandState(
        stamp=target.header.stamp,
        received_time=time.perf_counter(),
        cartesian_target=target,
      )
    )
    self._command_timeout_warning_active = False

  def _get_end_effector_pose(self, state: SimState) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "world"

    if self.end_effector_site_id == -1:
      return pose

    position = state.site_xpos[self.end_effector_site_id]
    orientation = self._mat_to_quat(state.site_xmat[self.end_effector_site_id])

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
    self.scene = mujoco.MjvScene(self.model, maxgeom=1000)
    self.render_context = mujoco.MjrContext(
      self.model, mujoco.mjtFontScale.mjFONTSCALE_150
    )
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.render_context)

  def _normalize_string_list(self, value: object) -> List[str]:
    if isinstance(value, (list, tuple)):
      return [str(item) for item in value if str(item)]
    if value is None:
      return []
    if isinstance(value, str):
      text = value.strip()
      if not text:
        return []
      if text.startswith("[") or text.startswith("("):
        try:
          parsed = ast.literal_eval(text)
        except (ValueError, SyntaxError):
          parsed = None
        if isinstance(parsed, (list, tuple)):
          return [str(item) for item in parsed if str(item)]
      if "," in text:
        return [part.strip().strip("'\"") for part in text.split(",") if part.strip()]
      return [text]
    return [str(value)]

  def _build_camera_publishers(self) -> List[CameraPublisher]:
    camera_names = self.camera_names or [self.camera_name]
    multi_camera_topics = bool(self.camera_names)

    publishers: List[CameraPublisher] = []
    for camera_name in camera_names:
      camera = mujoco.MjvCamera()
      cam_id = mujoco.mj_name2id(
        self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
      )
      if cam_id == -1:
        self.get_logger().warn(
          f"Camera '{camera_name}' not found; using free camera for this target."
        )
      else:
        camera.fixedcamid = cam_id
        camera.type = mujoco.mjtCamera.mjCAMERA_FIXED

      if multi_camera_topics:
        rgb_topic = f"camera/{camera_name}/rgb/image_raw"
        depth_topic = f"camera/{camera_name}/depth/image_raw"
      else:
        rgb_topic = "camera/rgb/image_raw"
        depth_topic = "camera/depth/image_raw"

      rgb_pub = None
      depth_pub = None
      if self.publish_rgb:
        rgb_pub = self.create_publisher(
          Image, rgb_topic, qos_profile_sensor_data
        )
      if self.publish_depth:
        depth_pub = self.create_publisher(
          Image, depth_topic, qos_profile_sensor_data
        )

      publishers.append(
        CameraPublisher(
          name=camera_name,
          camera=camera,
          rgb_pub=rgb_pub,
          depth_pub=depth_pub,
        )
      )

    if not publishers:
      self.get_logger().warn("No camera targets were created.")
    return publishers

  def _build_sensor_publishers(self) -> List[SensorPublisher]:
    sensor_names_param = self.get_parameter("sensor_names").value
    names = self._normalize_string_list(sensor_names_param)
    if not names:
      names = [
        mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        for i in range(self.model.nsensor)
      ]
      names = [name for name in names if name]
    else:
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

  def _get_sensor_data_from_array(
    self, sensor_id: int, sensordata: np.ndarray
  ) -> Optional[np.ndarray]:
    if sensor_id == -1:
      return None
    start_idx = int(self.model.sensor_adr[sensor_id])
    dim = int(self.model.sensor_dim[sensor_id])
    return sensordata[start_idx : start_idx + dim]

  def _on_raw_command(self, msg: Float64MultiArray) -> None:
    ctrl = self.command_validator.validate_raw(msg.data)
    if ctrl is None:
      return
    self._set_latest_command(
      CommandState(
        stamp=self.get_clock().now().to_msg(),
        received_time=time.perf_counter(),
        raw_ctrl=ctrl,
      )
    )
    self._command_timeout_warning_active = False

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

    joint_target = self.command_validator.validate_joint(msg.name, values)
    if joint_target is None:
      return
    self._set_latest_command(
      CommandState(
        stamp=msg.header.stamp,
        received_time=time.perf_counter(),
        joint_target=joint_target,
      )
    )
    self._command_timeout_warning_active = False

  def _command_is_stale(self, command: CommandState) -> bool:
    if self.command_timeout_sec <= 0.0 or command.received_time <= 0.0:
      return False
    return (time.perf_counter() - command.received_time) > self.command_timeout_sec

  def _step_simulation_once(self) -> None:
    command = self._get_latest_command()
    if self._command_is_stale(command):
      if not self._command_timeout_warning_active:
        self.get_logger().warn(
          f"No fresh command received for {self.command_timeout_sec:.3f}s; holding last control target."
        )
        self._command_timeout_warning_active = True
      command = CommandState(stamp=None)

    if command.cartesian_target is not None:
      target = command.cartesian_target
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
      ik_result = self.ik_solver.solve(
        target_position, target_quaternion, self.data.qpos
      )
      if ik_result is not None and self.actuator_count:
        if not ik_result.converged and not self._ik_warning_active:
          self.get_logger().warn(
            "IK did not fully converge; using best bounded solution "
            f"(pos_err={ik_result.position_error_norm:.5f}, "
            f"ori_err={ik_result.orientation_error_norm:.5f}, "
            f"iters={ik_result.iterations})."
          )
          self._ik_warning_active = True
        elif ik_result.converged:
          self._ik_warning_active = False
        self.last_ctrl = self.command_validator.clip_full_ctrl(
          self._qpos_solution_to_ctrl(ik_result.qpos)
        )
    elif command.raw_ctrl is not None:
      self.last_ctrl = np.array(command.raw_ctrl, copy=True)
    elif command.joint_target is not None:
      valid = np.isfinite(command.joint_target)
      self.last_ctrl[valid] = command.joint_target[valid]

    if self.actuator_count:
      self.data.ctrl[:] = self.last_ctrl

    steps = max(1, int(round(self.step_period / self.sim_dt)))
    for _ in range(steps):
      mujoco.mj_step(self.model, self.data)
    self._write_state_snapshot()

  def _publish_outputs_once(self) -> None:
    state = self._get_latest_state()
    if state is None:
      return
    now = state.stamp

    pose_msg = None
    joint_msg = None
    sensor_messages: List[Tuple[object, object]] = []
    wrench_msg = None

    if self.end_effector_pose_pub is not None:
      pose_msg = self._get_end_effector_pose(state)
      pose_msg.header.stamp = now

    if self.joint_state_pub is not None:
      joint_msg = JointState()
      joint_msg.header.stamp = now
      joint_msg.name = list(self.joint_names)
      joint_msg.position = [float(state.qpos[i]) for i in self.joint_qpos_adr]
      joint_msg.velocity = [float(state.qvel[i]) for i in self.joint_qvel_adr]

    for sensor_pub in self.sensor_publishers:
      sensor_values = self._get_sensor_data_from_array(
        sensor_pub.sensor_id, state.sensordata
      )
      if sensor_values is None:
        continue
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
      sensor_messages.append((sensor_pub.publisher, msg))

    if state.ext_force is not None and state.ext_torque is not None:
      force = state.ext_force
      torque = state.ext_torque
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

    if pose_msg is not None:
      self.end_effector_pose_pub.publish(pose_msg)
    if joint_msg is not None:
      self.joint_state_pub.publish(joint_msg)
    for publisher, msg in sensor_messages:
      publisher.publish(msg)
    if wrench_msg is not None:
      self.external_wrench_pub.publish(wrench_msg)

  def _render_images_raw(
    self,
    camera: mujoco.MjvCamera,
    width: int,
    height: int,
    data: mujoco.MjData,
  ) -> Tuple[np.ndarray, np.ndarray]:
    viewport = mujoco.MjrRect(0, 0, width, height)
    mujoco.mjv_updateScene(
      self.model,
      data,
      mujoco.MjvOption(),
      None,
      camera,
      mujoco.mjtCatBit.mjCAT_ALL,
      self.scene,
    )
    mujoco.mjr_render(viewport, self.scene, self.render_context)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.zeros((height, width), dtype=np.float64)
    mujoco.mjr_readPixels(rgb, depth, viewport, self.render_context)
    rgb = np.flipud(rgb)
    depth = np.flipud(depth)

    return rgb, depth

  def _linearize_depth(self, depth: np.ndarray) -> np.ndarray:
    near = self.depth_near
    far = self.depth_far
    denom = far - (far - near) * depth
    return (far * near) / np.maximum(denom, 1e-6)

  def destroy_node(self) -> bool:
    self._stop_threads()
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
