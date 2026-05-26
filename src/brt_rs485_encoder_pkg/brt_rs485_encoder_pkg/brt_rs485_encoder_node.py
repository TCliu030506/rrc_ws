#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 node for publishing BRT RS485 encoder measurements."""

from __future__ import annotations

import errno
import fcntl
import glob
import math
import os
import re
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int64
from std_srvs.srv import Trigger

from brt_rs485_encoder_pkg.brt_rs485_encoder import (
    EncoderClient,
    EncoderError,
)


# 这些名字同时用于：
# 1. 标识底层 EncoderClient 的读数类型；
# 2. 校验 YAML 中 JointState 的 position/velocity 来源；
# 3. 组织每个周期真正需要轮询的寄存器集合。
POSITION_KINDS = {"position", "position2", "multiturn"}
VELOCITY_KINDS = {"speed16", "speed32"}
READ_KINDS = POSITION_KINDS | VELOCITY_KINDS | {"turns"}
NONE_SOURCE = "none"


@dataclass(frozen=True)
class ReadSpec:
    """Static configuration for one encoder readout."""

    # kind: 节点内部读数名称，必须和 READ_SPECS 的 key 保持一致。
    kind: str
    # publish_param/topic_param: YAML 中控制该读数是否发布、发布到哪里的参数名。
    publish_param: str
    topic_param: str
    default_topic: str
    # raw_topic_param 为 None 表示该读数没有额外的原始计数话题。
    raw_topic_param: Optional[str]
    default_raw_topic: Optional[str]
    # value_type 决定使用 Float64 还是 Int64 发布换算后的物理量。
    value_type: str


@dataclass
class EncoderSample:
    """Decoded encoder sample ready for ROS publication."""

    kind: str
    # raw_value 是编码器直接返回的整数，未做单位换算。
    raw_value: Optional[int]
    # position_rad / velocity_rad_s / turns 三者通常只会有一个非空。
    position_rad: Optional[float] = None
    velocity_rad_s: Optional[float] = None
    turns: Optional[int] = None


# 每个可读内容的发布参数、默认话题和消息类型集中写在这里。
# 增加新读数时优先扩展这张表，避免在参数声明和 publisher 创建处复制逻辑。
READ_SPECS = {
    "position": ReadSpec(
        kind="position",
        publish_param="publish_position",
        topic_param="position_topic",
        default_topic="encoder/position",
        raw_topic_param="position_raw_count_topic",
        default_raw_topic="encoder/position_raw_count",
        value_type="position",
    ),
    "position2": ReadSpec(
        kind="position2",
        publish_param="publish_position2",
        topic_param="position2_topic",
        default_topic="encoder/position2",
        raw_topic_param="position2_raw_count_topic",
        default_raw_topic="encoder/position2_raw_count",
        value_type="position",
    ),
    "multiturn": ReadSpec(
        kind="multiturn",
        publish_param="publish_multiturn",
        topic_param="multiturn_topic",
        default_topic="encoder/multiturn",
        raw_topic_param="multiturn_raw_count_topic",
        default_raw_topic="encoder/multiturn_raw_count",
        value_type="position",
    ),
    "turns": ReadSpec(
        kind="turns",
        publish_param="publish_turns",
        topic_param="turns_topic",
        default_topic="encoder/turns",
        raw_topic_param=None,
        default_raw_topic=None,
        value_type="turns",
    ),
    "speed16": ReadSpec(
        kind="speed16",
        publish_param="publish_speed16",
        topic_param="speed16_topic",
        default_topic="encoder/speed16",
        raw_topic_param="speed16_raw_count_topic",
        default_raw_topic="encoder/speed16_raw_count",
        value_type="velocity",
    ),
    "speed32": ReadSpec(
        kind="speed32",
        publish_param="publish_speed32",
        topic_param="speed32_topic",
        default_topic="encoder/speed32",
        raw_topic_param="speed32_raw_count_topic",
        default_raw_topic="encoder/speed32_raw_count",
        value_type="velocity",
    ),
}


class BrtRs485EncoderNode(Node):
    """Poll BRT RS485 encoder readouts and publish ROS 2 messages."""

    def __init__(self) -> None:
        super().__init__("brt_rs485_encoder_node")

        # 串口与 Modbus 通信参数。
        self.declare_parameter("port", "")
        self.declare_parameter("auto_scan", True)
        self.declare_parameter("scan_interval_sec", 5.0)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("scan_port_globs", ["/dev/ttyUSB*", "/dev/ttyACM*"])
        # 进程级串口锁：避免双节点同时扫描/打开同一个串口。
        self.declare_parameter("enable_serial_locks", True)
        self.declare_parameter("scan_lock_path", "/tmp/brt_rs485_encoder_scan.lock")
        self.declare_parameter("port_lock_dir", "/tmp/brt_rs485_encoder_port_locks")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("address", 1)
        self.declare_parameter("timeout", 0.2)

        # 轮询频率与编码器量纲换算参数。
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("resolution", 65536)
        self.declare_parameter("sample_time_ms", 100)

        # 安装方向、零点和角度偏置修正。
        self.declare_parameter("position_sign", 1.0)
        self.declare_parameter("angle_offset_rad", 0.0)
        self.declare_parameter("zero_offset_count", 0)

        # 通信异常后不会每个 timer 周期都重开串口，而是按该间隔重试。
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("verbose", False)

        # JointState 是可选的标准汇总输出，位置和速度来源可以分别指定。
        self.declare_parameter("publish_joint_state", True)
        self.declare_parameter("joint_name", "brt_encoder_joint")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("joint_state_topic", "encoder/joint_state")
        self.declare_parameter("joint_state_position_source", "position")
        self.declare_parameter("joint_state_velocity_source", NONE_SOURCE)
        self.declare_parameter("reset_zero_service", "~/reset_zero")

        # 原始计数话题是全局开关；每个读数是否启用由 READ_SPECS 决定。
        self.declare_parameter("publish_raw_counts", True)
        for spec in READ_SPECS.values():
            default_publish = spec.kind == "position"
            self.declare_parameter(spec.publish_param, default_publish)
            self.declare_parameter(spec.topic_param, spec.default_topic)
            if spec.raw_topic_param is not None:
                self.declare_parameter(
                    spec.raw_topic_param,
                    spec.default_raw_topic,
                )

        self.port = str(self.get_parameter("port").value)
        self.auto_scan = bool(self.get_parameter("auto_scan").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.address = int(self.get_parameter("address").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.resolution = int(self.get_parameter("resolution").value)

        # 自动扫描状态。port 为空时会扫描；若固定 port 通信失败，也会清空 port 后重新扫描。
        self._last_scan_time = 0.0
        self.scan_interval_sec = float(
            self.get_parameter("scan_interval_sec").value
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.scan_port_globs = [
            str(item) for item in self.get_parameter("scan_port_globs").value
        ]
        self.enable_serial_locks = bool(
            self.get_parameter("enable_serial_locks").value
        )
        self.scan_lock_path = str(self.get_parameter("scan_lock_path").value)
        self.port_lock_dir = str(self.get_parameter("port_lock_dir").value)
        self._scan_lock_file: Optional[object] = None
        self._port_lock_file: Optional[object] = None
        self._locked_port: str = ""
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.sample_time_ms = int(self.get_parameter("sample_time_ms").value)
        self.position_sign = float(self.get_parameter("position_sign").value)
        self.angle_offset_rad = float(
            self.get_parameter("angle_offset_rad").value
        )
        self.zero_offset_count = int(
            self.get_parameter("zero_offset_count").value
        )
        self.reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.verbose = bool(self.get_parameter("verbose").value)

        self.publish_joint_state = bool(
            self.get_parameter("publish_joint_state").value
        )
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.joint_state_position_source = str(
            self.get_parameter("joint_state_position_source").value
        )
        self.joint_state_velocity_source = str(
            self.get_parameter("joint_state_velocity_source").value
        )
        self.publish_raw_counts = bool(
            self.get_parameter("publish_raw_counts").value
        )
        self.publish_flags = {
            spec.kind: bool(self.get_parameter(spec.publish_param).value)
            for spec in READ_SPECS.values()
        }

        self._validate_parameters()

        self.joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
        )

        # 创建 JointState publisher。禁用后仍可发布各个标量话题。
        self.joint_state_pub = None
        if self.publish_joint_state:
            self.joint_state_pub = self.create_publisher(
                JointState,
                self.joint_state_topic,
                qos_profile_sensor_data,
            )

        # 按配置为每个启用的读数创建 value publisher 和 raw publisher。
        # 未启用的读数不会创建 publisher，也不会在轮询周期内读取寄存器。
        self.value_publishers = {}
        self.raw_publishers = {}
        for spec in READ_SPECS.values():
            if not self.publish_flags[spec.kind]:
                continue
            self.value_publishers[spec.kind] = self.create_publisher(
                self._value_msg_type(spec),
                str(self.get_parameter(spec.topic_param).value),
                qos_profile_sensor_data,
            )
            if self.publish_raw_counts and spec.raw_topic_param is not None:
                self.raw_publishers[spec.kind] = self.create_publisher(
                    Int64,
                    str(self.get_parameter(spec.raw_topic_param).value),
                    qos_profile_sensor_data,
                )

        # read_dependency_reasons 记录“哪些已启用话题需要哪个读数”。
        # active_read_kinds 只来自这张依赖表，从而避免读取未启用话题才需要的数据。
        self.read_dependency_reasons = self._build_read_dependency_reasons()
        self.active_read_kinds = self._build_active_read_kinds()
        self.client: Optional[EncoderClient] = None
        self._next_reconnect_time = 0.0
        self._last_error_log_time = 0.0
        self._client_io_lock = threading.Lock()
        self.reset_zero_service_name = str(
            self.get_parameter("reset_zero_service").value
        )
        self.reset_zero_service = self.create_service(
            Trigger,
            self.reset_zero_service_name,
            self._handle_reset_zero,
        )

        # 如果同时启用很多读数，一个 timer 周期会连续发多条 Modbus 请求。
        # 这里用最坏超时时间做提醒，帮助发现 publish_rate_hz 配得过高的情况。
        period = 1.0 / self.publish_rate_hz
        active_count = max(1, len(self.active_read_kinds))
        if self.timeout * active_count > period:
            self.get_logger().warn(
                "Worst-case serial timeout is longer than the publish period; "
                "timer callbacks may run slower than publish_rate_hz."
            )
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "BRT RS485 encoder node started: "
            f"port={self.port}, baudrate={self.baudrate}, "
            f"address={self.address}, active_read_kinds={self.active_read_kinds}, "
            f"publish_rate_hz={self.publish_rate_hz:.3f}"
        )
        self.get_logger().info(
            f"Read dependencies: {self.read_dependency_reasons}"
        )

    def _candidate_serial_ports(self) -> list[str]:
        """Return existing serial ports ordered deterministically and deduplicated."""
        ports: list[str] = []
        seen_realpaths: set[str] = set()
        for pattern in self.scan_port_globs:
            for port in sorted(glob.glob(pattern)):
                realpath = os.path.realpath(port)
                if realpath in seen_realpaths:
                    continue
                seen_realpaths.add(realpath)
                ports.append(port)
        return ports

    def _lock_path_for_port(self, port: str) -> str:
        """Return a stable lock-file path for one serial port."""
        realpath = os.path.realpath(port)
        safe_name = re.sub(r"[^A-Za-z0-9_.-]", "_", realpath.strip("/"))
        return os.path.join(self.port_lock_dir, f"{safe_name}.lock")

    def _try_lock_file(self, path: str, *, blocking: bool) -> Optional[object]:
        """Acquire an advisory inter-process file lock and return its file object."""
        if not self.enable_serial_locks:
            return None

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        lock_file = open(path, "a+")
        flags = fcntl.LOCK_EX
        if not blocking:
            flags |= fcntl.LOCK_NB
        try:
            fcntl.flock(lock_file.fileno(), flags)
        except OSError as exc:
            lock_file.close()
            if not blocking and exc.errno in (errno.EACCES, errno.EAGAIN):
                return None
            raise
        return lock_file

    def _unlock_file(self, lock_file: Optional[object]) -> None:
        if lock_file is None:
            return
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            lock_file.close()

    def _acquire_scan_lock(self) -> Optional[object]:
        return self._try_lock_file(self.scan_lock_path, blocking=True)

    def _try_acquire_port_lock(
        self,
        port: str,
        *,
        blocking: bool = False,
    ) -> Optional[object]:
        return self._try_lock_file(
            self._lock_path_for_port(port),
            blocking=blocking,
        )

    def _claim_port_lock(self, port: str) -> bool:
        """Hold this port lock for the lifetime of the opened polling client."""
        if not self.enable_serial_locks:
            self._locked_port = port
            return True
        if self._port_lock_file is not None and self._locked_port == port:
            return True
        self._release_port_lock()
        lock_file = self._try_acquire_port_lock(port, blocking=False)
        if lock_file is None:
            return False
        self._port_lock_file = lock_file
        self._locked_port = port
        return True

    def _release_port_lock(self) -> None:
        self._unlock_file(self._port_lock_file)
        self._port_lock_file = None
        self._locked_port = ""

    def _claim_port_if_matching_address(self, port: str) -> bool:
        """Probe one port and keep its lock if this node owns that encoder."""
        if not self._claim_port_lock(port):
            self.get_logger().debug(
                f"Skip locked serial port during scan: {port}"
            )
            return False

        client = EncoderClient(
            port,
            baudrate=self.baudrate,
            address=self.address,
            timeout=self.scan_timeout,
            dry_run=False,
            verbose=self.verbose,
        )
        try:
            client.open()
            result = self._read_encoder(client, self.active_read_kinds[0])
            if result.get("raw_value") is not None:
                return True
            self._release_port_lock()
            return False
        except Exception:
            self._release_port_lock()
            raise
        finally:
            client.close()

    def _find_encoder_port(self) -> str:
        """自动扫描串口，找到能响应当前 address 的编码器。"""
        self.get_logger().info(
            f"Auto-scanning for encoder with address {self.address}..."
        )

        scan_lock = self._acquire_scan_lock()
        try:
            serial_ports = self._candidate_serial_ports()
            if not serial_ports:
                self.get_logger().warn(
                    f"No serial ports found by scan_port_globs={self.scan_port_globs}"
                )
                return ""

            self.get_logger().info(
                f"Found {len(serial_ports)} candidate serial ports: {serial_ports}"
            )

            for port in serial_ports:
                try:
                    if self._claim_port_if_matching_address(port):
                        self.get_logger().info(
                            f"Found encoder address {self.address} on port {port}"
                        )
                        return port
                    self.get_logger().debug(
                        f"Port {port} replied without a usable raw_value, "
                        "is locked by another node, or is not the target; trying next."
                    )
                except (EncoderError, TimeoutError, OSError) as exc:
                    self.get_logger().debug(
                        f"Port {port} is not encoder address {self.address}: {exc}"
                    )
                except Exception as exc:
                    self.get_logger().debug(f"Failed to probe {port}: {exc}")
                time.sleep(0.02)

            return ""
        finally:
            self._unlock_file(scan_lock)

    def _validate_parameters(self) -> None:
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")
        if not 1 <= self.address <= 255:
            raise ValueError("address must be 1..255")
        if self.resolution <= 0:
            raise ValueError("resolution must be > 0")
        if self.sample_time_ms <= 0:
            raise ValueError("sample_time_ms must be > 0")
        if self.timeout <= 0.0:
            raise ValueError("timeout must be > 0")
        if self.scan_timeout <= 0.0:
            raise ValueError("scan_timeout must be > 0")
        if self.scan_interval_sec < 0.0:
            raise ValueError("scan_interval_sec must be >= 0")
        if not self.scan_port_globs:
            raise ValueError("scan_port_globs must not be empty")
        if self.enable_serial_locks:
            if not self.scan_lock_path:
                raise ValueError("scan_lock_path must not be empty")
            if not self.port_lock_dir:
                raise ValueError("port_lock_dir must not be empty")
        if self.reconnect_interval_sec < 0.0:
            raise ValueError("reconnect_interval_sec must be >= 0")
        # auto_scan 模式下允许 port 为空，会在运行时动态扫描
        if not self.dry_run and not self.port and not self.auto_scan:
            raise ValueError("port must be set unless dry_run is true or auto_scan is enabled")
        if self.joint_state_position_source not in POSITION_KINDS | {NONE_SOURCE}:
            raise ValueError(
                "joint_state_position_source must be one of: "
                "position, position2, multiturn, none"
            )
        if self.joint_state_velocity_source not in VELOCITY_KINDS | {NONE_SOURCE}:
            raise ValueError(
                "joint_state_velocity_source must be one of: "
                "speed16, speed32, none"
            )

    def _build_read_dependency_reasons(self) -> dict[str, list[str]]:
        dependencies: dict[str, list[str]] = {}

        # 独立标量话题为 true 时，才加入对应编码器读数依赖。
        for kind, enabled in self.publish_flags.items():
            if not enabled:
                continue
            spec = READ_SPECS[kind]
            topic = str(self.get_parameter(spec.topic_param).value)
            dependencies.setdefault(kind, []).append(topic)

            # 原始计数话题依赖同一次读数，不会额外增加 Modbus 请求。
            if self.publish_raw_counts and spec.raw_topic_param is not None:
                raw_topic = str(self.get_parameter(spec.raw_topic_param).value)
                dependencies[kind].append(raw_topic)

        # JointState 本身也是一个话题。若它为 true，则必须读取它声明的来源。
        if self.publish_joint_state:
            if self.joint_state_position_source != NONE_SOURCE:
                dependencies.setdefault(
                    self.joint_state_position_source,
                    [],
                ).append(self.joint_state_topic)
            if self.joint_state_velocity_source != NONE_SOURCE:
                dependencies.setdefault(
                    self.joint_state_velocity_source,
                    [],
                ).append(self.joint_state_topic)

        return dependencies

    def _build_active_read_kinds(self) -> list[str]:
        ordered = [
            kind
            for kind in READ_SPECS
            if kind in self.read_dependency_reasons
        ]
        if not ordered:
            raise ValueError("At least one encoder readout must be enabled")
        return ordered

    def _value_msg_type(self, spec: ReadSpec) -> type:
        # 角度和速度是连续物理量，使用 Float64；圈数是整数，使用 Int64。
        if spec.value_type == "turns":
            return Int64
        return Float64

    def _ensure_client(self) -> Optional[EncoderClient]:
        # 如果启用自动扫描且端口未设置，按 address 查找实际串口。
        # 找到端口后不再等下一个 timer 周期，立即尝试加端口锁并打开，
        # 尽量缩短“扫描成功但端口尚未被本节点认领”的竞争窗口。
        if self.auto_scan and not self.port:
            now = time.monotonic()
            if now - self._last_scan_time < self.scan_interval_sec:
                return None

            self._last_scan_time = now
            found_port = self._find_encoder_port()
            if found_port:
                self.port = found_port
                self.get_logger().info(
                    f"Auto-scan succeeded, using port: {self.port}"
                )
            else:
                self.get_logger().warn(
                    f"Auto-scan failed to find encoder address {self.address}"
                )
                return None

        # 串口已经打开时直接复用，避免每个 timer 周期反复 open/close。
        if self.client is not None:
            return self.client

        # 上一次失败后还没到重连时间，先跳过本周期。
        now = time.monotonic()
        if now < self._next_reconnect_time:
            return None

        attempted_port = self.port
        if not self._claim_port_lock(attempted_port):
            self._next_reconnect_time = now + self.reconnect_interval_sec
            if self.auto_scan:
                self.port = ""
            self._log_error_throttled(
                "Serial port is already locked by another encoder node: "
                f"{attempted_port}"
            )
            return None

        # EncoderClient 封装了 Modbus RTU 帧构造、CRC 校验和寄存器解析。
        client = EncoderClient(
            attempted_port,
            baudrate=self.baudrate,
            address=self.address,
            timeout=self.timeout,
            dry_run=self.dry_run,
            verbose=self.verbose,
        )
        try:
            client.open()
        except Exception as exc:
            client.close()
            self._release_port_lock()
            self._next_reconnect_time = now + self.reconnect_interval_sec
            if self.auto_scan:
                self.port = ""
            self._log_error_throttled(f"Failed to open encoder: {exc}")
            return None

        self.client = client
        self.get_logger().info("Encoder serial connection opened.")
        return self.client

    def _close_client(self, *, forget_port: bool = False) -> None:
        # 读写异常后关闭串口，下个重连窗口再重新 open。
        if self.client is not None:
            self.client.close()
            self.client = None
        self._release_port_lock()
        if forget_port:
            self.port = ""
        self._next_reconnect_time = (
            time.monotonic() + self.reconnect_interval_sec
        )

    def _on_timer(self) -> None:
        with self._client_io_lock:
            client = self._ensure_client()
            if client is None:
                return

            try:
                # 一个周期内顺序读取所有 active_read_kinds。
                # RS485 是半双工总线，顺序请求比并发请求更稳妥。
                samples = self._read_samples(client)
            except (EncoderError, TimeoutError, OSError) as exc:
                self._close_client(forget_port=self.auto_scan)
                self._log_error_throttled(f"Encoder read failed: {exc}")
                return
            except Exception as exc:
                self._close_client(forget_port=self.auto_scan)
                self._log_error_throttled(f"Unexpected encoder error: {exc}")
                return

        stamp = self.get_clock().now().to_msg()
        # 标量话题和 JointState 使用同一批样本，避免同周期内重复读寄存器。
        self._publish_value_topics(samples)
        self._publish_joint_state(samples, stamp)

    def _handle_reset_zero(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self._client_io_lock:
            client = self._ensure_client()
            if client is None:
                response.success = False
                response.message = (
                    f"Encoder address {self.address} is not connected."
                )
                return response

            try:
                client.zero()
            except (EncoderError, TimeoutError, OSError) as exc:
                self._close_client(forget_port=self.auto_scan)
                response.success = False
                response.message = f"Failed to reset encoder zero: {exc}"
                return response
            except Exception as exc:
                self._close_client(forget_port=self.auto_scan)
                response.success = False
                response.message = f"Unexpected reset-zero error: {exc}"
                return response

        response.success = True
        response.message = (
            f"Reset zero succeeded for encoder address {self.address}."
        )
        return response

    def _read_samples(
        self,
        client: EncoderClient,
    ) -> dict[str, EncoderSample]:
        samples = {}
        for kind in self.active_read_kinds:
            result = self._read_encoder(client, kind)
            samples[kind] = self._decode_sample(kind, result)
        return samples

    def _read_encoder(
        self,
        client: EncoderClient,
        kind: str,
    ) -> dict[str, Any]:
        # 这里把节点内部的 kind 映射到底层 EncoderClient 的具体读函数。
        # 所有函数返回统一的 dict，后续再由 _decode_sample 做单位转换。
        readers: dict[str, Callable[[], dict[str, Any]]] = {
            "position": lambda: client.read_singleturn(self.resolution),
            "position2": lambda: client.read_singleturn2(self.resolution),
            "multiturn": lambda: client.read_virtual_multiturn(
                self.resolution
            ),
            "turns": client.read_virtual_turns,
            "speed16": lambda: client.read_speed16(
                self.resolution,
                self.sample_time_ms,
            ),
            "speed32": lambda: client.read_speed32(
                self.resolution,
                self.sample_time_ms,
            ),
        }
        return readers[kind]()

    def _decode_sample(
        self,
        kind: str,
        result: dict[str, Any],
    ) -> EncoderSample:
        raw_value = result.get("raw_value")
        if raw_value is not None:
            raw_value = int(raw_value)

        if kind in POSITION_KINDS and "angle_deg" in result:
            # 底层先按 resolution 算出角度，这里统一转成 ROS 常用的 rad。
            position_rad = math.radians(float(result["angle_deg"]))
            zero_offset_rad = (
                self.zero_offset_count * 2.0 * math.pi / self.resolution
            )
            return EncoderSample(
                kind=kind,
                raw_value=raw_value,
                position_rad=(
                    self.position_sign * (position_rad - zero_offset_rad)
                    + self.angle_offset_rad
                ),
            )

        if kind in VELOCITY_KINDS and "rpm" in result:
            # 底层速度单位为 rpm，ROS 中更常用 rad/s。
            return EncoderSample(
                kind=kind,
                raw_value=raw_value,
                velocity_rad_s=(
                    self.position_sign
                    * float(result["rpm"])
                    * 2.0
                    * math.pi
                    / 60.0
                ),
            )

        if kind == "turns" and raw_value is not None:
            return EncoderSample(kind=kind, raw_value=raw_value, turns=raw_value)

        return EncoderSample(kind=kind, raw_value=raw_value)

    def _publish_value_topics(self, samples: dict[str, EncoderSample]) -> None:
        # 只给已经创建 publisher 的读数发话题；未启用的读数不会进入这里。
        for kind, sample in samples.items():
            if kind in self.value_publishers:
                msg = self._make_value_msg(sample)
                if msg is not None:
                    self.value_publishers[kind].publish(msg)

            if kind in self.raw_publishers and sample.raw_value is not None:
                # 原始计数不做符号、零点、角度单位修正，保持协议返回值。
                raw_msg = Int64()
                raw_msg.data = sample.raw_value
                self.raw_publishers[kind].publish(raw_msg)

    def _make_value_msg(self, sample: EncoderSample) -> Optional[object]:
        # 根据 EncoderSample 中实际存在的物理量选择消息类型。
        if sample.position_rad is not None:
            msg = Float64()
            msg.data = sample.position_rad
            return msg
        if sample.velocity_rad_s is not None:
            msg = Float64()
            msg.data = sample.velocity_rad_s
            return msg
        if sample.turns is not None:
            msg = Int64()
            msg.data = sample.turns
            return msg
        return None

    def _publish_joint_state(
        self,
        samples: dict[str, EncoderSample],
        stamp: object,
    ) -> None:
        if self.joint_state_pub is None:
            return

        # JointState 的 position 和 velocity 可来自不同寄存器读数。
        position_sample = samples.get(self.joint_state_position_source)
        velocity_sample = samples.get(self.joint_state_velocity_source)
        has_position = (
            position_sample is not None
            and position_sample.position_rad is not None
        )
        has_velocity = (
            velocity_sample is not None
            and velocity_sample.velocity_rad_s is not None
        )
        if not has_position and not has_velocity:
            return

        # name 必须和 position/velocity 数组一一对应；这里只表示一个编码器关节。
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.name = [self.joint_name]
        if has_position:
            msg.position = [position_sample.position_rad]
        if has_velocity:
            msg.velocity = [velocity_sample.velocity_rad_s]
        self.joint_state_pub.publish(msg)

    def _log_error_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_time >= 2.0:
            self.get_logger().warn(message)
            self._last_error_log_time = now

    def destroy_node(self) -> bool:
        self._close_client()
        self._release_port_lock()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """Run the BRT RS485 encoder ROS 2 node."""
    rclpy.init(args=args)
    node = BrtRs485EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
