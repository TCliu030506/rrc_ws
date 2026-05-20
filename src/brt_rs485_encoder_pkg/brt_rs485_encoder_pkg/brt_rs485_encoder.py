#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BRT single-turn RS485 encoder client.

Protocol source: "001 RS485说明书通信协议 单圈 V2.5.pdf".
The encoder speaks standard Modbus-RTU and supports function 0x03
(read holding registers) and 0x06 (write single register).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional


BAUDRATE_CODES = {
    9600: 0x0000,
    19200: 0x0001,
    38400: 0x0002,
    57600: 0x0003,
    115200: 0x0004,
}

MODE_CODES = {
    "query": 0x0000,
    "single": 0x0001,
    "multiturn": 0x0004,
    "speed": 0x0005,
}

MODE_NAMES = {value: key for key, value in MODE_CODES.items()}

REGISTERS = {
    "singleturn": 0x0000,
    "virtual_multiturn": 0x0000,
    "virtual_turns": 0x0002,
    "speed16": 0x0003,
    "address": 0x0004,
    "baudrate": 0x0005,
    "mode": 0x0006,
    "auto_time_ms": 0x0007,
    "zero": 0x0008,
    "direction": 0x0009,
    "speed_sample_ms": 0x000A,
    "current_value": 0x000B,
    "midpoint": 0x000E,
    "speed32": 0x0020,
    "singleturn2": 0x0025,
}

EXCEPTION_CODES = {
    0x01: "Illegal function",
    0x02: "Illegal data address",
    0x03: "Illegal data value",
    0x04: "Slave device failure",
}


class EncoderError(Exception):
    """Base encoder communication error."""


class CrcError(EncoderError):
    """Raised when a Modbus frame CRC is invalid."""


class ModbusException(EncoderError):
    """Raised when a slave returns a Modbus exception response."""


@dataclass
class RegisterRead:
    address: int
    start_register: int
    count: int
    registers: list[int]
    raw_response: bytes


def now_text() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def parse_int(value: str) -> int:
    text = value.strip()
    if text.lower().endswith("h"):
        return int(text[:-1], 16)
    return int(text, 0)


def check_u8(name: str, value: int) -> None:
    if not 0 <= value <= 0xFF:
        raise ValueError(f"{name} must be 0..255, got {value}")


def check_u16(name: str, value: int) -> None:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} must be 0..65535, got {value}")


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc_bytes(data: bytes) -> bytes:
    crc = crc16_modbus(data)
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def append_crc(frame_without_crc: bytes) -> bytes:
    return frame_without_crc + crc_bytes(frame_without_crc)


def frame_crc_ok(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    got = frame[-2] | (frame[-1] << 8)
    return crc16_modbus(frame[:-2]) == got


def build_read_request(address: int, start_register: int, count: int) -> bytes:
    check_u8("address", address)
    check_u16("start_register", start_register)
    check_u16("count", count)
    if count < 1:
        raise ValueError("count must be >= 1")
    frame = bytes(
        (
            address,
            0x03,
            (start_register >> 8) & 0xFF,
            start_register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        )
    )
    return append_crc(frame)


def build_write_request(address: int, register: int, value: int) -> bytes:
    check_u8("address", address)
    check_u16("register", register)
    check_u16("value", value)
    frame = bytes(
        (
            address,
            0x06,
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        )
    )
    return append_crc(frame)


def registers_to_u32(registers: list[int]) -> int:
    if len(registers) != 2:
        raise ValueError("u32 conversion needs exactly 2 registers")
    return ((registers[0] & 0xFFFF) << 16) | (registers[1] & 0xFFFF)


def signed16(value: int) -> int:
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def signed32(value: int) -> int:
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def angle_deg(raw_value: int, resolution: int) -> float:
    if resolution <= 0:
        raise ValueError("resolution must be > 0")
    return raw_value * 360.0 / resolution


def rpm_from_delta(delta_counts: int, resolution: int, sample_time_ms: int) -> float:
    if resolution <= 0:
        raise ValueError("resolution must be > 0")
    if sample_time_ms <= 0:
        raise ValueError("sample_time_ms must be > 0")
    return delta_counts * 60000.0 / (resolution * sample_time_ms)


class TransactionStore:
    """SQLite storage for raw frames and decoded samples."""

    def __init__(self, db_path: str | Path):
        try:
            import sqlite3
        except Exception as exc:
            raise EncoderError(
                "SQLite storage is unavailable in this Python environment. "
                "Use --no-store, or run with a standard Python installation."
            ) from exc

        self.path = Path(db_path)
        if self.path.parent != Path("."):
            self.path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.path)
        self.conn.execute("PRAGMA journal_mode=WAL")
        self._init_schema()

    def _init_schema(self) -> None:
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS frames (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                direction TEXT NOT NULL,
                address INTEGER,
                function INTEGER,
                register_address INTEGER,
                value INTEGER,
                raw_hex TEXT NOT NULL,
                crc_ok INTEGER,
                note TEXT
            )
            """
        )
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS samples (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                address INTEGER,
                kind TEXT NOT NULL,
                raw_value INTEGER,
                angle_deg REAL,
                rpm REAL,
                resolution INTEGER,
                sample_time_ms INTEGER,
                raw_hex TEXT,
                note TEXT
            )
            """
        )
        self.conn.commit()

    def log_frame(
        self,
        direction: str,
        raw: bytes,
        *,
        register_address: Optional[int] = None,
        value: Optional[int] = None,
        crc_ok: Optional[bool] = None,
        note: str = "",
    ) -> None:
        address = raw[0] if len(raw) >= 1 else None
        function = raw[1] if len(raw) >= 2 else None
        self.conn.execute(
            """
            INSERT INTO frames (
                timestamp, direction, address, function, register_address,
                value, raw_hex, crc_ok, note
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                now_text(),
                direction,
                address,
                function,
                register_address,
                value,
                hex_bytes(raw),
                None if crc_ok is None else int(crc_ok),
                note,
            ),
        )
        self.conn.commit()

    def log_sample(
        self,
        kind: str,
        *,
        address: Optional[int],
        raw_value: Optional[int],
        raw_frame: bytes,
        angle: Optional[float] = None,
        rpm: Optional[float] = None,
        resolution: Optional[int] = None,
        sample_time_ms: Optional[int] = None,
        note: str = "",
    ) -> None:
        self.conn.execute(
            """
            INSERT INTO samples (
                timestamp, address, kind, raw_value, angle_deg, rpm,
                resolution, sample_time_ms, raw_hex, note
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                now_text(),
                address,
                kind,
                raw_value,
                angle,
                rpm,
                resolution,
                sample_time_ms,
                hex_bytes(raw_frame) if raw_frame else "",
                note,
            ),
        )
        self.conn.commit()

    def close(self) -> None:
        self.conn.close()


class EncoderClient:
    def __init__(
        self,
        port: Optional[str],
        *,
        baudrate: int = 9600,
        address: int = 1,
        timeout: float = 1.0,
        store: Optional[TransactionStore] = None,
        dry_run: bool = False,
        verbose: bool = False,
        flush_before_tx: bool = True,
    ):
        check_u8("address", address)
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.timeout = timeout
        self.store = store
        self.dry_run = dry_run
        self.verbose = verbose
        self.flush_before_tx = flush_before_tx
        self.ser: Any = None

    def __enter__(self) -> "EncoderClient":
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    def open(self) -> None:
        if self.dry_run:
            return
        if not self.port:
            raise EncoderError("Serial port is required unless --dry-run is used")
        try:
            import serial
        except ImportError as exc:
            raise EncoderError(
                "pyserial is not installed. Run: pip install -r requirements.txt"
            ) from exc
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=min(max(self.timeout, 0.05), 0.2),
            write_timeout=self.timeout,
            inter_byte_timeout=0.05,
        )

    def close(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _ensure_open(self) -> None:
        if self.dry_run:
            return
        if self.ser is None or not self.ser.is_open:
            raise EncoderError("Serial port is not open")

    def _read_exact(self, size: int, timeout: Optional[float] = None) -> bytes:
        self._ensure_open()
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        data = bytearray()
        while len(data) < size:
            chunk = self.ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
                continue
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timeout while reading {size} bytes, got {len(data)}"
                )
        return bytes(data)

    def send_frame(
        self,
        frame: bytes,
        *,
        register_address: Optional[int] = None,
        value: Optional[int] = None,
        note: str = "",
    ) -> None:
        if self.verbose or self.dry_run:
            print(f"TX: {hex_bytes(frame)}")
        if self.store is not None:
            self.store.log_frame(
                "tx",
                frame,
                register_address=register_address,
                value=value,
                crc_ok=frame_crc_ok(frame),
                note=note,
            )
        if self.dry_run:
            return
        self._ensure_open()
        if self.flush_before_tx:
            self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    def read_frame(
        self,
        timeout: Optional[float] = None,
        note: str = "",
        *,
        strict_crc: bool = True,
    ) -> bytes:
        prefix = self._read_exact(2, timeout)
        function = prefix[1]
        if function == 0x03:
            byte_count = self._read_exact(1, timeout)
            rest = self._read_exact(byte_count[0] + 2, timeout)
            frame = prefix + byte_count + rest
        elif function == 0x06:
            frame = prefix + self._read_exact(6, timeout)
        elif function & 0x80:
            frame = prefix + self._read_exact(3, timeout)
        else:
            # Unknown frame. Read until a short idle timeout and store what we got.
            data = bytearray(prefix)
            deadline = time.monotonic() + 0.05
            while time.monotonic() < deadline:
                chunk = self.ser.read(1)
                if chunk:
                    data.extend(chunk)
                    deadline = time.monotonic() + 0.05
            frame = bytes(data)

        crc_ok = frame_crc_ok(frame)
        if self.verbose:
            print(f"RX: {hex_bytes(frame)}  CRC={'OK' if crc_ok else 'BAD'}")
        if self.store is not None:
            self.store.log_frame("rx", frame, crc_ok=crc_ok, note=note)
        if strict_crc and not crc_ok:
            raise CrcError(f"CRC check failed for frame: {hex_bytes(frame)}")
        return frame

    def transact(
        self,
        request: bytes,
        expected_function: int,
        *,
        register_address: Optional[int] = None,
        value: Optional[int] = None,
        note: str = "",
    ) -> bytes:
        self.send_frame(
            request,
            register_address=register_address,
            value=value,
            note=note,
        )
        if self.dry_run:
            return b""
        response = self.read_frame(note=f"{note} response".strip())
        if response[0] != self.address:
            raise EncoderError(
                f"Unexpected slave address {response[0]}, expected {self.address}"
            )
        if response[1] & 0x80:
            code = response[2]
            message = EXCEPTION_CODES.get(code, f"Unknown exception 0x{code:02X}")
            raise ModbusException(f"Modbus exception: {message}")
        if response[1] != expected_function:
            raise EncoderError(
                f"Unexpected function 0x{response[1]:02X}, "
                f"expected 0x{expected_function:02X}"
            )
        return response

    def read_holding_registers(self, start_register: int, count: int) -> RegisterRead:
        request = build_read_request(self.address, start_register, count)
        response = self.transact(
            request,
            0x03,
            register_address=start_register,
            value=count,
            note="read holding registers",
        )
        if self.dry_run:
            return RegisterRead(self.address, start_register, count, [], b"")
        byte_count = response[2]
        if byte_count != count * 2:
            raise EncoderError(
                f"Unexpected byte count {byte_count}, expected {count * 2}"
            )
        data = response[3: 3 + byte_count]
        registers = [
            (data[index] << 8) | data[index + 1]
            for index in range(0, len(data), 2)
        ]
        return RegisterRead(self.address, start_register, count, registers, response)

    def write_register(self, register: int, value: int) -> bytes:
        request = build_write_request(self.address, register, value)
        response = self.transact(
            request,
            0x06,
            register_address=register,
            value=value,
            note="write single register",
        )
        if self.dry_run:
            return b""
        if response != request:
            raise EncoderError(
                f"Write echo mismatch. tx={hex_bytes(request)} rx={hex_bytes(response)}"
            )
        return response

    def read_singleturn(self, resolution: Optional[int] = None) -> dict[str, Any]:
        read = self.read_holding_registers(REGISTERS["singleturn"], 1)
        value = read.registers[0] if read.registers else None
        angle = (
            angle_deg(value, resolution)
            if value is not None and resolution
            else None
        )
        self._log_sample(
            "singleturn",
            value,
            read.raw_response,
            angle=angle,
            resolution=resolution,
        )
        return result_dict("singleturn", value, read.raw_response, angle=angle)

    def read_singleturn2(self, resolution: Optional[int] = None) -> dict[str, Any]:
        read = self.read_holding_registers(REGISTERS["singleturn2"], 2)
        value = registers_to_u32(read.registers) if read.registers else None
        angle = (
            angle_deg(value, resolution)
            if value is not None and resolution
            else None
        )
        self._log_sample(
            "singleturn2",
            value,
            read.raw_response,
            angle=angle,
            resolution=resolution,
        )
        return result_dict("singleturn2", value, read.raw_response, angle=angle)

    def read_virtual_multiturn(
        self, resolution: Optional[int] = None
    ) -> dict[str, Any]:
        read = self.read_holding_registers(REGISTERS["virtual_multiturn"], 2)
        value = registers_to_u32(read.registers) if read.registers else None
        angle = angle_deg(value, resolution) if value is not None and resolution else None
        self._log_sample(
            "virtual_multiturn",
            value,
            read.raw_response,
            angle=angle,
            resolution=resolution,
        )
        return result_dict("virtual_multiturn", value, read.raw_response, angle=angle)

    def read_virtual_turns(self) -> dict[str, Any]:
        read = self.read_holding_registers(REGISTERS["virtual_turns"], 1)
        value = read.registers[0] if read.registers else None
        self._log_sample("virtual_turns", value, read.raw_response)
        return result_dict("virtual_turns", value, read.raw_response)

    def read_speed16(
        self, resolution: Optional[int] = None, sample_time_ms: Optional[int] = None
    ) -> dict[str, Any]:
        read = self.read_holding_registers(REGISTERS["speed16"], 1)
        value = signed16(read.registers[0]) if read.registers else None
        rpm = (
            rpm_from_delta(value, resolution, sample_time_ms)
            if value is not None and resolution and sample_time_ms
            else None
        )
        self._log_sample(
            "speed16",
            value,
            read.raw_response,
            rpm=rpm,
            resolution=resolution,
            sample_time_ms=sample_time_ms,
        )
        return result_dict("speed16", value, read.raw_response, rpm=rpm)

    def read_speed32(
        self, resolution: Optional[int] = None, sample_time_ms: Optional[int] = None
    ) -> dict[str, Any]:
        read = self.read_holding_registers(REGISTERS["speed32"], 2)
        value = signed32(registers_to_u32(read.registers)) if read.registers else None
        rpm = (
            rpm_from_delta(value, resolution, sample_time_ms)
            if value is not None and resolution and sample_time_ms
            else None
        )
        self._log_sample(
            "speed32",
            value,
            read.raw_response,
            rpm=rpm,
            resolution=resolution,
            sample_time_ms=sample_time_ms,
        )
        return result_dict("speed32", value, read.raw_response, rpm=rpm)

    def set_address(self, new_address: int) -> bytes:
        check_u8("new_address", new_address)
        if new_address < 1:
            raise ValueError("new_address must be 1..255")
        response = self.write_register(REGISTERS["address"], new_address)
        if not self.dry_run:
            self.address = new_address
        return response

    def set_baudrate(self, baudrate: int) -> bytes:
        if baudrate not in BAUDRATE_CODES:
            raise ValueError(f"Unsupported baudrate: {baudrate}")
        response = self.write_register(REGISTERS["baudrate"], BAUDRATE_CODES[baudrate])
        if not self.dry_run:
            self.baudrate = baudrate
        return response

    def set_mode(self, mode: str) -> bytes:
        return self.write_register(REGISTERS["mode"], MODE_CODES[mode])

    def set_auto_time(self, milliseconds: int) -> bytes:
        check_u16("milliseconds", milliseconds)
        return self.write_register(REGISTERS["auto_time_ms"], milliseconds)

    def zero(self) -> bytes:
        return self.write_register(REGISTERS["zero"], 1)

    def set_direction(self, direction: str) -> bytes:
        value = {"cw": 0, "ccw": 1}[direction]
        return self.write_register(REGISTERS["direction"], value)

    def set_speed_sample_time(self, milliseconds: int) -> bytes:
        check_u16("milliseconds", milliseconds)
        return self.write_register(REGISTERS["speed_sample_ms"], milliseconds)

    def set_current_value(self, value: int) -> bytes:
        check_u16("value", value)
        return self.write_register(REGISTERS["current_value"], value)

    def set_midpoint(self) -> bytes:
        return self.write_register(REGISTERS["midpoint"], 1)

    def _log_sample(
        self,
        kind: str,
        raw_value: Optional[int],
        raw_frame: bytes,
        *,
        angle: Optional[float] = None,
        rpm: Optional[float] = None,
        resolution: Optional[int] = None,
        sample_time_ms: Optional[int] = None,
    ) -> None:
        if self.store is None:
            return
        self.store.log_sample(
            kind,
            address=self.address,
            raw_value=raw_value,
            raw_frame=raw_frame,
            angle=angle,
            rpm=rpm,
            resolution=resolution,
            sample_time_ms=sample_time_ms,
        )


def result_dict(
    kind: str,
    raw_value: Optional[int],
    raw_frame: bytes,
    *,
    angle: Optional[float] = None,
    rpm: Optional[float] = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {"kind": kind, "raw_value": raw_value}
    if angle is not None:
        result["angle_deg"] = angle
    if rpm is not None:
        result["rpm"] = rpm
    if raw_frame:
        result["raw_frame"] = hex_bytes(raw_frame)
    return result


def print_json(data: Any) -> None:
    print(json.dumps(data, ensure_ascii=False, indent=2))


def decode_auto_frame(
    frame: bytes,
    *,
    mode: str,
    resolution: Optional[int],
    sample_time_ms: Optional[int],
) -> dict[str, Any]:
    if len(frame) in {2, 4} and (len(frame) < 2 or frame[1] != 0x03):
        if len(frame) == 2:
            unsigned = (frame[0] << 8) | frame[1]
            if mode in {"speed16"}:
                value = signed16(unsigned)
                rpm = (
                    rpm_from_delta(value, resolution, sample_time_ms)
                    if resolution and sample_time_ms
                    else None
                )
                return result_dict("speed16_auto_raw", value, frame, rpm=rpm)
            angle = angle_deg(unsigned, resolution) if resolution else None
            return result_dict("singleturn_auto_raw", unsigned, frame, angle=angle)
        unsigned32 = (
            (frame[0] << 24) | (frame[1] << 16) | (frame[2] << 8) | frame[3]
        )
        if mode == "speed32":
            value = signed32(unsigned32)
            rpm = (
                rpm_from_delta(value, resolution, sample_time_ms)
                if resolution and sample_time_ms
                else None
            )
            return result_dict("speed32_auto_raw", value, frame, rpm=rpm)
        angle = angle_deg(unsigned32, resolution) if resolution else None
        kind = "singleturn2_auto_raw" if mode == "single2" else "multiturn_auto_raw"
        return result_dict(kind, unsigned32, frame, angle=angle)

    if len(frame) < 5 or frame[1] != 0x03:
        return {"kind": "raw", "raw_frame": hex_bytes(frame)}

    data = frame[3: 3 + frame[2]]
    registers = [
        (data[index] << 8) | data[index + 1]
        for index in range(0, len(data), 2)
        if index + 1 < len(data)
    ]

    if mode == "auto":
        if len(registers) == 1:
            mode = "single"
        elif len(registers) == 2:
            mode = "multiturn"
        else:
            mode = "raw"

    if mode == "single" and len(registers) >= 1:
        value = registers[0]
        angle = angle_deg(value, resolution) if resolution else None
        return result_dict("singleturn_auto", value, frame, angle=angle)
    if mode == "single2" and len(registers) >= 2:
        value = registers_to_u32(registers[:2])
        angle = angle_deg(value, resolution) if resolution else None
        return result_dict("singleturn2_auto", value, frame, angle=angle)
    if mode == "multiturn" and len(registers) >= 2:
        value = registers_to_u32(registers[:2])
        angle = angle_deg(value, resolution) if resolution else None
        return result_dict("virtual_multiturn_auto", value, frame, angle=angle)
    if mode == "speed16" and len(registers) >= 1:
        value = signed16(registers[0])
        rpm = (
            rpm_from_delta(value, resolution, sample_time_ms)
            if resolution and sample_time_ms
            else None
        )
        return result_dict("speed16_auto", value, frame, rpm=rpm)
    if mode == "speed32" and len(registers) >= 2:
        value = signed32(registers_to_u32(registers[:2]))
        rpm = (
            rpm_from_delta(value, resolution, sample_time_ms)
            if resolution and sample_time_ms
            else None
        )
        return result_dict("speed32_auto", value, frame, rpm=rpm)

    return {
        "kind": "raw_modbus_03",
        "registers": [f"0x{register:04X}" for register in registers],
        "raw_frame": hex_bytes(frame),
    }


def make_store(args: argparse.Namespace) -> Optional[TransactionStore]:
    if args.no_store or args.dry_run:
        return None
    return TransactionStore(args.db)


def make_client(args: argparse.Namespace) -> EncoderClient:
    return EncoderClient(
        args.port,
        baudrate=args.baudrate,
        address=args.address,
        timeout=args.timeout,
        store=make_store(args),
        dry_run=args.dry_run,
        verbose=args.verbose,
    )


def close_client(client: EncoderClient) -> None:
    store = client.store
    client.close()
    if store is not None:
        store.close()


def cmd_ports(_: argparse.Namespace) -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is not installed. Run: pip install -r requirements.txt")
        return 2
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0
    for port in ports:
        print(f"{port.device}\t{port.description}")
    return 0


def cmd_read(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            result = client.read_holding_registers(args.register, args.count)
            print_json(
                {
                    "address": result.address,
                    "start_register": f"0x{result.start_register:04X}",
                    "count": result.count,
                    "registers": [f"0x{register:04X}" for register in result.registers],
                    "raw_response": hex_bytes(result.raw_response),
                }
            )
        return 0
    finally:
        close_client(client)


def cmd_write(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.write_register(args.register, args.value)
            print_json(
                {
                    "register": f"0x{args.register:04X}",
                    "value": f"0x{args.value:04X}",
                    "echo": hex_bytes(response),
                    "ok": True if response or args.dry_run else False,
                }
            )
        return 0
    finally:
        close_client(client)


def cmd_position(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            if args.wide:
                result = client.read_singleturn2(args.resolution)
            else:
                result = client.read_singleturn(args.resolution)
            print_json(result)
        return 0
    finally:
        close_client(client)


def cmd_multiturn(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            print_json(client.read_virtual_multiturn(args.resolution))
        return 0
    finally:
        close_client(client)


def cmd_turns(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            print_json(client.read_virtual_turns())
        return 0
    finally:
        close_client(client)


def cmd_speed(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            if args.wide:
                result = client.read_speed32(args.resolution, args.sample_ms)
            else:
                result = client.read_speed16(args.resolution, args.sample_ms)
            print_json(result)
        return 0
    finally:
        close_client(client)


def cmd_set_address(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_address(args.new_address)
            print_json({"new_address": args.new_address, "echo": hex_bytes(response), "ok": True})
        return 0
    finally:
        close_client(client)


def cmd_set_baudrate(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_baudrate(args.new_baudrate)
            print_json(
                {
                    "new_baudrate": args.new_baudrate,
                    "code": BAUDRATE_CODES[args.new_baudrate],
                    "echo": hex_bytes(response),
                    "ok": True,
                    "note": "Reconnect using the new baudrate after this command.",
                }
            )
        return 0
    finally:
        close_client(client)


def cmd_set_mode(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_mode(args.mode)
            print_json(
                {
                    "mode": args.mode,
                    "code": MODE_CODES[args.mode],
                    "echo": hex_bytes(response),
                    "ok": True,
                }
            )
        return 0
    finally:
        close_client(client)


def cmd_simple_write(args: argparse.Namespace, method_name: str, payload: dict[str, Any]) -> int:
    client = make_client(args)
    try:
        with client:
            method = getattr(client, method_name)
            response = method()
            payload["echo"] = hex_bytes(response)
            payload["ok"] = True
            print_json(payload)
        return 0
    finally:
        close_client(client)


def cmd_set_auto_time(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_auto_time(args.milliseconds)
            print_json(
                {
                    "auto_time_ms": args.milliseconds,
                    "echo": hex_bytes(response),
                    "ok": True,
                }
            )
        return 0
    finally:
        close_client(client)


def cmd_zero(args: argparse.Namespace) -> int:
    return cmd_simple_write(args, "zero", {"action": "zero"})


def cmd_set_direction(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_direction(args.direction)
            print_json({"direction": args.direction, "echo": hex_bytes(response), "ok": True})
        return 0
    finally:
        close_client(client)


def cmd_set_speed_sample(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_speed_sample_time(args.milliseconds)
            print_json(
                {
                    "speed_sample_ms": args.milliseconds,
                    "echo": hex_bytes(response),
                    "ok": True,
                }
            )
        return 0
    finally:
        close_client(client)


def cmd_set_current(args: argparse.Namespace) -> int:
    client = make_client(args)
    try:
        with client:
            response = client.set_current_value(args.value)
            print_json({"current_value": args.value, "echo": hex_bytes(response), "ok": True})
        return 0
    finally:
        close_client(client)


def cmd_set_midpoint(args: argparse.Namespace) -> int:
    return cmd_simple_write(args, "set_midpoint", {"action": "set_midpoint"})


def cmd_monitor(args: argparse.Namespace) -> int:
    client = make_client(args)
    end_time = None if args.seconds <= 0 else time.monotonic() + args.seconds
    try:
        with client:
            while True:
                if end_time is not None and time.monotonic() >= end_time:
                    break
                timeout = args.timeout
                if end_time is not None:
                    timeout = max(0.05, min(timeout, end_time - time.monotonic()))
                try:
                    frame = client.read_frame(
                        timeout=timeout,
                        note="monitor",
                        strict_crc=False,
                    )
                except TimeoutError:
                    if end_time is None:
                        continue
                    if time.monotonic() >= end_time:
                        break
                    continue
                decoded = decode_auto_frame(
                    frame,
                    mode=args.decode,
                    resolution=args.resolution,
                    sample_time_ms=args.sample_ms,
                )
                print_json(decoded)
                if client.store is not None and "raw_value" in decoded:
                    client.store.log_sample(
                        decoded["kind"],
                        address=frame[0],
                        raw_value=decoded.get("raw_value"),
                        raw_frame=frame,
                        angle=decoded.get("angle_deg"),
                        rpm=decoded.get("rpm"),
                        resolution=args.resolution,
                        sample_time_ms=args.sample_ms,
                    )
        return 0
    finally:
        close_client(client)


def cmd_poll(args: argparse.Namespace) -> int:
    client = make_client(args)
    end_time = None if args.seconds <= 0 else time.monotonic() + args.seconds
    try:
        with client:
            while True:
                if args.kind == "position":
                    result = client.read_singleturn(args.resolution)
                elif args.kind == "position2":
                    result = client.read_singleturn2(args.resolution)
                elif args.kind == "multiturn":
                    result = client.read_virtual_multiturn(args.resolution)
                elif args.kind == "turns":
                    result = client.read_virtual_turns()
                elif args.kind == "speed16":
                    result = client.read_speed16(args.resolution, args.sample_ms)
                elif args.kind == "speed32":
                    result = client.read_speed32(args.resolution, args.sample_ms)
                else:
                    raise ValueError(f"Unknown poll kind: {args.kind}")
                print_json(result)
                if end_time is not None and time.monotonic() >= end_time:
                    break
                time.sleep(args.interval)
        return 0
    finally:
        close_client(client)


def cmd_scan(args: argparse.Namespace) -> int:
    client = make_client(args)
    found: list[int] = []
    try:
        with client:
            for address in range(args.start, args.end + 1):
                client.address = address
                try:
                    client.read_holding_registers(REGISTERS["singleturn"], 1)
                except Exception as exc:
                    if args.verbose:
                        print(f"{address}: no response ({exc})")
                    continue
                print(f"Found encoder address: {address}")
                found.append(address)
        print_json({"found": found})
        return 0
    finally:
        close_client(client)


def cmd_self_test(_: argparse.Namespace) -> int:
    vectors = [
        ("read singleturn", build_read_request(1, 0x0000, 1), "01 03 00 00 00 01 84 0A"),
        ("read multiturn", build_read_request(1, 0x0000, 2), "01 03 00 00 00 02 C4 0B"),
        ("read speed16", build_read_request(1, 0x0003, 1), "01 03 00 03 00 01 74 0A"),
        ("set address", build_write_request(1, 0x0004, 2), "01 06 00 04 00 02 49 CA"),
        ("set baudrate", build_write_request(1, 0x0005, 2), "01 06 00 05 00 02 18 0A"),
        ("set mode", build_write_request(1, 0x0006, 1), "01 06 00 06 00 01 A8 0B"),
        ("set auto time", build_write_request(1, 0x0007, 0x0064), "01 06 00 07 00 64 39 E0"),
        ("zero", build_write_request(1, 0x0008, 1), "01 06 00 08 00 01 C9 C8"),
        ("set direction cw", build_write_request(1, 0x0009, 0), "01 06 00 09 00 00 59 C8"),
        ("set speed sample", build_write_request(1, 0x000A, 0x03E8), "01 06 00 0A 03 E8 A9 76"),
        ("set current", build_write_request(1, 0x000B, 0x03E8), "01 06 00 0B 03 E8 F8 B6"),
        ("set midpoint", build_write_request(1, 0x000E, 1), "01 06 00 0E 00 01 29 C9"),
        ("read speed32", build_read_request(1, 0x0020, 2), "01 03 00 20 00 02 C5 C1"),
        ("read singleturn2", build_read_request(1, 0x0025, 2), "01 03 00 25 00 02 D5 C0"),
    ]
    failed = 0
    for name, frame, expected in vectors:
        got = hex_bytes(frame)
        ok = got == expected
        print(f"{'OK' if ok else 'BAD'}  {name}: {got}")
        if not ok:
            print(f"      expected: {expected}")
            failed += 1
    return 1 if failed else 0


def add_common_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", help="Serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600, choices=sorted(BAUDRATE_CODES))
    parser.add_argument("--address", type=int, default=1, help="Encoder slave address, default 1")
    parser.add_argument("--timeout", type=float, default=1.0, help="Response timeout in seconds")
    parser.add_argument("--db", default="encoder_data.sqlite3", help="SQLite storage path")
    parser.add_argument("--no-store", action="store_true", help="Do not store frames/samples")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print TX frame without opening serial port",
    )
    parser.add_argument("--verbose", action="store_true", help="Print TX/RX frames")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="BRT RS485 single-turn encoder Modbus-RTU tool"
    )
    add_common_serial_args(parser)
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("ports", help="List serial ports").set_defaults(func=cmd_ports)

    p = sub.add_parser("read", help="Read holding registers with function 0x03")
    p.add_argument("register", type=parse_int)
    p.add_argument("count", type=parse_int)
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("write", help="Write one register with function 0x06")
    p.add_argument("register", type=parse_int)
    p.add_argument("value", type=parse_int)
    p.set_defaults(func=cmd_write)

    p = sub.add_parser("position", help="Read single-turn position")
    p.add_argument("--resolution", type=int, default=65536)
    p.add_argument("--wide", action="store_true", help="Use 0x0025~0x0026 for 17-bit+ encoders")
    p.set_defaults(func=cmd_position)

    p = sub.add_parser("multiturn", help="Read virtual multiturn value")
    p.add_argument("--resolution", type=int, default=65536)
    p.set_defaults(func=cmd_multiturn)

    sub.add_parser("turns", help="Read virtual turns register 0x0002").set_defaults(func=cmd_turns)

    p = sub.add_parser("speed", help="Read angular speed delta and optional RPM")
    p.add_argument("--resolution", type=int, default=65536)
    p.add_argument("--sample-ms", type=int, default=100)
    p.add_argument("--wide", action="store_true", help="Use 0x0020~0x0021 signed 32-bit speed")
    p.set_defaults(func=cmd_speed)

    p = sub.add_parser("set-address", help="Set encoder address register 0x0004")
    p.add_argument("new_address", type=int)
    p.set_defaults(func=cmd_set_address)

    p = sub.add_parser("set-baudrate", help="Set baudrate register 0x0005")
    p.add_argument("new_baudrate", type=int, choices=sorted(BAUDRATE_CODES))
    p.set_defaults(func=cmd_set_baudrate)

    p = sub.add_parser("set-mode", help="Set encoder mode register 0x0006")
    p.add_argument("mode", choices=sorted(MODE_CODES))
    p.set_defaults(func=cmd_set_mode)

    p = sub.add_parser("set-auto-time", help="Set auto return interval register 0x0007 in ms")
    p.add_argument("milliseconds", type=int)
    p.set_defaults(func=cmd_set_auto_time)

    sub.add_parser("zero", help="Set current position as zero").set_defaults(func=cmd_zero)

    p = sub.add_parser("set-direction", help="Set increment direction: cw=0, ccw=1")
    p.add_argument("direction", choices=["cw", "ccw"])
    p.set_defaults(func=cmd_set_direction)

    p = sub.add_parser("set-speed-sample", help="Set speed sampling time register 0x000A in ms")
    p.add_argument("milliseconds", type=int)
    p.set_defaults(func=cmd_set_speed_sample)

    p = sub.add_parser("set-current", help="Set current encoder value register 0x000B")
    p.add_argument("value", type=int)
    p.set_defaults(func=cmd_set_current)

    sub.add_parser(
        "set-midpoint",
        help="Set current point as midpoint",
    ).set_defaults(func=cmd_set_midpoint)

    p = sub.add_parser("monitor", help="Receive and store auto-return frames")
    p.add_argument("--seconds", type=float, default=0, help="0 means run forever")
    p.add_argument(
        "--decode",
        choices=["auto", "raw", "single", "single2", "multiturn", "speed16", "speed32"],
        default="auto",
    )
    p.add_argument("--resolution", type=int, default=65536)
    p.add_argument("--sample-ms", type=int, default=100)
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser("poll", help="Actively poll data and store samples")
    p.add_argument(
        "kind",
        choices=[
            "position",
            "position2",
            "multiturn",
            "turns",
            "speed16",
            "speed32",
        ],
    )
    p.add_argument("--interval", type=float, default=0.1)
    p.add_argument("--seconds", type=float, default=0, help="0 means run forever")
    p.add_argument("--resolution", type=int, default=65536)
    p.add_argument("--sample-ms", type=int, default=100)
    p.set_defaults(func=cmd_poll)

    p = sub.add_parser("scan", help="Scan encoder addresses by reading register 0x0000")
    p.add_argument("--start", type=int, default=1)
    p.add_argument("--end", type=int, default=255)
    p.set_defaults(func=cmd_scan)

    sub.add_parser(
        "self-test",
        help="Check CRC vectors from the manual",
    ).set_defaults(func=cmd_self_test)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not hasattr(args, "func"):
        parser.print_help()
        return 2
    if args.command not in {"ports", "self-test"} and not args.dry_run and not args.port:
        parser.error("--port is required unless --dry-run is used")
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
