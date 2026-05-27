"""TG-9801 串口传输层。

本模块负责：
1) 串口打开/关闭；
2) 字节发送与缓冲区读取；
3) 可选 RS485Settings 方向控制配置；
4) 枚举系统可用串口设备。

该模块不关心业务协议，仅处理串口 I/O。
"""

from typing import List, Optional, Tuple
import warnings

import serial
from serial.tools import list_ports


class SerialTransport:
    """串口传输封装（底层 I/O）。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 1000000,
        timeout: float = 1.0,
        parity: str = 'N',
        stopbits: float = 1.0,
        bytesize: int = 8,
        use_rs485_mode: bool = False,
        rs485_rts_level_for_tx: bool = True,
        rs485_rts_level_for_rx: bool = False,
        rs485_delay_before_tx: float = 0.0,
        rs485_delay_before_rx: float = 0.0,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.parity = parity.upper()
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.use_rs485_mode = use_rs485_mode
        self.rs485_rts_level_for_tx = rs485_rts_level_for_tx
        self.rs485_rts_level_for_rx = rs485_rts_level_for_rx
        self.rs485_delay_before_tx = rs485_delay_before_tx
        self.rs485_delay_before_rx = rs485_delay_before_rx
        self._ser: Optional[serial.Serial] = None

    @property
    def is_open(self) -> bool:
        """串口是否已打开。"""
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        """打开串口并按配置应用参数。"""
        if self.is_open:
            return
        parity_map = {
            'N': serial.PARITY_NONE,
            'E': serial.PARITY_EVEN,
            'O': serial.PARITY_ODD,
        }
        stopbits_map = {
            1: serial.STOPBITS_ONE,
            1.0: serial.STOPBITS_ONE,
            2: serial.STOPBITS_TWO,
            2.0: serial.STOPBITS_TWO,
        }
        bytesize_map = {
            7: serial.SEVENBITS,
            8: serial.EIGHTBITS,
        }
        if self.parity not in parity_map:
            raise ValueError("parity 仅支持 N/E/O")
        if self.stopbits not in stopbits_map:
            raise ValueError("stopbits 仅支持 1 或 2")
        if self.bytesize not in bytesize_map:
            raise ValueError("bytesize 仅支持 7 或 8")

        self._ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
            parity=parity_map[self.parity],
            stopbits=stopbits_map[self.stopbits],
            bytesize=bytesize_map[self.bytesize],
        )

        # 可选：启用 pyserial RS485 自动收发方向控制。
        if self.use_rs485_mode:
            if not hasattr(serial, 'rs485'):
                warnings.warn(
                    '当前 pyserial 不支持 RS485Settings，已自动回退为普通串口模式。',
                    RuntimeWarning,
                )
            else:
                self._ser.rs485_mode = serial.rs485.RS485Settings(
                    rts_level_for_tx=self.rs485_rts_level_for_tx,
                    rts_level_for_rx=self.rs485_rts_level_for_rx,
                    delay_before_tx=self.rs485_delay_before_tx,
                    delay_before_rx=self.rs485_delay_before_rx,
                )

    def close(self) -> None:
        """关闭串口。"""
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def send(self, data: bytes) -> int:
        """发送字节并 flush，返回实际写入字节数。"""
        if not self.is_open or self._ser is None:
            raise RuntimeError("串口未打开")
        written = self._ser.write(data)
        self._ser.flush()
        return written

    def clear_buffers(self) -> None:
        """清空串口输入/输出缓冲区。"""
        if not self.is_open or self._ser is None:
            return
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def read_available(self) -> bytes:
        """读取当前可用接收字节（非阻塞按 in_waiting 读取）。"""
        if not self.is_open or self._ser is None:
            return b""
        count = self._ser.in_waiting
        if count <= 0:
            return b""
        return self._ser.read(count)

    @staticmethod
    def available_ports() -> List[Tuple[str, str]]:
        """列出系统可用串口设备及描述。"""
        ports = list_ports.comports()
        return [(p.device, p.description) for p in ports]
