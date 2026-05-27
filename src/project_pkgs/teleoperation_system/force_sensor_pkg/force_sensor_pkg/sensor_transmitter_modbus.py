import argparse
import logging
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional

from pymodbus.client import ModbusSerialClient


@dataclass
class RegisterSpec:
    address: int
    count: int
    data_type: str  # uint16, int16, uint32, int32, float32


class PressureTransmitterClient:
    def __init__(
        self,
        port: str,
        baudrate: int,
        slave_id: int,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
        timeout: float = 1.0,
        byte_order: str = "big",
        word_order: str = "big",
    ) -> None:
        # 创建 Modbus-RTU 串口客户端（RS485）
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout,
        )
        # 从站地址（1-247）
        self.slave_id = slave_id
        self.byte_order = byte_order.lower()
        self.word_order = word_order.lower()

        # 基于手册的寄存器映射（内部地址）
        self.registers = {
            "pressure": RegisterSpec(address=0x0000, count=1, data_type="uint16"),
            "ad_value": RegisterSpec(address=0x0002, count=1, data_type="uint16"),
            "calibration_index": RegisterSpec(address=0x0005, count=1, data_type="uint16"),
            "calibration_value": RegisterSpec(address=0x0006, count=1, data_type="uint16"),
            "calibration_points": RegisterSpec(address=0x0007, count=1, data_type="uint16"),
            "avg_filter": RegisterSpec(address=0x0008, count=1, data_type="uint16"),
            "device_address": RegisterSpec(address=0x0009, count=1, data_type="uint16"),
            "baudrate": RegisterSpec(address=0x000A, count=1, data_type="uint16"),
            "pressure_threshold": RegisterSpec(address=0x000E, count=1, data_type="uint16"),
            "trigger": RegisterSpec(address=0x000F, count=1, data_type="uint16"),
        }

        self._baudrate_map = {
            1: 2400,
            2: 4800,
            3: 9600,
            4: 19200,
            5: 38400,
            6: 57600,
            7: 76800,
            8: 115200,
        }

        # 线性换算：pressure 寄存器 -> 克(g)
        # 用户提供的两点：pressure=0 -> 0g；pressure=1130 -> 1000g
        self._pressure_raw_min = 0.0
        self._pressure_raw_max = 1130.0
        self._pressure_g_min = 0.0
        self._pressure_g_max = 1000.0

    def connect(self) -> None:
        # 建立串口连接
        if not self.client.connect():
            raise ConnectionError("无法连接串口设备，请检查端口与参数")

    def close(self) -> None:
        # 关闭串口连接
        self.client.close()

    def _decode(self, registers: list[int], data_type: str):
        # 16位整数直接从首个寄存器解析
        if data_type in {"uint16", "int16"}:
            value = registers[0]
            if data_type == "uint16":
                return value
            if value & 0x8000:
                return value - 0x10000
            return value

        # 32位类型需要两个寄存器
        if len(registers) != 2:
            raise ValueError("32位数据需要2个寄存器")

        reg_bytes = [r.to_bytes(2, byteorder="big", signed=False) for r in registers]

        # 字节序/字序处理（默认大端）
        if self.byte_order == "little":
            reg_bytes = [b[::-1] for b in reg_bytes]
        if self.word_order == "little":
            reg_bytes = reg_bytes[::-1]

        payload = b"".join(reg_bytes)
        if data_type == "uint32":
            return int.from_bytes(payload, byteorder="big", signed=False)
        if data_type == "int32":
            return int.from_bytes(payload, byteorder="big", signed=True)
        if data_type == "float32":
            return struct.unpack(">f", payload)[0]
        raise ValueError(f"不支持的数据类型: {data_type}")

    def read_register(self, spec: RegisterSpec):
        # 读保持寄存器（功能码 0x03）
        result = self.client.read_holding_registers(
            address=spec.address,
            count=spec.count,
            device_id=self.slave_id,
        )
        if result.isError():
            raise IOError(f"读取寄存器失败: {result}")
        return self._decode(result.registers, spec.data_type)

    def read_pressure(self) -> float:
        # 压力值（16位无符号）
        return float(self.read_register(self.registers["pressure"]))

    def pressure_to_grams(self, pressure_raw: float) -> float:
        # 将 pressure 寄存器值线性换算为克(g)
        raw_span = self._pressure_raw_max - self._pressure_raw_min
        if raw_span <= 0:
            raise ValueError("无效的压力换算范围")
        ratio = (pressure_raw - self._pressure_raw_min) / raw_span
        grams = self._pressure_g_min + ratio * (self._pressure_g_max - self._pressure_g_min)
        return max(0.0, grams)

    def read_ad_value(self) -> int:
        # AD 原始值（16位无符号）
        return int(self.read_register(self.registers["ad_value"]))

    def read_raw(self, address: int, count: int) -> list[int]:
        # 低层读取：返回原始寄存器数组
        result = self.client.read_holding_registers(
            address=address,
            count=count,
            device_id=self.slave_id,
        )
        if result.isError():
            raise IOError(f"读取寄存器失败: {result}")
        return result.registers

    def write_register(self, address: int, value: int) -> None:
        # 写单寄存器（功能码 0x06）
        result = self.client.write_register(
            address=address,
            value=value,
            device_id=self.slave_id,
        )
        if result.isError():
            raise IOError(f"写寄存器失败: {result}")

    def write_registers(self, address: int, values: list[int]) -> None:
        # 写多寄存器（功能码 0x10）
        result = self.client.write_registers(
            address=address,
            values=values,
            device_id=self.slave_id,
        )
        if result.isError():
            raise IOError(f"写多个寄存器失败: {result}")

    def set_device_address(self, new_address: int) -> None:
        # 修改从站地址（立即生效）
        if not 1 <= new_address <= 247:
            raise ValueError("地址范围应为 1-247")
        self.write_register(self.registers["device_address"].address, new_address)

    def set_baudrate(self, baudrate: int) -> None:
        # 修改波特率（立即生效）
        code = None
        for k, v in self._baudrate_map.items():
            if v == baudrate:
                code = k
                break
        if code is None:
            raise ValueError("不支持的波特率")
        self.write_register(self.registers["baudrate"].address, code)

    def set_avg_filter(self, points: int) -> None:
        # 设置平均滤波点数
        if not 1 <= points <= 50:
            raise ValueError("滤波点数范围应为 1-50")
        self.write_register(self.registers["avg_filter"].address, points)

    def set_pressure_threshold(self, value: int) -> None:
        # 设置压力阈值
        if not 0 <= value <= 65535:
            raise ValueError("阈值范围应为 0-65535")
        self.write_register(self.registers["pressure_threshold"].address, value)

    def set_calibration_points(self, points: int) -> None:
        # 设置标定点数
        if not 3 <= points <= 12:
            raise ValueError("标定点数范围应为 3-12")
        self.write_register(self.registers["calibration_points"].address, points)

    def calibrate_point(self, index: int, value: int) -> None:
        # 标定单个点：先写序号，再写标定值
        if not 1 <= index <= 12:
            raise ValueError("标定序号范围应为 1-12")
        if not 0 <= value <= 65535:
            raise ValueError("标定值范围应为 0-65535")
        self.write_register(self.registers["calibration_index"].address, index)
        self.write_register(self.registers["calibration_value"].address, value)

    def enable_auto_send(self) -> None:
        # 触发寄存器写 1，开启自动发送
        self.write_register(self.registers["trigger"].address, 1)

    def factory_reset(self) -> None:
        # 触发寄存器写 9，恢复出厂设置
        self.write_register(self.registers["trigger"].address, 9)


def parse_args() -> argparse.Namespace:
    # 命令行参数
    parser = argparse.ArgumentParser(description="薄膜压力变送器 Modbus-RTU 读取程序")
    parser.add_argument("--port", required=True, help="串口号，例如 COM3")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--slave", type=int, default=1)
    parser.add_argument("--parity", default="N", choices=["N", "E", "O"])
    parser.add_argument("--stopbits", type=int, default=1, choices=[1, 2])
    parser.add_argument("--bytesize", type=int, default=8, choices=[7, 8])
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--byte-order", default="big", choices=["big", "little"])
    parser.add_argument("--word-order", default="big", choices=["big", "little"])
    parser.add_argument("--interval", type=float, default=0.5, help="周期读取间隔（秒）")
    parser.add_argument("--once", action="store_true", help="只读一次")
    parser.add_argument("--log", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log), format="%(asctime)s %(levelname)s %(message)s")

    client = PressureTransmitterClient(
        port=args.port,
        baudrate=args.baudrate,
        slave_id=args.slave,
        parity=args.parity,
        stopbits=args.stopbits,
        bytesize=args.bytesize,
        timeout=args.timeout,
        byte_order=args.byte_order,
        word_order=args.word_order,
    )

    try:
        # 建立连接
        client.connect()
        logging.info("已连接设备")

        def read_once():
            # 单次读取压力与AD值，并换算为克(g)
            pressure = client.read_pressure()
            ad_value = client.read_ad_value()
            grams = client.pressure_to_grams(pressure)
            logging.info("pressure=%.6f ad=%d grams=%.3f", pressure, ad_value, grams)

        if args.once:
            # 只读一次
            read_once()
            return 0

        # 周期性读取
        while True:
            read_once()
            time.sleep(args.interval)

    except KeyboardInterrupt:
        logging.info("用户中断")
        return 0
    except Exception as exc:
        logging.error("运行失败: %s", exc)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
