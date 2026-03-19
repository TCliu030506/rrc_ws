import logging
import struct
from dataclasses import dataclass
from typing import Optional

from pymodbus.client import ModbusSerialClient
import rclpy
from rclpy.node import Node

from force_sensor_interfaces.msg import PressureData


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


class SensorTransmitterModbusNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_transmitter_modbus_node")

        # 参数声明
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("slave", 1)
        self.declare_parameter("parity", "N")
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("bytesize", 8)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("byte_order", "big")
        self.declare_parameter("word_order", "big")
        self.declare_parameter("interval", 0.5)
        self.declare_parameter("topic_name", "force_sensor_1")

        # 读取参数
        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        slave = self.get_parameter("slave").get_parameter_value().integer_value
        parity_param = self.get_parameter("parity").get_parameter_value().string_value
        stopbits = self.get_parameter("stopbits").get_parameter_value().integer_value
        bytesize = self.get_parameter("bytesize").get_parameter_value().integer_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value
        byte_order = self.get_parameter("byte_order").get_parameter_value().string_value
        word_order = self.get_parameter("word_order").get_parameter_value().string_value
        interval = self.get_parameter("interval").get_parameter_value().double_value
        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

        token = (parity_param or "N").strip().upper()
        if token in {"N", "NONE"}:
            parity = "N"
        elif token in {"E", "EVEN"}:
            parity = "E"
        elif token in {"O", "ODD"}:
            parity = "O"
        else:
            raise ValueError("parity 参数仅支持 N/E/O")

        self._interval = max(0.001, float(interval))

        # 创建发布者
        self.publisher = self.create_publisher(PressureData, topic_name, 10)

        # 初始化 Modbus 客户端
        self.client = PressureTransmitterClient(
            port=port,
            baudrate=int(baudrate),
            slave_id=int(slave),
            parity=parity,
            stopbits=int(stopbits),
            bytesize=int(bytesize),
            timeout=float(timeout),
            byte_order=byte_order,
            word_order=word_order,
        )

        self.client.connect()
        self.get_logger().info(f"已连接设备: port={port}, slave={slave}")
        self.get_logger().info(f"发布话题: {topic_name}")

        # 定时读取并发布
        self.timer = self.create_timer(self._interval, self.read_once_and_publish)

    def read_once_and_publish(self) -> None:
        # 单次读取压力与AD值，并换算为克(g)
        try:
            pressure = float(self.client.read_pressure())
            ad_value = int(self.client.read_ad_value())
            grams = float(self.client.pressure_to_grams(pressure))

            msg = PressureData()
            msg.pressure = pressure
            msg.ad_value = ad_value
            msg.grams = grams
            self.publisher.publish(msg)

            self.get_logger().info(
                f"pressure={pressure:.6f} ad={ad_value} grams={grams:.3f}"
            )
        except Exception as exc:
            self.get_logger().error(f"读取/发布失败: {exc}")

    def destroy_node(self) -> bool:
        try:
            self.client.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SensorTransmitterModbusNode] = None
    try:
        node = SensorTransmitterModbusNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logging.error("节点运行失败: %s", exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
