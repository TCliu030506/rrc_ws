# serial_client.py
import sys
import threading
import time
from typing import List, Tuple, Optional
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


class SerialClient:
    def __init__(
        self,
        port: str = "com32",
        baudrate: int = 1000000,
        timeout: float = 1.0,
        encoding: str = "utf-8",
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoding = encoding
        self._ser: Optional["serial.Serial"] = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先安装：pip install pyserial")
        if self.is_open:
            return
        self._ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

    def close(self) -> None:
        if self._ser:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def send_bytes(self, data: bytes) -> int:
        if not self.is_open:
            raise RuntimeError("串口未打开，请先调用 open()")
        # 返回写入的字节数
        return self._ser.write(data)

    def send_text(self, text: str, append_newline: bool = False) -> int:
        payload = text.encode(self.encoding)
        if append_newline:
            payload += b"\n"
        return self.send_bytes(payload)

    def flush(self) -> None:
        if not self.is_open:
            raise RuntimeError("串口未打开，请先调用 open()")
        self._ser.flush()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    @staticmethod
    def available_ports() -> List[Tuple[str, str]]:
        if list_ports is None:
            raise RuntimeError("未安装 pyserial")
        ports = list_ports.comports()
        return [(p.device, p.description) for p in ports]


class SerialReader(threading.Thread):
    """
    后台接收线程：轮询串口缓冲区，打印收到的数据（十六进制 + 可见ASCII）。
    """
    def __init__(self, ser: "serial.Serial", encoding: str = "utf-8"):
        super().__init__(daemon=True)
        self._ser = ser
        self._encoding = encoding
        self._stop_evt = threading.Event()

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                if self._ser and self._ser.is_open:
                    n = getattr(self._ser, "in_waiting", 0)
                    if n:
                        data = self._ser.read(n)
                        hex_str = data.hex(" ")
                        ascii_preview = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in data)
                        print(f"\n[RX] 收到 {len(data)} 字节 | HEX: {hex_str} | ASCII: {ascii_preview}")
                    else:
                        # 等待一点时间再检查
                        time.sleep(0.02)
                else:
                    time.sleep(0.05)
            except Exception as e:
                # 捕捉异常，避免线程退出
                print(f"[RX] 接收异常: {e}")
                time.sleep(0.1)


def _parse_hex_string(hex_str: str) -> bytes:
    s = hex_str.strip().lower()
    s = s.replace("0x", " ").replace(",", " ").replace(";", " ")
    parts = [p for p in s.split() if p]
    return bytes(int(p, 16) for p in parts)


def _parse_number_string(num_str: str) -> bytes:
    s = num_str.strip()
    if not s:
        return b""
    s = s.replace(",", " ").replace(";", " ")
    parts = [p for p in s.split() if p]
    values = []
    for p in parts:
        try:
            v = int(p, 0)
        except ValueError:
            raise ValueError(f"无法解析数字: {p}")
        if not (0 <= v <= 255):
            raise ValueError(f"数值超出字节范围(0-255): {v}")
        values.append(v)
    return bytes(values)


def _bytes_to_hex(data: bytes) -> str:
    return data.hex(" ")
# ---------------------- 协议构造 ----------------------
def build_rs485_custom(addr: int, grip: bool) -> bytes:
    """
    简易自定义RS485帧：
    格式: [0xA3,0xB4,long,addr,cmd,cmdvalue,checksum]
    A3 B4 05 addr cmd 00 E8 03 F4
    A3 B4: 固定帧头
    05: 数据长度(不含帧头和校验)
    addr: 设备地址(0..255)
    cmd:03夹取,00释放
    00:保留位                   (释放时忽略)
    E8 03:速度值1000(小端)      (释放时忽略)
    F4: 校验和(addr+cmd)低8位
    """
    if not (0 <= addr <= 255):
        raise ValueError("地址范围应为 0..255")
    cmd = 0x03 if grip else 0x01
    if grip:
        length = 0x05
        data = bytes([addr, cmd, 0x00, 0xE8, 0x03])
        checksum = (length + addr + cmd + 0x00 + 0xE8 + 0x03) & 0xFF
    else:
        length = 0x02
        data = bytes([addr, cmd])
        checksum = (length + addr + cmd) & 0xFF
    return bytes([0xA3, 0xB4, length]) + data + bytes([checksum])

def _modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_modbus_write_coil(unit_id: int, coil_addr: int, on: bool) -> bytes:
    """
    Modbus RTU 写多个保持寄存器(功能码0x10)：
    帧: [ID, 0x10, start_hi, start_lo, count_hi, count_lo, byte_count, reg1_hi, reg1_lo, reg2_hi, reg2_lo, CRC_lo, CRC_hi]
    夹取示例:  01 10 00 0A 00 02 04 03 E8 00 01 32 60
    松开示例: 01 10 00 0A 00 02 04 03 E8 00 00 F3 A0
    03 E8 = 1000 (速度), 00 01(夹取)/00 00(松开)
    """
    if not (0 <= unit_id <= 247):
        raise ValueError("Modbus设备地址范围应为 0..247")
    if not (0 <= coil_addr <= 0xFFFF):
        raise ValueError("起始地址范围应为 0..65535")

    start_hi = (coil_addr >> 8) & 0xFF
    start_lo = coil_addr & 0xFF
    count_hi, count_lo = 0x00, 0x02
    byte_count = 0x04

    # reg1 = 0x03E8, reg2 = 0x0001(夹取)/0x0000(松开)
    reg1_hi, reg1_lo = 0x03, 0xE8
    reg2_hi, reg2_lo = (0x00, 0x01) if on else (0x00, 0x00)

    payload = bytes([
        unit_id, 0x10,
        start_hi, start_lo,
        count_hi, count_lo,
        byte_count,
        reg1_hi, reg1_lo,
        reg2_hi, reg2_lo
    ])

    crc = _modbus_crc16(payload)
    crc_lo = crc & 0xFF
    crc_hi = (crc >> 8) & 0xFF
    return payload + bytes([crc_lo, crc_hi])


def _print_ports() -> None:
    for dev, desc in SerialClient.available_ports():
        print(f"{dev}  -  {desc}")


def main(argv: List[str]) -> int:
    # 启动后列出所有可用串口，用户通过编号选择要连接的端口
    try:
        ports = SerialClient.available_ports()
    except RuntimeError as e:
        print(str(e))
        return 1

    if not ports:
        print("未发现可用串口。请连接设备后重试。")
        return 1

    print("发现以下串口设备：")
    for i, (dev, desc) in enumerate(ports):
        print(f"[{i}] {dev}  -  {desc}")

    # 选择编号
    while True:
        try:
            sel = input("请选择要连接的编号(或 q 退出): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("已退出。")
            return 0

        if not sel:
            continue
        lower = sel.lower()
        if lower in ("exit", "quit", "q"):
            print("已退出。")
            return 0
        try:
            idx = int(sel)
        except ValueError:
            print("请输入有效数字编号。")
            continue
        if not (0 <= idx < len(ports)):
            print(f"编号超出范围(0..{len(ports)-1})。")
            continue
        break

    port = ports[idx][0]
    client = SerialClient(port=port, baudrate=1000000, timeout=1.0)

    reader: Optional[SerialReader] = None
    try:
        client.open()
        print(f"已连接 {port} @ {client.baudrate}，进入交互模式。")
        # 启动后台接收线程
        if client._ser:
            reader = SerialReader(client._ser, encoding=client.encoding)
            reader.start()

        # 顶层协议选择菜单
        while True:
            print("\n请选择协议：")
            print("[1] RS485自定义协议")
            print("[2] ModbusRTU协议")
            print("[q] 退出")
            try:
                sel_proto = input("协议选择 > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("已退出。")
                break
            if not sel_proto:
                continue
            if sel_proto in ("q", "quit", "exit"):
                break
            if sel_proto not in ("1", "2"):
                print("请输入 1 或 2。")
                continue

            # 通用：设备地址
            dev_addr = 1
            try:
                addr_in = input("设备地址(默认1): ").strip()
                if addr_in:
                    dev_addr = int(addr_in, 0)
            except Exception:
                print("地址输入无效，采用默认1。")
                dev_addr = 1

            # 如果是Modbus，寄存器起始地址默认 0x000A
            coil_addr = 0x000A

            # 动作菜单
            while True:
                print("\n请选择动作：")
                print("[1] 夹取")
                print("[2] 松开")
                print("[b] 返回上级协议菜单")
                print("[raw] 手动发送字节(十进制/0x前缀/0b前缀，空格/逗号/分号分隔)")
                print("[q] 退出")
                try:
                    sel_act = input("动作选择 > ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    print("已退出。")
                    sel_act = "q"
                if not sel_act:
                    continue
                if sel_act in ("q", "quit", "exit"):
                    # 退出整个程序
                    raise SystemExit(0)
                if sel_act == "b":
                    # 返回上级
                    break
                if sel_act == "raw":
                    # 手动发送字节
                    try:
                        line = input("输入字节行 > ").strip()
                        data = _parse_number_string(line)
                        if not data:
                            continue
                        n = client.send_bytes(data)
                        client.flush()
                        print(f"[TX] 已发送 {n} 字节 | 数据: {_bytes_to_hex(data)}")
                    except Exception as e:
                        print(f"发送失败: {e}")
                    continue

                if sel_act not in ("1", "2"):
                    print("请输入 1/2/b/raw/q。")
                    continue

                grip = sel_act == "1"  # 1=夹取 => on=True
                try:
                    if sel_proto == "1":
                        frame = build_rs485_custom(dev_addr, grip)
                    else:
                        frame = build_modbus_write_coil(dev_addr, coil_addr, grip)
                    n = client.send_bytes(frame)
                    client.flush()
                    print(f"[TX] 已发送 {n} 字节 | 数据: {_bytes_to_hex(frame)}")
                except Exception as e:
                    print(f"发送失败: {e}")

    finally:
        try:
            if reader:
                reader.stop()
                # 给线程一点时间退出
                time.sleep(0.05)
        finally:
            client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))