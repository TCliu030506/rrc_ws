# 当前通信协议（广播节点参数 protocol；常见值：modbus_rtu / rs485_custom）。
string protocol

# 当前目标设备地址（广播节点参数 device_addr）。
int32 device_addr

# 设备 ID（来自 get_id）。当 has_device_id=false 时，本字段值无效（通常为0）。
int32 device_id
# 设备出厂序列号（来自 get_serial）。当 has_serial=false 时，本字段值无效（通常为空串）。
string serial

# device_id / serial 是否已成功读取。
bool has_device_id
bool has_serial
