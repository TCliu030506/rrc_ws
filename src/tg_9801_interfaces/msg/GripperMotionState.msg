# 状态消息发布时间（广播节点本地时钟），不是设备原始时间戳。
builtin_interfaces/Time stamp

# 夹爪状态码（来自 get_status 的 status 字段；寄存器语义由设备协议定义）。
int32 status_code
# 夹爪硬度/接触硬度量（来自 get_status 的 hardness 字段）。
int32 hardness
# 夹爪电流（来自 get_status 的 current 字段）。
int32 current

# 目标夹取力（来自 get_force 读取结果）。
int32 target_force
# 速度设定值（当前实现为读取 holding 寄存器 modbus_speed_register 的值）。
int32 speed
# 随动状态（来自 get_follow_status 读取结果）。
int32 follow_status

# 由 position 与阈值比较得到：position <= open_position_threshold。
bool opened
# 由 position 与阈值比较得到：position >= closed_position_threshold。
bool closed

# 对应字段是否已被成功读取并有效写入本消息。
bool has_status
bool has_force
bool has_speed
bool has_follow_status
