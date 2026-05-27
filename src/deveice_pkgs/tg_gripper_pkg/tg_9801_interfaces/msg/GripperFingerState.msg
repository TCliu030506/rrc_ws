# 状态消息发布时间（广播节点本地时钟），不是设备原始时间戳。
builtin_interfaces/Time stamp

# 夹爪当前位置（来自 get_status 的 position 字段）。
int32 position

# 末端各方向受力量，顺序固定为：[x1, x2, y1, y2, z1, z2]。
# 当前来源为广播节点解析到的状态字段（与设备协议字段同名）。
int32[6] directional_forces

# 位置与六维力分量是否全部有效。
bool valid
