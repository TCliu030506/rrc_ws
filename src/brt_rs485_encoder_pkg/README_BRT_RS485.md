# BRT RS485 单圈编码器通信程序

本程序基于 `001 RS485说明书通信协议 单圈 V2.5.pdf` 编写，用 Python 实现编码器的 Modbus-RTU 通信、主动查询、参数写入、自动回传接收、帧日志和采样数据存储。

## 协议要点

- 串口默认参数：`9600 bps, 8N1`，无校验，1 位停止位。
- 默认站号：`1`，可配置范围 `1..255`。
- 支持功能码：
  - `0x03`：读保持寄存器。
  - `0x06`：写单个寄存器。
- CRC 为标准 Modbus CRC16，实际发送顺序为低字节在前。例如读 `0x0000` 一个寄存器：`01 03 00 00 00 01 84 0A`。

## 安装

```powershell
pip install -r requirements.txt
```

如果电脑有多个 Python，请用实际环境对应的命令，例如：

```powershell
py -m pip install -r requirements.txt
```

## 常用命令

列出串口：

```powershell
python .\brt_rs485_encoder.py ports
```

读取单圈值并按 16 bit 分辨率换算角度：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 position --resolution 65536
```

17 bit 及以上编码器读取单圈值 2：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 position --wide --resolution 131072
```

读取虚拟多圈值：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 multiturn --resolution 65536
```

读取 16 位角速度并换算 RPM：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 speed --resolution 65536 --sample-ms 100
```

读取 32 位角速度 2：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 speed --wide --resolution 65536 --sample-ms 100
```

通用读寄存器：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 read 0x0000 2
```

通用写寄存器：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 write 0x0007 100
```

## 参数设置

设置地址为 `2`：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-address 2
```

设置波特率为 `38400`：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-baudrate 38400
```

设置为查询模式：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-mode query
```

设置为自动回传单圈值：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-mode single
```

设置为自动回传虚拟多圈值：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-mode multiturn
```

设置为自动回传角速度值：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-mode speed
```

设置自动回传周期为 `100 ms`：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-auto-time 100
```

设置当前位置为零点：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 zero
```

设置顺时针递增：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-direction cw
```

设置逆时针递增：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-direction ccw
```

设置角速度采样时间：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-speed-sample 100
```

设置当前编码器值：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-current 1000
```

设置当前位置为中点：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-midpoint
```

## 自动回传接收

先把编码器切到自动回传模式，再监听：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 set-mode single
python .\brt_rs485_encoder.py --port COM3 --address 1 set-auto-time 100
python .\brt_rs485_encoder.py --port COM3 --address 1 monitor --decode single --resolution 65536
```

监听 10 秒：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 monitor --seconds 10 --decode auto
```

## 主动轮询和数据存储

每 `0.1 s` 轮询一次单圈值，持续 `60 s`，自动写入 `encoder_data.sqlite3`：

```powershell
python .\brt_rs485_encoder.py --port COM3 --address 1 poll position --interval 0.1 --seconds 60 --resolution 65536
```

默认会保存两类信息：

- `frames`：所有发送/接收帧、方向、功能码、CRC 状态。
- `samples`：解析后的单圈值、多圈值、速度、角度、RPM 等。

指定数据库路径：

```powershell
python .\brt_rs485_encoder.py --port COM3 --db .\data\encoder.sqlite3 poll position --seconds 10
```

不保存数据：

```powershell
python .\brt_rs485_encoder.py --port COM3 --no-store position
```

## 调试

仅打印发送帧，不打开串口：

```powershell
python .\brt_rs485_encoder.py --dry-run --address 1 read 0x0000 1
```

按说明书样例校验 CRC：

```powershell
python .\brt_rs485_encoder.py self-test
```

扫描地址：

```powershell
python .\brt_rs485_encoder.py --port COM3 scan --start 1 --end 255
```

开启帧打印：

```powershell
python .\brt_rs485_encoder.py --port COM3 --verbose position
```

## ROS 2 节点

功能包提供 `brt_rs485_encoder_node`，按固定频率主动轮询编码器并发布 ROS 2 标准消息：

- `sensor_msgs/msg/JointState`：默认发布到 `encoder/joint_state`，角度单位为 rad，速度单位为 rad/s。
- `std_msgs/msg/Float64`：发布角度或角速度标量。
- `std_msgs/msg/Int64`：发布原始计数、速度原始增量或虚拟圈数。

构建后直接运行：

```bash
ros2 run brt_rs485_encoder_pkg brt_rs485_encoder_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=9600 \
  -p address:=1 \
  -p publish_rate_hz:=20.0 \
  -p publish_position:=true \
  -p resolution:=65536
```

也可以使用 launch。默认会加载
`config/brt_rs485_encoder.yaml`，没有在命令行指定的参数都使用 YAML
里的值：

```bash
ros2 launch brt_rs485_encoder_pkg brt_rs485_encoder.launch.py port:=/dev/ttyUSB0 publish_rate_hz:=50.0
```

使用自定义 YAML：

```bash
ros2 launch brt_rs485_encoder_pkg brt_rs485_encoder.launch.py config_file:=/path/to/encoder.yaml
```

常用参数：

- `publish_position`：发布单圈角度，默认话题 `encoder/position`。
- `publish_position2`：发布 17 bit 及以上单圈角度，默认话题 `encoder/position2`。
- `publish_multiturn`：发布虚拟多圈累计角度，默认话题 `encoder/multiturn`。
- `publish_turns`：发布虚拟圈数，默认话题 `encoder/turns`。
- `publish_speed16`：发布 16 位速度，默认话题 `encoder/speed16`。
- `publish_speed32`：发布 32 位速度，默认话题 `encoder/speed32`。
- `publish_raw_counts`：为已启用的角度/速度读数额外发布原始计数话题。
- `publish_joint_state`：是否发布 `JointState` 汇总消息。
- `joint_state_position_source`：`JointState.position` 来源，可选 `position`、`position2`、`multiturn`、`none`。
- `joint_state_velocity_source`：`JointState.velocity` 来源，可选 `speed16`、`speed32`、`none`。
- `joint_name`：发布到 `JointState.name` 的关节名。
- `joint_state_topic`：`JointState` 输出话题，默认 `encoder/joint_state`。
- `position_sign`：方向系数，反向安装时可设为 `-1.0`。
- `zero_offset_count`：发布角度前先减去的原始零点计数。
- `angle_offset_rad`：发布角度前额外叠加的 rad 偏置。

例如同时发布单圈角度和 16 位速度，并把两者合成到 `JointState`：

```bash
ros2 launch brt_rs485_encoder_pkg brt_rs485_encoder.launch.py \
  publish_position:=true \
  publish_speed16:=true \
  joint_state_position_source:=position \
  joint_state_velocity_source:=speed16
```

如果希望接入标准 `/joint_states` 生态，可以启动时重映射或设置参数：

```bash
ros2 launch brt_rs485_encoder_pkg brt_rs485_encoder.launch.py joint_state_topic:=/joint_states joint_name:=your_joint_name
```

## 寄存器功能覆盖

| 功能 | 地址 | 命令 |
| --- | --- | --- |
| 编码器单圈值 | `0x0000` | `position` |
| 编码器虚拟多圈值 | `0x0000~0x0001` | `multiturn` |
| 编码器虚拟圈数值 | `0x0002` | `turns` |
| 编码器角速度值 | `0x0003` | `speed` |
| 编码器地址 | `0x0004` | `set-address` |
| 波特率 | `0x0005` | `set-baudrate` |
| 编码器模式 | `0x0006` | `set-mode` |
| 自动回传时间 | `0x0007` | `set-auto-time` |
| 重置零点 | `0x0008` | `zero` |
| 值递增方向 | `0x0009` | `set-direction` |
| 角速度采样时间 | `0x000A` | `set-speed-sample` |
| 设置当前值 | `0x000B` | `set-current` |
| 设置中点 | `0x000E` | `set-midpoint` |
| 角速度值 2 | `0x0020~0x0021` | `speed --wide` |
| 单圈值 2 | `0x0025~0x0026` | `position --wide` |
