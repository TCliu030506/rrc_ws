# learning_qos 延时测试日志说明

本文档说明 `qos_helloworld_pub` / `qos_helloworld_sub` 在 DDS 延时测试时打印的所有关键数值含义。

## 1. 订阅端日志字段（one-way）

示例：

```text
[INFO] [1776416351.287283630] [qos_helloworld_sub]: I heard seq=559, msg="Hello World", one-way=0.493ms (双机需NTP/PTP同步才准确)
```

字段说明：

1. `[INFO]`
   日志级别，表示普通信息输出。
2. `[1776416351.287283630]`
   ROS 日志时间戳（秒，浮点格式），表示该条日志打印时刻。
3. `[qos_helloworld_sub]`
   打印该日志的节点名（订阅端节点）。
4. `seq=559`
   消息序号。每发送一条消息序号递增，用于匹配消息和 ACK。
5. `msg="Hello World"`
   消息体中的业务文本内容。
6. `one-way=0.493ms`
   单向时延（发布端 -> 订阅端）约为 0.493 毫秒。

计算公式：

$$
\text{one-way delay} = t_{sub\_recv} - t_{pub\_send}
$$

说明：

- 双机场景下，若两机时钟未同步，该值只适合看趋势，不适合当作严格绝对值。
- 若做了 NTP/PTP 严格同步，one-way 更有参考意义。

## 2. 发布端日志字段（RTT 统计）

示例：

```text
[INFO] [1776416351.338409654] [qos_helloworld_pub]: RTT seq=560, rtt=1.666ms, avg=1.340ms, p95=1.812ms, p99=2.283ms, n=200
```

字段说明：

1. `[INFO]`
   日志级别，表示普通信息输出。
2. `[1776416351.338409654]`
   ROS 日志时间戳（秒，浮点格式），表示该条日志打印时刻。
3. `[qos_helloworld_pub]`
   打印该日志的节点名（发布端节点）。
4. `RTT seq=560`
   正在统计的这一条消息序号是 560。
5. `rtt=1.666ms`
   当前这条消息的往返时延（Round Trip Time）。

计算过程：

- 发布端发送消息并记录时间。
- 订阅端收到后发布 ACK。
- 发布端收到 ACK，计算从发送到 ACK 返回的总耗时。

6. `avg=1.340ms`
   统计窗口内 RTT 的平均值（滑动窗口平均）。
7. `p95=1.812ms`
   95 分位值：窗口内 95% 的 RTT 样本小于等于 1.812 ms。
8. `p99=2.283ms`
   99 分位值：窗口内 99% 的 RTT 样本小于等于 2.283 ms。
9. `n=200`
   当前参与统计的 RTT 样本数量（窗口样本数）。

## 3. 如何快速解读这些值

1. 看链路整体水平：优先看 `avg`。
2. 看抖动和尾部风险：重点看 `p95`、`p99`。
3. 看偶发异常：观察单条 `rtt` 是否明显高于 `p99`。
4. 看是否丢包/乱序：检查 `seq` 是否连续递增。
5. 双机下若未同步时钟：`one-way` 只看相对变化趋势，绝对判断以 `RTT` 为主。

## 4. 术语速查

1. one-way：单向时延（发布到订阅）。
2. RTT：往返时延（发布 -> 订阅 -> ACK 返回发布）。
3. p95/p99：分位数，用于衡量尾延迟和抖动稳定性。
4. 滑动窗口：仅对最近 `n` 条样本做统计。

## 5. 当前测试程序默认行为

1. 测试消息话题：`chatter`
2. ACK 回传话题：`chatter_ack`
3. 消息内携带字段：`seq`、`send_ns`、`msg`
4. 发布端持续输出：`rtt/avg/p95/p99/n`
5. 订阅端持续输出：`one-way`

## 6. 双机时钟同步：PTP 步骤

如果要让 `one-way` 的绝对值更可信，建议先把两台机器的系统时间同步好，再做 DDS 延时测试。PTP 适合有线网卡和交换机支持较好的场景；如果你现在是通过无线网卡连接，建议直接看下一节的 chrony 步骤。

### 6.1 先确认网卡和网络条件

在两台机器上分别执行：

```bash
ip link
ethtool -T eth0
```

说明：

1. `eth0` 是示例网卡名，实际以你的机器为准。
2. 如果 `ethtool -T` 显示支持硬件时间戳，PTP 精度会更好。
3. 如果没有硬件时间戳，也可以使用软件时间戳，但精度会下降。

### 6.2 安装 PTP 工具

两台机器都执行：

```bash
sudo apt update
sudo apt install linuxptp ethtool
```

### 6.3 建议先停掉 chrony

如果系统里已经运行了 chrony，建议在做 PTP 测试时先停掉，避免两个对时机制互相干扰：

```bash
sudo systemctl stop chrony
sudo systemctl disable chrony
```

测试结束后，如有需要可以再恢复。

### 6.4 启动 ptp4l

如果网卡支持硬件时间戳，优先使用硬件模式。

主机上执行：

```bash
sudo ptp4l -i eth0 -m -H
```

从机上执行：

```bash
sudo ptp4l -i eth0 -m -H -s
```

如果网卡不支持硬件时间戳，则改用软件模式：

主机上执行：

```bash
sudo ptp4l -i eth0 -m -S
```

从机上执行：

```bash
sudo ptp4l -i eth0 -m -S -s
```

说明：

1. `-m` 表示把同步过程打印到终端。
2. `-H` 表示使用硬件时间戳。
3. `-S` 表示使用软件时间戳。
4. `-s` 表示从机模式。

### 6.5 启动 phc2sys

如果你使用的是硬件时间戳，建议再运行 `phc2sys`，把系统时间对齐到 PTP 时钟。

两台机器都执行：

```bash
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m
```

说明：

1. `-s eth0` 表示源是网卡上的 PTP 硬件时钟。
2. `-c CLOCK_REALTIME` 表示把系统时钟同步到实时钟。
3. `-w` 表示等待 PTP 同步建立后再开始校正。
4. `-m` 表示打印同步日志。

### 6.6 验证同步是否成功

可以在两台机器上分别检查：

```bash
date
timedatectl status
```

如果使用 chrony，还可以检查：

```bash
chronyc tracking
chronyc sources -v
```

如果使用 PTP，重点观察 `ptp4l` 和 `phc2sys` 的输出日志是否逐渐收敛、offset 是否变小且稳定。

### 6.7 同步完成后再测 DDS 延时

PTP 同步完成后，再运行本包的测试程序：

```bash
ros2 run learning_qos qos_helloworld_sub
ros2 run learning_qos qos_helloworld_pub --ros-args -p period_sec:=0.02 -p stats_window:=500
```

此时：

1. `one-way` 的绝对值会更可信。
2. `RTT` 仍然是最稳妥的端到端指标。
3. 如果看到 `seq` 连续、`RTT` 稳定，说明 DDS 通信和时钟同步都正常。

### 6.8 常见注意事项

1. 如果 `ptp4l` 一直不同步，先检查网线、交换机和网卡能力。
2. 如果网络里有交换机，最好确认交换机是否支持 PTP 透传。
3. 如果要做最简单的双机验证，建议先让两台电脑直连测试。
4. 如果你只是想让当前实验的 `one-way` 更接近真实值，PTP 已经足够；如果后续要做更严格的时间分析，再进一步检查硬件时间戳能力。

### 6.9 可直接复制的命令模板

下面给出一套最简模板。假设：

1. 主机网卡名是 `eth0`。
2. 从机网卡名也是 `eth0`。
3. 你先停掉 chrony，再做 PTP 同步。

主机上执行：

```bash
sudo systemctl stop chrony
sudo systemctl disable chrony
sudo ptp4l -i eth0 -m -H
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m
```

从机上执行：

```bash
sudo systemctl stop chrony
sudo systemctl disable chrony
sudo ptp4l -i eth0 -m -H -s
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w -m
```

如果你的网卡不支持硬件时间戳，把 `-H` 改成 `-S`：

```bash
sudo ptp4l -i eth0 -m -S
sudo ptp4l -i eth0 -m -S -s
```

同步完成后，再启动 DDS 测试：

```bash
ros2 run learning_qos qos_helloworld_sub
ros2 run learning_qos qos_helloworld_pub --ros-args -p period_sec:=0.02 -p stats_window:=500
```

## 7. 无线网卡时钟同步：chrony 步骤

如果你是通过无线网卡联网，优先使用 chrony 做时钟同步。无线环境下不建议强行使用 PTP，因为无线链路的时延抖动和驱动能力通常不适合高精度 PTP 对时。

### 7.1 适用场景

1. 两台机器都通过 WiFi 接入同一个局域网。
2. 你希望把 `one-way` 测试做得“足够可信”，而不是追求微秒级极限精度。
3. 你希望部署步骤简单，便于快速验证 DDS 通信延时。

### 7.2 角色划分

建议把其中一台机器作为 `chrony` 时间服务器（server），另一台作为同步客户端（client）：

1. `server`：时间源主机，先保持自身时间稳定，再向局域网提供 NTP 服务。
2. `client`：从机，向 `server` 同步系统时间。

推荐 `server` 这台机器本身能联网，并且已经能正常获取外部时间源，这样对时结果会更稳妥。

### 7.3 安装 chrony

两台机器都执行：

```bash
sudo apt update
sudo apt install chrony
```

### 7.4 配置 server

在作为时间服务器的那台机器上，编辑 `chrony` 配置文件：

```bash
sudo nano /etc/chrony/chrony.conf
```

建议保留或添加外部时间源，例如默认的公网 NTP 源；同时允许局域网客户端访问。结合你当前无线网卡地址 `10.42.0.1/24`，这里可以直接加入：

```conf
allow 10.42.0.0/24
```

说明：

1. `10.42.0.0/24` 是你当前无线网卡所在网段，适合直接用于这台机器的 chrony server 配置。
2. `allow` 的作用是允许该网段内的机器向这台 `server` 请求时间同步。
3. 如果你的 `server` 没有外网时间源，依然可以提供局域网时间服务，但精度会受它自身时钟质量影响。

配置完成后重启服务：

```bash
sudo systemctl enable chrony
sudo systemctl restart chrony
```

### 7.5 配置 client

在作为同步客户端的那台机器上，编辑 `chrony` 配置文件：

```bash
sudo nano /etc/chrony/chrony.conf
```

把默认的公网 `pool` 或 `server` 行注释掉，改成指向你的时间服务器 IP。结合你当前这台机器的实际地址，可以直接写成：

```conf
server 10.42.0.1 iburst prefer
```

说明：

1. `10.42.0.1` 是你这台机器当前无线网卡的实际 IP，如果它作为 chrony server，就直接使用这个地址。
2. `iburst` 可以加快初始同步。
3. `prefer` 表示优先选择这台服务器作为同步源。

配置完成后重启服务：

```bash
sudo systemctl enable chrony
sudo systemctl restart chrony
```

### 7.6 验证同步状态

在两台机器上分别执行：

```bash
chronyc tracking
chronyc sources -v
timedatectl status
```

你重点看这些结果：

1. `chronyc sources -v` 中是否能看到从机已经选中了 `server`。
2. `chronyc tracking` 中 `System time` 和 `Last offset` 是否逐步收敛到较小值。
3. `timedatectl status` 中系统时间是否正常更新。

如果本机启用了防火墙，确保 UDP 123 端口没有被阻断，否则 client 可能无法访问 server。

### 7.7 同步完成后再测 DDS 延时

chrony 同步稳定后，再启动本包的测试程序：

```bash
ros2 run learning_qos qos_helloworld_sub
ros2 run learning_qos qos_helloworld_pub --ros-args -p period_sec:=0.02 -p stats_window:=500
```

此时：

1. `one-way` 的绝对值会比未同步时更可信。
2. `RTT` 仍然是最稳妥的端到端指标。
3. 如果你只是验证 DDS 是否通、延时是否稳定，chrony 已经足够。

### 7.8 常见注意事项

1. 无线网络的抖动通常比有线更大，所以 `one-way` 的波动可能会更明显，这是正常现象。
2. 如果你后续要做更严格的时延分析，有线 + PTP 的结果通常比无线 + chrony 更好。
3. 如果 `chronyc sources -v` 长期看不到同步源，优先检查 `server` 的 IP、无线网络连通性和防火墙。
4. 如果系统里原本没有安装 `chrony`，先执行安装步骤，不要直接运行 `systemctl stop chrony`。
