#!/bin/bash
echo ">>> 重新启动 CAN 接口 can0 ..."

# 1. 关闭 can0（如果已存在）
sudo ip link set can0 down

# 2. 重新配置波特率并启动
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 3. 打印状态
ip -details link show can0 | grep -E "can |state"
echo ">>> can0 已启动完成。"
