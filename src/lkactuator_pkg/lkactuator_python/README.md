# 1) 先 down
sudo ip link set can0 down 2>/dev/null

# 2) 重新配置并 up
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

以上功能已经集成到脚本canup.sh
任意终端输入bash canup.sh即可启动

# 3) 重新生成功能包
colcon build --packages-select lkactuator_python

# 4) 刷新环境变量
source install/setup.bash

前进cd 退格cd .. 就是“后退一层目录”,cd - 是“返回上一个访问过的目录”。

关于candump能接受到但终端收不到的解决方案：重新插拔usb-can