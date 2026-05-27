# 使用说明
## ros2 run inspire_actuator insactuator_server_node
topic: 'insactuator_msg', type: coordinate::msg::ArrayInt16
service: 'insactuator_srv', type: inspire_msg::srv::InspireScript

## ros2 launch inspire_actuator insactuator.launch.py
topic: 'insactuator_msg', type: coordinate::msg::ArrayInt16
service: 'insactuator_srv', type: inspire_msg::srv::InspireScript

ROS_SRV: ros2 service call /insactuator_srv inspire_msg/srv/InspireScript "{command: ' ', id: , data: [ ]}"

## 修改端口配置
insactuator.launch.py会加载param/insactuator.launch.py,修改YAML文件即可

# 故障排查
## 如果能找到inspire设备但无法建立通讯, 打开访问权限, 并设置USB设备符号链接(2号端口)
sudo cp ~/zzrobot_ws/src/inspire_pkg/inspire_actuator/dev/inspire-usb-serial.rules /etc/udev/rules.d/

## 重新加载 udev 规则
sudo udevadm control --reload-rules

## 检查外接串口设备信息
udevadm info --attribute-walk /dev/ttyUSB0 |grep devpath

## Check serial port
ls -l /dev/ttyUSB*

## Unable to open port
maybe the node do not have the permission to write serialport
run "sudo chmod 666 /dev/ttyUSB0" first
or "sudo usermod -aG dialout rus(username)"

## [insactuator_server_node]: WRONG DATA HEAD:-[55 aa]
串口发送的信息原路返回通常是因为串口连接存在环路或者某些配置上的问题导致的。以下是一些可能导致这种情况发生的常见原因：
串口线连接错误： 如果串口的发送线（TX）和接收线（RX）直接连接在一起，发送的数据将会直接返回。确保 TX 和 RX 连接到正确的对应引脚上。
串口设置不匹配： 如果发送端和接收端的波特率、数据位、停止位和校验位等设置不匹配，会导致通信错误，可能导致数据原路返回。确保发送端和接收端的串口设置完全一致。
信号反射： 在长距离传输或高速传输时，信号可能会在传输线上发生反射，从而导致部分信号返回发送端。这种情况可能需要使用阻抗匹配或信号衰减来解决。
串口缓冲区溢出： 如果接收端无法及时处理发送端发送的数据，可能会导致串口缓冲区溢出，此时数据可能会原路返回。
串口设备故障： 串口设备的硬件故障也可能导致数据原路返回。检查串口设备是否工作正常，可能需要更换设备。
针对以上可能原因，逐一检查和排除问题，可以帮助解决串口发送的信息原路返回的情况。