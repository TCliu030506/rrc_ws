# 使用说明

## 信息格式
- type: aimooe_sdk::msg::AimCoord

# 故障排查

## 如果能找到Aimooe设备但无法建立通讯, 打开访问权限
sudo cp ~/zzrobot_ws/src/aimooe_pkg/aimooe_sdk/dev/aimooe-usb-ethernet.rules /etc/udev/rules.d/

## 重新加载 udev 规则
sudo udevadm control --reload-rules