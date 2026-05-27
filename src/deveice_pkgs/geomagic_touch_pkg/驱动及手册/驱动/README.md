# Geomagic Touch / OpenHaptics + TouchDriver 安装说明（ROS2 / omni_common）

本 README 用于在 Linux 上安装两套组件，并成功启动 ROS2 包 `omni_common`：

1. **OpenHaptics SDK Developer Edition 3.4-0**（提供 libHD/libHL 等 SDK 库）
2. **TouchDriver 2023_01_12**（提供设备运行时库、udev 规则、Touch_Setup/Diagnostic 工具、设备配置）

> 适用场景：`ros2 launch omni_common omni_state.launch.py` 启动并读取 Geomagic Touch 状态。

---

## 目录结构

- `1_openhaptics_3.4-0-developer-edition-amd64/openhaptics_3.4-0-developer-edition-amd64/`
  - `install`
  - `opt/`
  - `usr/`
  - `README_INSTALL`

- `2_TouchDriver2023_01_12/TouchDriver_2023_01_12/`
  - `install_haptic_driver`
  - `ListCOMPortHapticDevices`
  - `usr/`
  - `rules.d/99-3dsystems.rules`
  - `bin/Touch_Setup`
  - `bin/Touch_Diagnostic`
  - `bin/Touch_AdvancedConfig`

---

# 0. 前置检查

插入设备后确认 USB ID（示例为 2988:0302）：
```bash
lsusb | grep -i 2988
```
确认串口设备节点（通常是 /dev/ttyACM0）
```bash
ls -l /dev/ttyACM*
```
# 1. 安装 OpenHaptics SDK 3.4-0

进入 OpenHaptics 解压目录并安装：(到该文件夹下右键打开终端)
```bash
cd openhaptics_3.4-0-developer-edition-amd64
sudo ./install
```

安装依赖（用于编译/运行 examples）
```bash
sudo apt update
sudo apt install -y build-essential freeglut3-dev libncurses5-dev zlib1g-dev
```

验证安装路径与环境变量：安装后建议重启一次
```bash
echo $OH_SDK_BASE
ls -la /opt/OpenHaptics/Developer/3.4-0/examples
```

# 2. 安装 TouchDriver 2023_01_12

## 2.1 注意：必须用 bash 执行 install_haptic_driver

该脚本使用 bash 语法，若用 /bin/sh 执行会报错 !=: not found。
正确安装方式：
```bash
cd TouchDriver_2023_01_12
sudo bash ./install_haptic_driver
sudo ldconfig
```

## 2.2 安装并修正 udev 规则（非常关键）

TouchDriver 带的规则可能写的是 idProduct=="0304"，但实际设备常见为 2988:0302。
请确认并修正规则：

查看规则：
```bash
cat /etc/udev/rules.d/99-3dsystems.rules
```
如果设备是 2988:0302，确保规则为：
```bash
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2988", ATTRS{idProduct}=="0302", GROUP="users", MODE="0666"
```

修改后刷新 udev：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
验证普通用户可访问串口：
```bash
ls -l /dev/ttyACM0
```

# 3. 生成 GTDD 配置（解决“初始化失败”的核心步骤）

omni_state 会读取 $GTDD_HOME（默认 ~/.3dsystems）来加载设备配置。
TouchDriver 包内没有直接提供 config 文件，需要通过 Touch_Setup 生成。

推荐方式：不要 sudo，直接用普通用户运行 Touch_Setup
（前提：udev 规则已设置好权限，例如 MODE=0666）
```bash
export GTDD_HOME=$HOME/.3dsystems
mkdir -p "$GTDD_HOME/config"

cd TouchDriver_2023_01_12
./bin/Touch_Setup
./bin/Touch_Diagnostic
```
生成后检查配置是否存在：
```bash
find "$GTDD_HOME" -maxdepth 4 -type f -ls
```
如果必须用 sudo 运行（不推荐，但可用）
必须显式指定 GTDD_HOME，否则会写到 /root/.3dsystems：
```bash
export GTDD_HOME=$HOME/.3dsystems
sudo env GTDD_HOME=$GTDD_HOME ./bin/Touch_Setup
sudo env GTDD_HOME=$GTDD_HOME ./bin/Touch_Diagnostic
```

# 4. 常见坑：配置写到了 /root 以及“多一层 .3dsystems”

## 4.1 如果你误用 sudo 运行 Touch_Setup
配置会出现在：

/root/.3dsystems/config/...

而普通用户目录为空：

/home/<user>/.3dsystems/config/（空）

可以把 root 的配置复制到用户目录（可作为补救）：

```bash
sudo find /root/.3dsystems -maxdepth 4 -type f -ls
mkdir -p ~/.3dsystems/config
sudo cp -a /root/.3dsystems/config/* ~/.3dsystems/config/
sudo chown -R $USER:$USER ~/.3dsystems
```
## 4.2 复制时出现嵌套：~/.3dsystems/.3dsystems/config/...

如果执行了类似：
```bash
mkdir -p ~/.3dsystems
sudo cp -a /root/.3dsystems ~/.3dsystems
```
会导致：

~/.3dsystems/.3dsystems/config/...

修复方法（把文件挪回正确位置）：
```bash
mkdir -p ~/.3dsystems/config
mv ~/.3dsystems/.3dsystems/config/* ~/.3dsystems/config/
rmdir ~/.3dsystems/.3dsystems/config 2>/dev/null || true
rmdir ~/.3dsystems/.3dsystems 2>/dev/null || true
```

# 5. 运行 ROS2 omni_common

如果启动时报 libncurses.so.5: cannot open ...，安装：
```bash
sudo apt install -y libncurses5
```
启动前设置 GTDD_HOME 并 source 工作空间：
```bash
export GTDD_HOME=$HOME/.3dsystems
source ~/zzrobot_ws/install/setup.bash
ros2 launch omni_common omni_state.launch.py
```

# 6. 诊断清单

## 6.1 检查库是否安装

ls -l /usr/lib/libPhantomIOLib42.so
ldconfig -p | grep PhantomIO

## 6.2 检查程序依赖库是否齐全

ldd ~/zzrobot_ws/install/omni_common/lib/omni_common/omni_state | grep -E "not found|libHD|libHL|libHDU|Phantom"

## 6.3 检查 GTDD 配置是否存在

echo $GTDD_HOME
find "$GTDD_HOME" -maxdepth 4 -type f -ls

## 6.4 检查设备与规则是否匹配

lsusb | grep -i 2988
cat /etc/udev/rules.d/99-3dsystems.rules
ls -l /dev/ttyACM*


