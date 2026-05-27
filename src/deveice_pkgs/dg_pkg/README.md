# 使用说明
- source install/setup.bash
## 运行方法：
## mtactuator:
- sudo chmod 666 /dev/ttyUSB0
- ros2 run mtactuator mtactuator_server_node_8

- ros2 run dg_python dg_control

## UR5:
- ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 launch_rviz:=false
- ros2 run dg_python dg_ur5
## ROKAE:
- ros2 launch xmate_cr7_script cr7_driver.launch.py
- ros2 run dg_python dg_rokae

colcon build --symlink-install
colcon build --packages-select dg_python

## M3733C1:
- ros2 launch force_sensor forcesensor.launch.py

## omni_common:
- ros2 launch omni_common omni_state.launch.py
- ros2 run dg_python dg_omni_control

git config --global user.name '程钰翔'
git config --global user.email 'cheng-yuxiang328@user.noreply.gitee.com'