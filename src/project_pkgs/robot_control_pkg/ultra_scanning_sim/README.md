当前已经实现了运动规划——柔顺——逆运动学求解——仿真运动的流程

三个启动文件：

ultra_scanning_sim.launch.py 启动扫查仿真测试相关程序
sim_base.launch.py 启动仿真基础的Gazebo环境和重力补偿

force_sensor_pose_test.launch.py 启动力传感器测试相关程序（开发时用于测试力传感器是否正常）

问题1：力传感器没有发布出话题，可能是先前仿真力传感器部分代码和配置丢失？不知道原因？节后需要首先解决这个问题。

解决方法：原因是先前重新卸载安装了gazebo，导致所需的力传感器插件丢失，安装后问题解决。


问题2：力传感器发布的数据不正确，导致运动扫查时柔顺控制不正常。

解决方法：
操作：原因是在asm_description_mount.xacro，同时配置了joint_name 和 body_name，导致力传感器发布的数据不正确。在asm_description_mount.xacro中，将body_name注释，即可解决。！！！ 但是力依旧不正确
操作2：尝试不使用libgazebo_ros_ft_sensor.so