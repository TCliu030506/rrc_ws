# 监听电机回复消息话题：
ros2 topic echo /lkposition

# 监听处理后关节角度和位姿消息话题：
ros2 topic echo /x6position

终端输入：
src/lkactuator_pkg/canup.sh 
启动can通讯
sudo ip link set can0 down
断开连接

ros2 run lkactuator_python service
(已弃用)ros2 run lkactuator_python service_time
加载launch文件以启动x6_server并读取yaml中的参数
ros2 launch x6_control_python x6.launch.py
ros2 launch x6_control_python lk6.launch.py
ros2 run lkactuator_python lk_demo
ros2 run x6_control_python move_demo
ros2 run x6_control_python interpo_test
(已弃用)ros2 run x6_control_python interpo_time_control

# 三个实验：
1、重复定位精度
    输入两个位姿，反复抵达这两个位姿，测量定位精度
2、轨迹精度
    执行设定好的轨迹，测量轨迹精度
3、追踪模式
    从光定位仪不断获取位姿，电机不断到达指定位姿

# 心得：
从launch文件启动才会加载参数文件
ros2 run 启动节点只会读取节点内部的参数服务器
x6_demo和x6_server是两个节点，他们之间的参数服务器不能互通(方法不对？)

colcon build --symlink-install （基于task文件）
ctrl+shift+b即可编译 无需输入

hasattr函数用于判断一个对象是否“有某个属性”，返回值为True或False
可用于实现只需初始化一次的场合：若没有该属性，则初始化该属性；若有该属性，则可进行处理（如+1）

定时器触发 → 先收上次反馈 → 再发下一电机读取命令 的模式能显著提高效率（无需等待反馈）

t0 = time.perf_counter()
待测运行时间的函数
t1 = time.perf_counter()
print(f"函数耗时: {(t1 - t0)*1000:.3f} ms")

服务就是大号的发布者和订阅者

未收到角度反馈是因为收到了别的反馈（非92）可用candump监听看看

服务和话题的数组传递需要tolist()，不然会报错（）
但是对于定义好的数组消息类型（如float32[6]），tolist()并不会生效，依然是numpy.array类型
函数重名问题 和系统的函数不要重名否则会报__enter__

以下是影响速度平滑的参数：
1、插值点个数 超过50会锯齿
2、最快电机的速度 超过18会锯齿

提交前先点暂存更改 再写一行字提交 同步更改 

两个安全问题：
   1、每次重新上电后，若让电机运动到x位置，电机可能会运动到下一圈的x位置
   （已解决 让电机断电前停在-180到180之间即可）
   2、轨迹结束时，本应最后一个停止的电机可能会继续运动，损坏结构
