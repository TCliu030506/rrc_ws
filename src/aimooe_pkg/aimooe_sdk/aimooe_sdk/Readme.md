# 功能包适用于Ubuntu22.04 + python3.10 + opencv4.7.0
将功能包直接拷贝到Home目录下
或者修改aimooeSDK-py310文件夹下set_env.sh的环境变量位置

# 检查AimPosition.so文件依赖
ldd AimPosition.so

# 解决libopencv_core.so.407 => not found
1. 根据系统路径修改aimooeSDK文件夹下set_env.sh的环境变量位置
2. 每个终端运行程序前先source set_env.sh

# 在~/aimooeSDK-py310运行
source set_env.sh
python3 Demo.py 