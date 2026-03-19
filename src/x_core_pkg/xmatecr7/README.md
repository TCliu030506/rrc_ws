## 关于本项目

- 珞石xMate CR7机械臂在rviz2中可视化。

## 开始

- 编译通过后，可以按照下面的步骤操作。

### 1.运行节点

在工作空间下启动终端
```sh
source install/setup.bash
  ```
```sh
ros2 launch xmatecr7 rviz.launch.py
```

### 2.可视化模型

- 打开rviz2后，点击左下角Add选项，将rviz_default_plugins目录下的RobotModel和TF两项分别添加，点击右下角的OK完成。
- 将左侧Global Options栏目下Fixed Frame的map改为选择base_link，此时将会出现机械臂各关节坐标系。
- 将RobotModel的Description Topic改为/robot_description，此时将会出现机械臂的实体。


### 3.简单控制

- 可以在Joint State Publisher窗口下拖动或输入各个关节的角度（单位为弧度），改变机械臂位姿。



