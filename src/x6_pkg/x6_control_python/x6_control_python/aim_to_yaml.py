import rclpy
import yaml
import os   
from pathlib import Path
import numpy as np
from rclpy.node import Node
from aimooe_sdk.msg import AimCoord

class aim_to_yaml(Node):
    def __init__(self):
        super().__init__('x6_server_node')
        self.message_count = 0
        self.position_data = []
        self.aim_sub = self.create_subscription(AimCoord,'aimooe_coord',self.aimsub_callback,10)

        package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.declare_parameter('param_file', str(os.path.join(package_path,'param', 'x6param.yaml')))
        self.param_file = Path(self.get_parameter('param_file').value)

    def aimsub_callback(self, msg: AimCoord):
        """
        处理 AimCoord 消息，将 angle_diff 保存到 YAML 文件。
        """
        x6pose = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        # 将当前接收到的值添加到存储列表
        self.position_data.append(x6pose)

        # 增加计数
        self.message_count += 1

        # 如果收到了 20 条消息，计算平均值并保存到 YAML 文件
        if self.message_count >= 20:
            # 计算平均值
            avg_position = np.mean(self.position_data, axis=0)  # 按列计算平均值

            # 保存到 YAML 文件
            self._save_param_to_yaml(avg_position)

            # 重置计数和数据列表，以便下次计算
            self.message_count = 0
            self.position_data = []

    def _save_param_to_yaml(self, values, param_name='x6pose'):
        """
        将 `values` 写入到 ROS 2 参数文件（self.param_file）中，
        结构为:
        <node_name>:
        ros__parameters:
            <param_name>: <values>
        """
        # ---- 1) 读取旧文件（若存在），保留其它参数 ----
        data = {}
        if self.param_file.exists():
            try:
                with open(self.param_file, 'r', encoding='utf-8') as f:
                    loaded = yaml.safe_load(f)
                    if isinstance(loaded, dict):
                        data = loaded
            except Exception as e:
                self.get_logger().warn(f'读取 {self.param_file} 失败，将覆盖写入：{e}')

        # ---- 2) 进入当前节点名对应的块，并进入 ros__parameters ----
        node_key   = self.get_name()                 # 例如 'x6_server_node'
        node_block = data.get(node_key, {})          # 节点块，不存在则新建
        ros_params = node_block.get('ros__parameters', {})  # 参数块

        # ---- 3) 更新/写入目标参数 ----
        arr = np.asarray(values, dtype=float)        # 强制为浮点数组
        ros_params[param_name] = arr.tolist()        # 转回 list 写入 YAML

        node_block['ros__parameters'] = ros_params
        data[node_key] = node_block

        # ---- 4) 安全写回（临时文件 + 原子替换）----
        tmp = self.param_file.with_suffix(self.param_file.suffix + '.tmp')
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
            os.replace(tmp, self.param_file)  # 原子替换，防止写到一半损坏
            self.get_logger().info(f'已保存 {param_name} 到 {self.param_file}')
        except Exception as e:
            self.get_logger().error(f'写入 {self.param_file} 失败：{e}')
            if tmp.exists():
                try:
                    tmp.unlink()
                except Exception:
                    pass

def main():
    rclpy.init()
    node = aim_to_yaml()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
