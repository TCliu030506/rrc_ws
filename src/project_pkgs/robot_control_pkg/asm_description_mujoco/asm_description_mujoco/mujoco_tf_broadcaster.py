import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R


class MujocoTfBroadcaster(Node):
    def __init__(self):
        super().__init__('mujoco_tf_broadcaster')
        
        self.declare_parameter('model_path', '')
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_states_topic', '/joint_states')
        
        self.model_path = str(self.get_parameter('model_path').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        
        if not self.model_path:
            self.get_logger().error("model_path parameter is required!")
            raise RuntimeError("model_path parameter is required")
        
        try:
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.data = mujoco.MjData(self.model)
            self.get_logger().info(f"Loaded model: {self.model_path}")
            self.get_logger().info(f"Total bodies: {self.model.nbody}, Total joints: {self.model.njnt}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise
        
        # 打印模型结构（调试）
        # self._print_model_structure()
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._build_body_cache()
        
        self.joint_states_received = False
        self.joint_states_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_states_callback,
            qos_profile_sensor_data
        )
        
        self.tf_timer = self.create_timer(1.0/self.publish_rate, self._publish_tf)
    
    def _print_model_structure(self):
        """打印模型结构（用于调试）"""
        self.get_logger().info("=== Model Structure ===")
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            parent_id = int(self.model.body_parentid[i])
            parent_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, parent_id) if parent_id >= 0 and parent_id < self.model.nbody else "none"
            self.get_logger().info(f"  Body {i}: {name} (parent_id={parent_id}, parent_name={parent_name})")
    
    def _build_body_cache(self):
        """缓存 body 名称和父子关系"""
        self.body_cache = []
        
        # self.get_logger().info("=== Building Body Cache ===")
        
        for i in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if not body_name:
                continue
            
            parent_id = int(self.model.body_parentid[i])
            
            # 处理父节点
            if parent_id < 0:
                if body_name.lower() == 'world':
                    self.get_logger().info(f"  Skipping world body (index {i})")
                    continue
                parent_name = self.base_frame
            elif parent_id == 0:
                if body_name.lower() == 'world':
                    self.get_logger().info(f"  Skipping world body (index {i})")
                    continue
                parent_name = self.base_frame
            else:
                if parent_id < self.model.nbody:
                    parent_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
                else:
                    parent_name = None
                
                if not parent_name:
                    self.get_logger().warn(f"  Parent {parent_id} not found for {body_name}, using base_frame")
                    parent_name = self.base_frame
            
            self.body_cache.append({
                'id': i,
                'name': body_name,
                'parent_name': parent_name
            })
            # self.get_logger().info(f"  {i}: {body_name} -> {parent_name}")
        
        # self.get_logger().info(f"=== Cached {len(self.body_cache)} bodies ===")
    
    def _joint_states_callback(self, msg: JointState):
        """接收关节状态并更新内部数据"""
        if not msg.name or not msg.position:
            return
        
        for name, position in zip(msg.name, msg.position):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id >= 0:
                qpos_adr = int(self.model.jnt_qposadr[joint_id])
                self.data.qpos[qpos_adr] = position
        
        if msg.velocity and len(msg.velocity) == len(msg.name):
            for name, velocity in zip(msg.name, msg.velocity):
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id >= 0:
                    qvel_adr = int(self.model.jnt_dofadr[joint_id])
                    self.data.qvel[qvel_adr] = velocity
        
        mujoco.mj_forward(self.model, self.data)
        self.joint_states_received = True
    
    def _publish_tf(self):
        if not self.joint_states_received:
            return
        
        now = self.get_clock().now().to_msg()
        
        for body_info in self.body_cache:
            body = self.data.body(body_info['id'])
            
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = body_info['parent_name']
            t.child_frame_id = body_info['name']
            
            # ========== 正确做法：使用相对位姿 ==========
            # body.xpos 是世界坐标系下的绝对位置
            # 对于相对位置，需要从模型中获取 body_pos
            rel_pos = np.array(self.model.body_pos[body_info['id']])
            
            # 获取相对姿态（四元数）
            rel_quat = np.array(self.model.body_quat[body_info['id']])
            
            # 如果父节点不是 world，需要考虑父节点的变换
            # 实际上，mj_forward 已经计算了相对变换到绝对变换
            # 但 TF 需要的是相对变换
            
            # 正确方法：计算相对变换
            parent_id = int(self.model.body_parentid[body_info['id']])
            
            if parent_id == 0 or body_info['parent_name'] == 'world':
                # 直接使用模型定义的相对变换
                t.transform.translation.x = float(rel_pos[0])
                t.transform.translation.y = float(rel_pos[1])
                t.transform.translation.z = float(rel_pos[2])
                
                # 归一化四元数
                rel_quat = self.normalize_quat(rel_quat)
                
                t.transform.rotation.x = float(rel_quat[1])  # x
                t.transform.rotation.y = float(rel_quat[2])  # y
                t.transform.rotation.z = float(rel_quat[3])  # z
                t.transform.rotation.w = float(rel_quat[0])  # w
            else:
                # 需要计算相对于非 world 父节点的变换
                parent_body = self.data.body(parent_id)
                parent_xpos = np.array(parent_body.xpos)
                parent_xmat = np.array(parent_body.xmat).reshape(3, 3)
                
                body_xpos = np.array(body.xpos)
                
                # 相对位置 = parent_xmat^T × (body_xpos - parent_xpos)
                rel_pos = parent_xmat.T @ (body_xpos - parent_xpos)
                
                # 相对旋转 = parent_xmat^T × body_xmat
                body_xmat = np.array(body.xmat).reshape(3, 3)
                rel_rot = parent_xmat.T @ body_xmat
                
                # 转换为四元数
                # rel_quat = self._rot_to_quat(rel_rot)
                rel_quat = R.from_matrix(rel_rot).as_quat()
                # 归一化四元数
                rel_quat = self.normalize_quat(rel_quat)

                t.transform.translation.x = float(rel_pos[0])
                t.transform.translation.y = float(rel_pos[1])
                t.transform.translation.z = float(rel_pos[2])
                
                t.transform.rotation.x = float(rel_quat[0])  # x
                t.transform.rotation.y = float(rel_quat[1])  # y
                t.transform.rotation.z = float(rel_quat[2])  # z
                t.transform.rotation.w = float(rel_quat[3])  # w

            self.tf_broadcaster.sendTransform(t)
            # self.get_logger().info(f"Published TF: {body_info['parent_name']} -> {body_info['name']} (pos={t.transform.translation}, rot={t.transform.rotation})")
    
    def normalize_quat(self, quat):
        norm = np.linalg.norm(quat)
        if norm < 1e-10:
            return np.array([1, 0, 0, 0])  # 单位四元数
        return quat / norm

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MujocoTfBroadcaster()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()