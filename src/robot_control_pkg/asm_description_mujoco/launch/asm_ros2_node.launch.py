import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("asm_description_mujoco")
    default_model_path = os.path.join(
        package_share,
        "ur5e_with_asm",
        "scene_all.xml",
    )

    model_path = LaunchConfiguration("model_path")
    keyframe_name = LaunchConfiguration("keyframe_name")
    use_viewer = LaunchConfiguration("use_viewer")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    publish_end_effector_pose = LaunchConfiguration("publish_end_effector_pose")
    publish_raw_sensors = LaunchConfiguration("publish_raw_sensors")
    sensor_topic_prefix = LaunchConfiguration("sensor_topic_prefix")
    end_effector_site_name = LaunchConfiguration("end_effector_site_name")
    end_effector_pose_topic = LaunchConfiguration("end_effector_pose_topic")
    cartesian_command_topic = LaunchConfiguration("cartesian_command_topic")
    publish_rgb = LaunchConfiguration("publish_rgb")
    publish_depth = LaunchConfiguration("publish_depth")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_name = LaunchConfiguration("camera_name")
    depth_near = LaunchConfiguration("depth_near")
    depth_far = LaunchConfiguration("depth_far")
    depth_max_m = LaunchConfiguration("depth_max_m")
    step_rate_hz = LaunchConfiguration("step_rate_hz")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    control_mode = LaunchConfiguration("control_mode")
    raw_command_topic = LaunchConfiguration("raw_command_topic")
    joint_command_topic = LaunchConfiguration("joint_command_topic")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    external_wrench_topic = LaunchConfiguration("external_wrench_topic")
    
    # 添加 TF 相关参数
    use_tf_broadcaster = LaunchConfiguration("use_tf_broadcaster")
    tf_base_frame = LaunchConfiguration("tf_base_frame")
    tf_publish_rate = LaunchConfiguration("tf_publish_rate")

    return LaunchDescription(
        [
            # ========== 原有参数 ==========
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model_path,
                description="Path to the MuJoCo XML model.",
            ),
            DeclareLaunchArgument(
                "keyframe_name",
                default_value="home",
                description="Keyframe to load at startup.",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="true",
                description="Whether to launch the MuJoCo viewer.",
            ),
            DeclareLaunchArgument(
                "publish_joint_states",
                default_value="true",
                description="Whether to publish joint states.",
            ),
            DeclareLaunchArgument(
                "publish_end_effector_pose",
                default_value="true",
                description="Whether to publish the end-effector pose.",
            ),
            DeclareLaunchArgument(
                "publish_raw_sensors",
                default_value="true",
                description="Whether to publish raw sensors.",
            ),
            DeclareLaunchArgument(
                "sensor_topic_prefix",
                default_value="sensors",
                description="Topic prefix for sensor outputs.",
            ),
            DeclareLaunchArgument(
                "end_effector_site_name",
                default_value="asm_ee_site",
                description="MuJoCo site used as the end effector reference.",
            ),
            DeclareLaunchArgument(
                "end_effector_pose_topic",
                default_value="end_effector_pose",
                description="Topic used to publish FK pose.",
            ),
            DeclareLaunchArgument(
                "cartesian_command_topic",
                default_value="cartesian_target_pose",
                description="Topic used to receive Cartesian target poses.",
            ),
            DeclareLaunchArgument(
                "publish_rgb",
                default_value="true",
                description="Whether to publish RGB images.",
            ),
            DeclareLaunchArgument(
                "publish_depth",
                default_value="false",
                description="Whether to publish depth images.",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="640",
                description="Rendered image width.",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="480",
                description="Rendered image height.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera_fixed",
                description="Fixed camera name used for rendering.",
            ),
            DeclareLaunchArgument(
                "depth_near",
                default_value="0.1",
                description="Depth linearization near plane.",
            ),
            DeclareLaunchArgument(
                "depth_far",
                default_value="50.0",
                description="Depth linearization far plane.",
            ),
            DeclareLaunchArgument(
                "depth_max_m",
                default_value="0.0",
                description="Maximum depth value for clipping; 0 disables clipping.",
            ),
            DeclareLaunchArgument(
                "step_rate_hz",
                default_value="0.0",
                description="Simulation step rate; 0 uses the model timestep.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="50.0",
                description="Publishing rate in Hz.",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="position",
                description="Joint command interpretation mode.",
            ),
            DeclareLaunchArgument(
                "raw_command_topic",
                default_value="command_raw",
                description="Topic for raw Float64MultiArray commands.",
            ),
            DeclareLaunchArgument(
                "joint_command_topic",
                default_value="command_joint",
                description="Topic for JointState commands.",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="joint_states",
                description="Topic for published joint states.",
            ),
            DeclareLaunchArgument(
                "external_wrench_topic",
                default_value="/sensors/wrench",
                description="Topic for combined external wrench output.",
            ),
            
            # ========== TF 广播器参数 ==========
            DeclareLaunchArgument(
                "use_tf_broadcaster",
                default_value="true",
                description="Whether to launch the MuJoCo TF broadcaster.",
            ),
            DeclareLaunchArgument(
                "tf_base_frame",
                default_value="world",
                description="Base frame name for TF tree.",
            ),
            DeclareLaunchArgument(
                "tf_publish_rate",
                default_value="50.0",
                description="TF publish rate in Hz.",
            ),

            # ========== 主仿真节点 ==========
            Node(
                package="asm_description_mujoco",
                executable="asm_ros2_node",
                name="asm_mujoco_node",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "keyframe_name": keyframe_name,
                        "use_viewer": use_viewer,
                        "publish_joint_states": publish_joint_states,
                        "publish_end_effector_pose": publish_end_effector_pose,
                        "publish_raw_sensors": publish_raw_sensors,
                        "sensor_topic_prefix": sensor_topic_prefix,
                        "end_effector_site_name": end_effector_site_name,
                        "end_effector_pose_topic": end_effector_pose_topic,
                        "cartesian_command_topic": cartesian_command_topic,
                        "publish_rgb": publish_rgb,
                        "publish_depth": publish_depth,
                        "image_width": image_width,
                        "image_height": image_height,
                        "camera_name": camera_name,
                        "depth_near": depth_near,
                        "depth_far": depth_far,
                        "depth_max_m": depth_max_m,
                        "step_rate_hz": step_rate_hz,
                        "publish_rate_hz": publish_rate_hz,
                        "control_mode": control_mode,
                        "raw_command_topic": raw_command_topic,
                        "joint_command_topic": joint_command_topic,
                        "joint_state_topic": joint_state_topic,
                        "external_wrench_topic": external_wrench_topic,
                    }
                ],
            ),
            
            # ========== TF 广播器节点（延迟启动） ==========
            TimerAction(
                period=1.0,  # 延迟1秒启动，确保仿真节点已启动并发布关节状态
                condition=IfCondition(use_tf_broadcaster),
                actions=[
                    Node(
                        package="asm_description_mujoco",
                        executable="mujoco_tf_broadcaster",
                        name="mujoco_tf_broadcaster",
                        output="screen",
                        parameters=[
                            {
                                "model_path": model_path,
                                "base_frame": tf_base_frame,
                                "publish_rate": tf_publish_rate,
                                "joint_states_topic": joint_state_topic,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )