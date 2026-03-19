from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # 定义6个节点的配置
    nodes_config = [
        {"node_id": "filter_1", "input_topic": "/lkposition", "output_topic": "/out_1", "target_frame_id": "1"},
        {"node_id": "filter_2", "input_topic": "/lkposition", "output_topic": "/out_2", "target_frame_id": "2"},
        {"node_id": "filter_3", "input_topic": "/lkposition", "output_topic": "/out_3", "target_frame_id": "3"},
        {"node_id": "filter_4", "input_topic": "/lkposition", "output_topic": "/out_4", "target_frame_id": "4"},
        {"node_id": "filter_5", "input_topic": "/lkposition", "output_topic": "/out_5", "target_frame_id": "5"},
        {"node_id": "filter_6", "input_topic": "/lkposition", "output_topic": "/out_6", "target_frame_id": "6"},
        # {"node_id": "filter_cmd_1", "input_topic": "/cmd_msg", "output_topic": "/cmd_1", "target_frame_id": "1"},
        # {"node_id": "filter_cmd_2", "input_topic": "/cmd_msg", "output_topic": "/cmd_2", "target_frame_id": "2"},
        # {"node_id": "filter_cmd_3", "input_topic": "/cmd_msg", "output_topic": "/cmd_3", "target_frame_id": "3"},
        # {"node_id": "filter_cmd_4", "input_topic": "/cmd_msg", "output_topic": "/cmd_4", "target_frame_id": "4"},
        # {"node_id": "filter_cmd_5", "input_topic": "/cmd_msg", "output_topic": "/cmd_5", "target_frame_id": "5"},
        # {"node_id": "filter_cmd_6", "input_topic": "/cmd_msg", "output_topic": "/cmd_6", "target_frame_id": "6"},
    ]
    
    for config in nodes_config:
        node = Node(
            package="x6_control_python",
            executable="lk_filter_demo",
            name=f"lk_filter_{config['node_id']}",
            parameters=[
                {"input_topic": config["input_topic"]},
                {"output_topic": config["output_topic"]},
                {"target_frame_id": config["target_frame_id"]}
            ],
            # 可选：添加命名空间
            # namespace=f"filter_{config['node_id']}",
            output="screen"
        )
        ld.add_action(node)
    
    return ld