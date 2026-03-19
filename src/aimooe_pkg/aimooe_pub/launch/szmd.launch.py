from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # 定义6个节点的配置
    nodes_config = [
        {"node_id": "filter_1", "input_topic": "/aimooe_coord", "output_topic": "/out_1", "target_frame_id": "点1"},
        {"node_id": "filter_2", "input_topic": "/aimooe_coord", "output_topic": "/out_2", "target_frame_id": "点2"},
        {"node_id": "filter_3", "input_topic": "/aimooe_coord", "output_topic": "/out_3", "target_frame_id": "点3"},
        {"node_id": "filter_4", "input_topic": "/aimooe_coord", "output_topic": "/out_4", "target_frame_id": "点4"},
        {"node_id": "filter_5", "input_topic": "/aimooe_coord", "output_topic": "/out_5", "target_frame_id": "点5"},
        {"node_id": "filter_6", "input_topic": "/aimooe_coord", "output_topic": "/out_6", "target_frame_id": "点6"},  
        {"node_id": "filter_7", "input_topic": "/aimooe_coord", "output_topic": "/out_7", "target_frame_id": "点7"},
        {"node_id": "filter_8", "input_topic": "/aimooe_coord", "output_topic": "/out_8", "target_frame_id": "点8"},
        {"node_id": "filter_9", "input_topic": "/aimooe_coord", "output_topic": "/out_9", "target_frame_id": "点9"},
        {"node_id": "filter_10", "input_topic": "/aimooe_coord", "output_topic": "/out_10", "target_frame_id": "点10"},
        {"node_id": "filter_11", "input_topic": "/aimooe_coord", "output_topic": "/out_11", "target_frame_id": "点11"},
        {"node_id": "filter_12", "input_topic": "/aimooe_coord", "output_topic": "/out_12", "target_frame_id": "点12"},
        {"node_id": "filter_13", "input_topic": "/aimooe_coord", "output_topic": "/out_13", "target_frame_id": "点13"},
        {"node_id": "filter_14", "input_topic": "/insactuator_msg", "output_topic": "/data_4", "target_frame_id": "inspire_actuator_id4"},
        {"node_id": "filter_15", "input_topic": "/insactuator_msg", "output_topic": "/data_5", "target_frame_id": "inspire_actuator_id5"},
            
    ]
    
    for config in nodes_config:
        node = Node(
            package="aimooe_pub",
            executable="szmd_filter_demo",
            name=f"szmd_filter_{config['node_id']}",
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