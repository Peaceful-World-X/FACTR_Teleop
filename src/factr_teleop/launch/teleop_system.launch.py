from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # 1. 主遥操节点 (factr_teleop_franka)
    # Python 代码内部目前通过 hardcode 路径 src/factr_teleop/factr_teleop/configs/ 来查找 config
    # 因此这里只需要传递文件名即可
    
    factr_teleop_node = Node(
        package='factr_teleop',
        executable='factr_teleop_franka',
        name='factr_teleop_franka',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"config_file": "franka_example.yaml"}
        ]
    )

    # 2. 夹爪桥接节点 (gripper_bridge_node)
    gripper_node = Node(
        package='factr_teleop',
        executable='gripper_bridge_node',
        name='gripper_bridge_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            # 串口配置
            # {"port": "/dev/ttyUSB2"},  # 默认: /dev/ttyUSB2
            # {"baudrate": 115200},      # 默认: 115200
            # {"slave_id": 9},           # 默认: 9
            
            # 控制模式配置
            {"control_mode": "absolute"},  # "absolute" 或 "relative"，默认: "absolute"
            {"leader_gripper_min_rad": 0.0},   # Leader 夹爪最小位置（弧度），默认: 0.0
            {"leader_gripper_max_rad": 0.8},   # Leader 夹爪最大位置（弧度），默认: 0.8
        ]
    )

    return LaunchDescription([
        factr_teleop_node,
        gripper_node,
    ])
