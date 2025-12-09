# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    data_record_node = Node(
        package='bc',
        executable='data_record',
        name='data_record_node',          
        output='screen',                
        parameters=[
            {
                "state_topics": [
                    "/factr_teleop/right/cmd_franka_pos", # 右臂关节位置命令
                    "/franka/right/obs_franka_torque",    # 右臂关节力矩观测
                ]
            },
            {
                "camera_topics": [
                    "/realsense/front/im", # 前摄像头
                ]
            },
            {"dataset_name": "test"}
        ]
    )
    # 1. 遥操节点 (factr_teleop_franka)
    # Python 代码内部目前通过 hardcode 路径 src/factr_teleop/factr_teleop/configs/ 来查找 config
    # 因此这里只需要传递文件名即可
    factr_teleop_franka_right = Node(
        package='factr_teleop',
        executable='factr_teleop_franka',
        name='factr_teleop_franka_right',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"config_file": "franka_right.yaml"}
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
            {"control_mode": "relative"},  # "absolute" 或 "relative"，默认: "absolute"
            {"leader_gripper_min_rad": 0.0},   # Leader 夹爪最小位置（弧度），默认: 0.0
            {"leader_gripper_max_rad": 0.8},   # Leader 夹爪最大位置（弧度），默认: 0.8
        ]
    )
    
    #3. RealSense 摄像头节点
    realsense_node = Node(
        package='cameras',
        executable='realsense',
        name='front',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"serial": "409122274290"},
            {"name": "front"},
        ]
    )

    '''
    realsense_node = Node(
        package='cameras',
        executable='realsense',
        name='front',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"serial": "419122270824"},
            {"name": "front"},
        ]
    )    
    '''

    
    # 4. ZMQ -> ROS 桥接节点（将 Franka ZMQ 数据转为 ROS 话题）
    zmq_ros_bridge_node = Node(
        package='factr_teleop',
        executable='zmq_ros_bridge',
        name='zmq_ros_bridge',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        factr_teleop_franka_right,
        gripper_node,
        data_record_node,
        realsense_node,
        zmq_ros_bridge_node,
    ])