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


import time
import numpy as np

import rclpy
from sensor_msgs.msg import JointState

from factr_teleop.factr_teleop import FACTRTeleop
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from python_utils.global_configs import franka_left_real_zmq_addresses, franka_right_real_zmq_addresses


def create_array_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg


class FACTRTeleopFrankaZMQ(FACTRTeleop):
    """基于 ZMQ 的 FACTR ↔ Franka 通信实现（Franka 端适配器）。

    该类通过 ZMQ 的发布/订阅机制在 leader（FACTR）和 follower（Franka）之间建立通信，
    并将收到的部分信息转发为 ROS 消息以便于记录与调试，例如 Franka 的关节状态与力矩。

    同时提供一个夹爪力反馈的示例：从 follower 的夹爪或力矩来源订阅力信息，并将其用于 leader 侧的力反馈计算。
    """

    def __init__(self):
        super().__init__()
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.gripper_external_torque = 0.0
        
        # 配置 Leader 到 Follower 的关节偏移量
        arm_config = self.config.get("arm_teleop", {})
        # 关节偏移量：follower_joint_pos = leader_joint_pos + joint_offset[i]
        self.joint_offset = np.array(arm_config.get("joint_offset", [0.0] * self.num_arm_joints))
        
        # 验证偏移量配置
        if len(self.joint_offset) != self.num_arm_joints:
            raise ValueError(f"joint_offset 长度 ({len(self.joint_offset)}) 必须等于关节数量 ({self.num_arm_joints})")
        
        # 记录配置信息
        if np.any(self.joint_offset != 0.0):
            self.get_logger().info(f"关节偏移量配置: {self.joint_offset}")
            self.get_logger().info(f"  示例：Leader 关节 0 的 0 度将映射到 Follower 的 {np.degrees(self.joint_offset[0]):.2f} 度")

    def set_up_communication(self):
        if self.name == "left":
            zmq_addresses = franka_left_real_zmq_addresses
        elif self.name == "right":
            zmq_addresses = franka_right_real_zmq_addresses
        else:
            raise ValueError(f"Invalid robot name '{self.name}'. Expected 'left' or 'right'.")

        # 用于向 Franka 跟随臂发送关节位置命令的 ZMQ 发布器
        self.franka_cmd_pub = ZMQPublisher(zmq_addresses["joint_pos_cmd_pub"])
        # 用于获取 Franka 跟随臂当前关节位置与速度的 ZMQ 订阅器
        self.franka_joint_state_sub = ZMQSubscriber(zmq_addresses["joint_state_sub"])
        # 将 Franka 当前关节状态转发到 ROS 的发布器
        self.obs_franka_state_pub = self.create_publisher(JointState, f'/franka/{self.name}/obs_franka_state', 10)
        # 将 Franka 与夹爪命令转发到 ROS 的发布器
        self.cmd_franka_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/cmd_franka_pos', 10)
        self.cmd_gripper_pos_pub = self.create_publisher(JointState, f'/factr_teleop/{self.name}/cmd_gripper_pos', 10)

        if self.enable_torque_feedback:
            # 用于获取 Franka 跟随臂外部关节力矩的 ZMQ 订阅器
            self.franka_torque_sub = ZMQSubscriber(zmq_addresses["joint_torque_sub"])
            # 将 Franka 的外部关节力矩转发到 ROS 的发布器
            # self.obs_franka_torque_pub = self.create_publisher(JointState, f'/franka/{self.name}/obs_franka_torque', 10)
        
        if self.enable_gripper_feedback:
            # 订阅夹爪力矩信息的 ROS 订阅器
            self.obs_gripper_torque_pub = self.create_subscription(
                JointState, f'/gripper/{self.name}/obs_gripper_torque', 
                self._gripper_external_torque_callback, 
                1,
            )

    def _gripper_external_torque_callback(self, data):
        gripper_external_torque = data.position[0]
        self.gripper_external_torque = self.gripper_torque_ema_beta * self.gripper_external_torque + \
            (1-self.gripper_torque_ema_beta) * gripper_external_torque
        
    def get_leader_gripper_feedback(self):
        return self.gripper_external_torque
    
    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0*gripper_feedback / self.gripper_feedback_gain
        return torque_gripper
    
    def get_leader_arm_external_joint_torque(self):
        external_torque = self.franka_torque_sub.message
        if external_torque is None:
            # 如果没有收到力矩数据，返回全零数组
            external_torque = np.zeros(7, dtype=np.float64)
            self.get_logger().debug("No torque data received from ZMQ, returning zeros")
        else:
            self.get_logger().debug(f"Received torque data: {external_torque.shape}")
        # 将 Franka 的外部关节力矩转发到 ROS
        # self.obs_franka_torque_pub.publish(create_array_msg(external_torque))
        return external_torque

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        # 应用关节偏移量：follower_joint_pos = leader_joint_pos + joint_offset
        follower_arm_pos = leader_arm_pos + self.joint_offset
        
        # 将映射后的 follower 关节位置发送为位置目标（ZMQ 发布）
        self.franka_cmd_pub.send_message(follower_arm_pos)
        # 将目标位置命令也转发为 ROS 消息以便记录/调试（发送映射后的位置）
        self.cmd_franka_pos_pub.publish(create_array_msg(follower_arm_pos))

        # 发送夹爪目标到 follower
        self.cmd_gripper_pos_pub.publish(create_array_msg([leader_gripper_pos]))

        # 将当前 Franka 跟随臂的关节状态转为 ROS 消息（用于行为克隆与数据采集）
        franka_state = self.franka_joint_state_sub.message
        if franka_state is not None:
            self.obs_franka_state_pub.publish(create_array_msg(franka_state))
        else:
            self.get_logger().debug("No Franka state data received from ZMQ")
        

def main(args=None):
    rclpy.init(args=args)
    factr_teleop_franka_zmq = FACTRTeleopFrankaZMQ()

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_franka_zmq)
    except KeyboardInterrupt:
        factr_teleop_franka_zmq.get_logger().info("Keyboard interrupt received. Shutting down...")
        try:
            factr_teleop_franka_zmq.shut_down()
        except Exception as e:
            factr_teleop_franka_zmq.get_logger().warning(f"关闭过程中出现异常: {e}")
    except Exception as e:
        factr_teleop_franka_zmq.get_logger().error(f"运行时出现异常: {e}")
        try:
            factr_teleop_franka_zmq.shut_down()
        except Exception as shutdown_e:
            factr_teleop_franka_zmq.get_logger().warning(f"紧急关闭过程中出现异常: {shutdown_e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

