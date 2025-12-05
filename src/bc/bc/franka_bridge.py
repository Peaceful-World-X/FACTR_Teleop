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

import numpy as np
import time
import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bc.utils import create_joint_state_msg
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from python_utils.global_configs import franka_left_real_zmq_addresses
from python_utils.global_configs import franka_right_real_zmq_addresses


class FrankaBridge(Node):  
    """
    This class is used to bridge the Franka arm to the ROS system.
    It subscribes to the Franka arm's joint state and torque topics and publishes the joint state and torque to the ROS system.
    """
    def __init__(self):
        super().__init__('franka_bridge')
        self.torque_feedback = self.declare_parameter('torque_feedback', True).get_parameter_value().bool_value
        self.publish_rate = self.declare_parameter('publish_rate', 300.0).get_parameter_value().double_value
        self.enable_left_arm = self.declare_parameter('enable_left_arm', False).get_parameter_value().bool_value
        self.enable_right_arm = self.declare_parameter('enable_right_arm', True).get_parameter_value().bool_value

        # 初始化左臂（如果启用）
        if self.enable_left_arm:
            left_zmq_addresses = franka_left_real_zmq_addresses
            self.left_franka_cmd_pub = ZMQPublisher(left_zmq_addresses["joint_pos_cmd_pub"])
            self.left_franka_cmd_sub = self.create_subscription(JointState, f'/factr_teleop/left/cmd_franka_pos', self.left_franka_cmd_callback, 10)
            
            self.left_franka_pos_sub = ZMQSubscriber(left_zmq_addresses["joint_state_sub"])
            self.left_franka_pos_pub = self.create_publisher(JointState, f'/franka/left/obs_franka_state', 10)
            
            self.left_franka_torque_sub = ZMQSubscriber(left_zmq_addresses["joint_torque_sub"])
            self.left_franka_torque_pub = self.create_publisher(JointState, f'/franka/left/obs_franka_torque', 10)

            while self.left_franka_torque_sub.message is None:
                time.sleep(0.1)
                self.get_logger().info("Has not received left Franka's torques")
        else:
            self.left_franka_cmd_pub = None
            self.left_franka_pos_sub = None
            self.left_franka_torque_sub = None

        # 初始化右臂（如果启用）
        if self.enable_right_arm:
            right_zmq_addresses = franka_right_real_zmq_addresses
            self.right_franka_cmd_pub = ZMQPublisher(right_zmq_addresses["joint_pos_cmd_pub"])
            self.right_franka_cmd_sub = self.create_subscription(JointState, f'/factr_teleop/right/cmd_franka_pos', self.right_franka_cmd_callback, 10)
            
            self.right_franka_pos_sub = ZMQSubscriber(right_zmq_addresses["joint_state_sub"])
            self.right_franka_pos_pub = self.create_publisher(JointState, f'/franka/right/obs_franka_state', 10)
            
            self.right_franka_torque_sub = ZMQSubscriber(right_zmq_addresses["joint_torque_sub"])
            self.right_franka_torque_pub = self.create_publisher(JointState, f'/franka/right/obs_franka_torque', 10)

            while self.right_franka_torque_sub.message is None:
                time.sleep(0.1)
                self.get_logger().info("Has not received right Franka's torques")
        else:
            self.right_franka_cmd_pub = None
            self.right_franka_pos_sub = None
            self.right_franka_torque_sub = None

        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self.timer_callback)
    
    def timer_callback(self):
        if self.enable_left_arm and self.left_franka_pos_sub and self.left_franka_pos_sub.message is not None:
            self.left_franka_pos_pub.publish(create_joint_state_msg(self.left_franka_pos_sub.message[0:7]))
            if self.torque_feedback and self.left_franka_torque_sub and self.left_franka_torque_sub.message is not None:
                self.left_franka_torque_pub.publish(create_joint_state_msg(self.left_franka_torque_sub.message))
        
        if self.enable_right_arm and self.right_franka_pos_sub and self.right_franka_pos_sub.message is not None:
            self.right_franka_pos_pub.publish(create_joint_state_msg(self.right_franka_pos_sub.message[0:7]))
            if self.torque_feedback and self.right_franka_torque_sub and self.right_franka_torque_sub.message is not None:
                self.right_franka_torque_pub.publish(create_joint_state_msg(self.right_franka_torque_sub.message))

    def left_franka_cmd_callback(self, msg):
        if self.enable_left_arm and self.left_franka_cmd_pub:
            self.left_franka_cmd_pub.send_message(np.array(msg.position))

    def right_franka_cmd_callback(self, msg):
        if self.enable_right_arm and self.right_franka_cmd_pub:
            self.right_franka_cmd_pub.send_message(np.array(msg.position))


def main(args=None):
    rclpy.init(args=args)
    node = FrankaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()