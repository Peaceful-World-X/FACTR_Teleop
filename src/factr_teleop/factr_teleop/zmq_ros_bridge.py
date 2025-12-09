#!/usr/bin/env python3
"""
ZMQ to ROS 2 Bridge Node
将 Franka Bridge (ZMQ) 的数据转发为 ROS 2 话题。
"""

import rclpy
from rclpy.node import Node
import zmq
import numpy as np
import threading
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ZMQ 配置 (需与 franka_factr_bridge_franky.py 保持一致)
STATE_SUB_ADDRESS = "tcp://127.0.0.1:3099"
TORQUE_SUB_ADDRESS = "tcp://127.0.0.1:3087"
POSE_SUB_ADDRESS = "tcp://127.0.0.1:3098"

class ZmqRosBridge(Node):
    def __init__(self):
        super().__init__('zmq_ros_bridge')
        
        self.running = True
        self.zmq_context = zmq.Context()
        
        # --- ZMQ 订阅者 ---
        
        # 1. 状态 (q, dq)
        self.state_sub = self.zmq_context.socket(zmq.SUB)
        self.state_sub.connect(STATE_SUB_ADDRESS)
        self.state_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.state_sub.setsockopt(zmq.CONFLATE, 1)
        
        # 2. 力矩 (tau)
        self.torque_sub = self.zmq_context.socket(zmq.SUB)
        self.torque_sub.connect(TORQUE_SUB_ADDRESS)
        self.torque_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.torque_sub.setsockopt(zmq.CONFLATE, 1)
        
        # 3. 末端位姿 (pose)
        self.pose_sub = self.zmq_context.socket(zmq.SUB)
        self.pose_sub.connect(POSE_SUB_ADDRESS)
        self.pose_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.pose_sub.setsockopt(zmq.CONFLATE, 1)
        
        self.get_logger().info(f"ZMQ Subscribed: State={STATE_SUB_ADDRESS}, Torque={TORQUE_SUB_ADDRESS}, Pose={POSE_SUB_ADDRESS}")

        # --- ROS 发布者 ---
        
        # /franka/joint_states: 包含 position(q), velocity(dq), effort(tau)
        self.joint_pub = self.create_publisher(JointState, '/franka/joint_states', 10)
        
        # /franka/end_effector_pose: 4x4 矩阵展平
        self.pose_pub = self.create_publisher(Float64MultiArray, '/franka/end_effector_pose', 10)

        # /franka/right/obs_franka_torque: 仅发布 effort (7 维)
        self.torque_pub = self.create_publisher(JointState, '/franka/right/obs_franka_torque', 10)
        
        # 启动接收线程
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()

    def receive_loop(self):
        """循环接收 ZMQ 数据并发布 ROS 消息"""
        poller = zmq.Poller()
        poller.register(self.state_sub, zmq.POLLIN)
        poller.register(self.torque_sub, zmq.POLLIN)
        poller.register(self.pose_sub, zmq.POLLIN)
        
        # 缓存数据以组合成一个 JointState
        current_q = None
        current_dq = None
        current_tau = None
        
        while self.running and rclpy.ok():
            try:
                # 检查 ZMQ context 是否仍然有效
                try:
                    socks = dict(poller.poll(timeout=100)) # 100ms timeout
                except zmq.ZMQError as poll_e:
                    if "Context was terminated" in str(poll_e):
                        self.get_logger().info("ZMQ context 已终止，停止接收循环")
                        break
                    else:
                        raise poll_e
                
                # 1. 处理状态 (q, dq)
                if self.state_sub in socks:
                    msg = self.state_sub.recv(zmq.NOBLOCK)
                    # 14个float32: 7 pos + 7 vel
                    if len(msg) == 14 * 4: 
                        data = np.frombuffer(msg, dtype=np.float32)
                        current_q = data[:7]
                        current_dq = data[7:]
                
                # 2. 处理力矩 (tau)
                if self.torque_sub in socks:
                    msg = self.torque_sub.recv(zmq.NOBLOCK)
                    # 7个float32
                    if len(msg) == 7 * 4:
                        current_tau = np.frombuffer(msg, dtype=np.float32)
                
                # 3. 处理位姿 (pose)
                if self.pose_sub in socks:
                    msg = self.pose_sub.recv(zmq.NOBLOCK)
                    # 16个float64 (4x4 matrix, col-major)
                    if len(msg) == 16 * 8:
                        pose_mat = np.frombuffer(msg, dtype=np.float64)
                        
                        ros_msg = Float64MultiArray()
                        ros_msg.data = pose_mat.tolist()
                        self.pose_pub.publish(ros_msg)

                # 4. 如果凑齐了关节数据，发布 JointState
                if current_q is not None and current_dq is not None:
                    # 如果没有收到力矩，补零
                    if current_tau is None:
                        current_tau = np.zeros(7, dtype=np.float32)
                        
                    joint_msg = JointState()
                    joint_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_msg.name = [f"panda_joint{i+1}" for i in range(7)]
                    joint_msg.position = current_q.astype(float).tolist()
                    joint_msg.velocity = current_dq.astype(float).tolist()
                    joint_msg.effort = current_tau.astype(float).tolist()
                    
                    self.joint_pub.publish(joint_msg)

                    # 单独发布力矩到 /franka/right/obs_franka_torque，供 ROS teleop 节点使用
                    torque_msg = JointState()
                    torque_msg.header.stamp = joint_msg.header.stamp
                    torque_msg.effort = current_tau.astype(float).tolist()
                    self.torque_pub.publish(torque_msg)
                    
                    # 清空缓存（或保留最新值？通常保留最新值更稳健，这里简单起见清空，
                    # 但考虑到 ZMQ 频率可能不同步，保留最新值可能更好。
                    # 这里改为：不强制清空，每次循环只要有数据就发，
                    # 实际频率取决于 ZMQ 发送频率 ~100Hz）
                    
            except zmq.ZMQError as zmq_e:
                if "Context was terminated" in str(zmq_e):
                    self.get_logger().info("ZMQ context 已终止，正常退出接收循环")
                    break
                else:
                    self.get_logger().error(f"ZMQ 错误: {zmq_e}")
                    time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Error in bridge loop: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        self.get_logger().info("正在关闭 ZMQ-ROS 桥接节点...")
        self.running = False
        # 等待一小段时间让接收循环自然结束
        time.sleep(0.2)
        try:
            self.zmq_context.term()
            self.get_logger().info("ZMQ context 已终止")
        except Exception as e:
            self.get_logger().warning(f"终止 ZMQ context 时出错: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZmqRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
