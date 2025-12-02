#!/usr/bin/env python3
"""
Translate ROS gripper command topic -> ZMQ gripper command binary messages.
Usage:
  python3 scripts/gripper_cmd_translator.py --name right --zmq tcp://127.0.0.1:2108 --speed 0.05

Behavior:
- Subscribes to ROS topic `/factr_teleop/{name}/cmd_gripper_pos` (sensor_msgs/JointState)
- On message, reads first position as desired gripper width and sends a ZMQ binary message:
  [cmd, params...] as numpy array (float32 or float64). Default message for position is [0, width, speed]
  where cmd=0 means "move" (width, speed).

This lets FACTR publish gripper targets as a ROS topic while the bridge listens for ZMQ messages.
"""

import argparse
import time
import numpy as np
import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperTranslatorNode(Node):
    def __init__(self, name: str, zmq_addr: str, speed: float, dtype: str):
        super().__init__(f'gripper_cmd_translator_{name}')
        self.name = name
        self.zmq_addr = zmq_addr
        self.speed = float(speed)
        self.dtype = np.float32 if dtype == 'float32' else np.float64

        # ZMQ publisher (connect to bridge's gripper SUB)
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        # set linger small so shutdown is quick
        self.sock.setsockopt(zmq.LINGER, 100)
        self.get_logger().info(f"Connecting ZMQ PUB to {self.zmq_addr}")
        self.sock.connect(self.zmq_addr)
        # small sleep to allow connect to establish
        time.sleep(0.1)

        topic = f'/factr_teleop/{self.name}/cmd_gripper_pos'
        self.sub = self.create_subscription(JointState, topic, self._cb, 1)
        self.get_logger().info(f"Subscribed to ROS topic {topic}")

    def _cb(self, msg: JointState):
        if not msg.position:
            self.get_logger().warning('Received empty JointState for gripper command')
            return
        width = float(msg.position[0])
        # Construct command: cmd=0 (move), width, speed
        arr = np.array([0, width, self.speed], dtype=self.dtype)
        try:
            self.sock.send(arr.tobytes())
            self.get_logger().info(f"Sent gripper move cmd: width={width:.4f} speed={self.speed:.4f} dtype={self.dtype}")
        except Exception as e:
            self.get_logger().error(f"Failed to send ZMQ gripper cmd: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='right', help='robot name used in topic (left/right)')
    parser.add_argument('--zmq', default='tcp://127.0.0.1:2108', help='ZMQ address to connect to (bridge gripper cmd)')
    parser.add_argument('--speed', type=float, default=0.05, help='default gripper speed when sending move')
    parser.add_argument('--dtype', choices=['float32', 'float64'], default='float32', help='numeric dtype to send')
    args = parser.parse_args()

    rclpy.init()
    node = GripperTranslatorNode(args.name, args.zmq, args.speed, args.dtype)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down translator')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
