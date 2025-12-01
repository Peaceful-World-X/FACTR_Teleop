#!/usr/bin/env python3
"""Franka ↔ FACTR 桥接程序

实时桥接 Franka Emika Panda 机器人与 FACTR 遥操作系统。
通过 ZMQ 提供双向通信，适用于高频率（例如 500Hz）轨迹流传输。
"""

import logging
import signal
import sys
import threading
import time
from typing import Optional

import numpy as np
import zmq

try:
    from franky import Robot, JointMotion
except ImportError as e:
    print(f"ERROR: franky library not installed: {e}")
    sys.exit(1)

# Configuration
ROBOT_IP = "10.0.10.2"
CMD_SUB_ADDRESS = "tcp://127.0.0.1:2098"
STATE_PUB_ADDRESS = "tcp://127.0.0.1:3099"
TORQUE_PUB_ADDRESS = "tcp://127.0.0.1:3087"

INITIAL_POSITION = np.array([
    0.00461679, 0.00581697, -0.00479847, -1.56943803,
    0.00456447, 1.56894485, 1.56894485
], dtype=np.float64)

RELATIVE_DYNAMICS_FACTOR = 0.2
STATE_PUB_FREQUENCY = 100.0  # Hz
ZMQ_CONNECTION_WAIT_TIME = 2.0  # seconds

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)


class FrankaFactrBridge:
    """Franka 与 FACTR 之间的桥接类。"""

    def __init__(self):
        self.running = False
        self.robot: Optional[Robot] = None
        self.zmq_context = None
        self.cmd_subscriber = None
        self.state_publisher = None
        self.torque_publisher = None

    def setup_zmq(self):
        """初始化 ZMQ 通信套接字。"""
        self.zmq_context = zmq.Context()

        # 命令订阅器：接收来自 FACTR 的关节位置命令
        self.cmd_subscriber = self.zmq_context.socket(zmq.SUB)
        self.cmd_subscriber.connect(CMD_SUB_ADDRESS)
        self.cmd_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_subscriber.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message

        # 状态发布器：发布关节位置和速度
        self.state_publisher = self.zmq_context.socket(zmq.PUB)
        self.state_publisher.bind(STATE_PUB_ADDRESS)

        # 力矩发布器：发布外部关节力矩
        self.torque_publisher = self.zmq_context.socket(zmq.PUB)
        self.torque_publisher.bind(TORQUE_PUB_ADDRESS)

        logger.info(f"ZMQ initialized: CMD={CMD_SUB_ADDRESS}, "
                   f"State={STATE_PUB_ADDRESS}, Torque={TORQUE_PUB_ADDRESS}")
        time.sleep(ZMQ_CONNECTION_WAIT_TIME)  # Wait for subscribers to connect

    def _get_robot_state(self):
        """从机器人状态中提取关节位置、速度和力矩。"""
        state = self.robot.state

    # 获取关节位置与速度
        q = np.array(getattr(state, 'q', []))
        dq = np.array(getattr(state, 'dq', []))

        if len(q) == 0:
            js = self.robot.current_joint_state
            q = np.array(js.position)
            dq = np.array(js.velocity)

        # 获取外部力矩（优先使用滤波后的值，若不存在则回退到测量值）
        if hasattr(state, 'tau_ext_hat_filtered'):
            tau = np.array(state.tau_ext_hat_filtered)
        elif hasattr(state, 'tau_J'):
            tau = np.array(state.tau_J)
        else:
            tau = np.zeros(7)

        return q, dq, tau

    def state_publisher_loop(self):
        """后台线程：以固定频率发布机器人状态。"""
        period = 1.0 / STATE_PUB_FREQUENCY
        while self.running and self.robot:
            try:
                start_time = time.time()
                q, dq, tau = self._get_robot_state()
                # 发布状态：14 个 float（7 个位置 + 7 个速度）
                state_msg = np.concatenate([q, dq]).astype(np.float32)
                self.state_publisher.send(state_msg.tobytes(), zmq.NOBLOCK)
                # 发布力矩：7 个 float
                torque_msg = tau.astype(np.float32)
                self.torque_publisher.send(torque_msg.tobytes(), zmq.NOBLOCK)

                # Frequency control
                elapsed = time.time() - start_time
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                logger.warning(f"State publisher error: {e}")
                time.sleep(0.1)

    def control_loop(self):
        """主控制循环：接收命令并执行异步运动。"""
        # Move to initial position
        try:
            init_motion = JointMotion(INITIAL_POSITION.tolist())
            self.robot.move(init_motion)
            logger.info("Reached initial position")
        except Exception as e:
            logger.error(f"Failed to reach initial position: {e}")
            return

        logger.info(f"Control loop started (dynamics factor: {RELATIVE_DYNAMICS_FACTOR})")

        # 清空 ZMQ 缓冲区（消费掉任何遗留的命令）
        try:
            while True:
                self.cmd_subscriber.recv(zmq.NOBLOCK)
        except zmq.Again:
            pass

        # Main control loop
        while self.running:
            try:
                try:
                    message = self.cmd_subscriber.recv(zmq.NOBLOCK)
                    # 这里期望接收到 7 个 double（8 字节/项）
                    if len(message) == 7 * 8:  # 7 个 double（56 字节）
                        target = np.frombuffer(message, dtype=np.float64)
                        motion = JointMotion(target.tolist())
                        # 异步下发控制点，底层会平滑执行
                        self.robot.move(motion, asynchronous=True)
                except zmq.Again:
                    # 没有新命令时保持当前运动
                    pass
            except Exception as e:
                logger.error(f"Motion error: {e}")
                # Attempt error recovery
                try:
                    if hasattr(self.robot, 'automatic_error_recovery'):
                        self.robot.automatic_error_recovery()
                    elif hasattr(self.robot, 'recover_from_errors'):
                        self.robot.recover_from_errors()
                except Exception as rec_e:
                    logger.warning(f"Recovery failed: {rec_e}")
                time.sleep(0.5)

    def start(self):
        """初始化并启动桥接程序。"""
        logger.info("Initializing Franka-FACTR Bridge")

        # Connect to robot
        try:
            self.robot = Robot(ROBOT_IP)
            self.robot.relative_dynamics_factor = RELATIVE_DYNAMICS_FACTOR
            logger.info(f"Connected to robot at {ROBOT_IP}")
        except Exception as e:
            logger.error(f"Robot connection failed: {e}")
            return

        # 设置 ZMQ
        try:
            self.setup_zmq()
        except Exception as e:
            logger.error(f"ZMQ setup failed: {e}")
            return

        self.running = True

    # 启动状态发布线程
        pub_thread = threading.Thread(target=self.state_publisher_loop, daemon=True)
        pub_thread.start()
        time.sleep(0.5)

        # Enter control loop
        try:
            self.control_loop()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.stop()

    def stop(self):
        """停止桥接并清理资源。"""
        logger.info("Stopping bridge")
        self.running = False

        if self.zmq_context:
            self.zmq_context.term()

        if self.robot:
            try:
                try:
                    self.robot.join_motion()
                except Exception:
                    pass
                self.robot.stop()
            except Exception as e:
                logger.warning(f"Error stopping robot: {e}")

        logger.info("Bridge stopped")


def signal_handler(sig, frame):
    """SIGINT 信号处理器（Ctrl-C）。"""
    logger.info("收到 SIGINT，正在退出")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    bridge = FrankaFactrBridge()
    bridge.start()
