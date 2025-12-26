#!/usr/bin/env python3
"""
Franka机械臂动作回放功能

从采集的data.pkl文件中读取关节角度和夹爪命令序列，
通过ZMQ和ROS接口控制Franka机械臂进行动作回放。

使用方法:
    python action_playback.py --data_path /path/to/data.pkl --speed 1.0
"""

import argparse
import pickle
import time
import signal
import sys
import threading
import select
import tty
import termios
from pathlib import Path
from typing import Dict, List, Optional, Any
import logging

import numpy as np
import zmq

# ROS 2 相关导入
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
except ImportError:
    print("警告: ROS 2 未安装，将无法控制夹爪")
    rclpy = None
    Node = None
    JointState = None
    Float64MultiArray = None

# 配置常量 - 仿照现有bridge程序的地址配置
DEFAULT_ZMQ_CMD_ADDRESS = "tcp://127.0.0.1:2098"  # Franka命令地址 
DEFAULT_ROS_GRIPPER_TOPIC = "/factr_teleop/right/cmd_gripper_pos"  # 夹爪命令话题
DEFAULT_PLAYBACK_SPEED = 1.0  # 回放速度倍数
DEFAULT_FRAME_RATE = 30.0  # 固定帧率 (Hz)

# Franka关节限制
JOINT_LIMITS = np.array([
    [-2.8973, 2.8973],    # joint 1
    [-1.7628, 1.7628],    # joint 2
    [-2.8973, 2.8973],    # joint 3
    [-3.0718, -0.0698],   # joint 4
    [-2.8973, 2.8973],    # joint 5
    [-0.0175, 3.7525],    # joint 6
    [-1.48, 3.0718]       # joint 7
])

class ActionPlayback:
    """Franka机械臂动作回放类"""

    def __init__(self, data_path: str, speed: float = DEFAULT_PLAYBACK_SPEED,
                 zmq_address: str = DEFAULT_ZMQ_CMD_ADDRESS,
                 gripper_topic: str = DEFAULT_ROS_GRIPPER_TOPIC):
        """
        初始化回放器

        Args:
            data_path: data.pkl文件路径
            speed: 回放速度倍数 (1.0为原始速度)
            zmq_address: ZMQ命令发布地址
            gripper_topic: ROS夹爪命令话题
        """
        self.data_path = Path(data_path)
        self.speed = speed
        self.zmq_address = zmq_address
        self.gripper_topic = gripper_topic

        # 状态变量
        self.episode_data = []
        self.joint_positions = []  # 关节位置序列
        self.gripper_commands = []  # 夹爪命令序列
        self.timestamps = []  # 时间戳序列

        self.running = False
        self.paused = False
        self.current_frame = 0

        # ZMQ相关
        self.zmq_context = None
        self.cmd_publisher = None

        # ROS相关
        self.ros_node = None
        self.gripper_publisher = None

        # 日志
        self.logger = logging.getLogger("ActionPlayback")
        self.logger.setLevel(logging.INFO)

        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)

    def load_data(self) -> bool:
        """加载并解析data.pkl文件"""
        try:
            if not self.data_path.exists():
                self.logger.error(f"数据文件不存在: {self.data_path}")
                return False

            with open(self.data_path, 'rb') as f:
                self.episode_data = pickle.load(f)

            if not self.episode_data:
                self.logger.error("数据文件为空")
                return False

            self.logger.info(f"成功加载 {len(self.episode_data)} 帧数据")

            # 解析数据
            return self._parse_data()

        except Exception as e:
            self.logger.error(f"加载数据失败: {e}")
            return False

    def _parse_data(self) -> bool:
        """解析数据，提取关节位置和夹爪命令

        专门查找：
        - 关节位置：/factr_teleop/right/cmd_franka_pos
        - 夹爪命令：leader_gripper
        - 时间戳：基于固定30Hz帧率生成
        """
        try:
            self.joint_positions = []
            self.gripper_commands = []
            self.timestamps = []

            for frame_idx, frame_data in enumerate(self.episode_data):
                # 提取关节位置 - 专门查找 /factr_teleop/right/cmd_franka_pos
                joint_pos = self._extract_cmd_franka_pos(frame_data)
                if joint_pos is not None:
                    self.joint_positions.append(joint_pos)

                    # 生成固定30Hz时间戳
                    timestamp = frame_idx / DEFAULT_FRAME_RATE
                    self.timestamps.append(timestamp)

                    # 提取夹爪命令 - 查找 leader_gripper
                    gripper_cmd = self._extract_leader_gripper(frame_data)
                    self.gripper_commands.append(gripper_cmd if gripper_cmd is not None else 0.0)

                    self.logger.debug(f"帧 {frame_idx}: 关节={joint_pos[:3]}..., 夹爪={gripper_cmd}")
                else:
                    self.logger.warning(f"帧 {frame_idx} 未找到关节位置数据，跳过")

            if not self.joint_positions:
                self.logger.error("未找到有效的关节位置数据 (/factr_teleop/right/cmd_franka_pos)")
                return False

            self.logger.info(f"解析完成: {len(self.joint_positions)} 帧关节数据, {len(self.gripper_commands)} 帧夹爪数据")
            self.logger.info(f"总时长: {self.timestamps[-1]:.2f} 秒 (30Hz)")
            return True

        except Exception as e:
            self.logger.error(f"解析数据失败: {e}")
            return False

    def _extract_cmd_franka_pos(self, frame_data: Dict) -> Optional[np.ndarray]:
        """从帧数据中提取关节位置命令 - 专门查找 /factr_teleop/right/cmd_franka_pos"""
        # 直接查找指定的topic
        target_topic = "/factr_teleop/right/cmd_franka_pos"

        if target_topic in frame_data:
            data = frame_data[target_topic]
            if isinstance(data, dict) and 'position' in data:
                position = data['position']
                if isinstance(position, np.ndarray) and len(position) == 7:
                    # 检查关节限制
                    if self._check_joint_limits(position):
                        return position.astype(np.float64)
            elif isinstance(data, np.ndarray) and len(data) == 7:
                if self._check_joint_limits(data):
                    return data.astype(np.float64)

        # 如果没找到，记录警告
        if target_topic not in frame_data:
            self.logger.debug(f"未找到 {target_topic}，可用topics: {list(frame_data.keys())}")

        return None

    def _extract_leader_gripper(self, frame_data: Dict) -> Optional[float]:
        """从帧数据中提取夹爪命令 - 专门查找 leader_gripper

        pkl文件中的leader_gripper是0-1范围（0=完全关闭，1=完全张开）
        需要映射到0-0.8弧度范围（发送给gripper_bridge的格式）
        """
        # 直接查找 leader_gripper
        if 'leader_gripper' in frame_data:
            gripper_value = frame_data['leader_gripper']
            if isinstance(gripper_value, (int, float)):
                raw_value = float(gripper_value)
            elif isinstance(gripper_value, np.ndarray) and len(gripper_value) >= 1:
                raw_value = float(gripper_value[0])
            else:
                return None

            # 映射 0-1 范围到 0-0.8 弧度范围
            mapped_value = raw_value * 0.8
            # 确保在有效范围内
            mapped_value = max(0.0, min(0.8, mapped_value))

            self.logger.debug(f"夹爪映射: {raw_value} -> {mapped_value} 弧度")
            return mapped_value

        # 如果没找到，记录调试信息
        self.logger.debug(f"未找到 leader_gripper，可用keys: {list(frame_data.keys())}")
        return None

    def _check_joint_limits(self, positions: np.ndarray) -> bool:
        """检查关节位置是否在安全范围内"""
        positions = np.array(positions)
        within_limits = np.all((positions >= JOINT_LIMITS[:, 0]) & (positions <= JOINT_LIMITS[:, 1]))

        if not within_limits:
            self.logger.warning(f"关节位置超出限制: {positions}")
            # 进行裁剪
            positions = np.clip(positions, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            self.logger.info(f"已裁剪到安全范围: {positions}")

        return True  # 总是返回True，因为我们会裁剪

    def setup_zmq(self) -> bool:
        try:
            self.zmq_context = zmq.Context()
            self.cmd_publisher = self.zmq_context.socket(zmq.PUB)
            self.cmd_publisher.bind(self.zmq_address)  # 连接到 franka_bridge 的绑定地址
            self.logger.info(f"ZMQ发布器绑定到: {self.zmq_address}")
            time.sleep(1)  # 等待连接建立，防止消息丢失
            return True
        except Exception as e:
            self.logger.error(f"ZMQ设置失败: {e}")
            return False

    def setup_ros(self) -> bool:
        """设置ROS节点和发布器"""
        if rclpy is None:
            self.logger.warning("ROS 2未安装，跳过ROS设置")
            return True

        try:
            rclpy.init()
            self.ros_node = Node('action_playback')

            self.gripper_publisher = self.ros_node.create_publisher(
                JointState,
                self.gripper_topic,
                10
            )

            print(f"DEBUG: ROS初始化成功 - 节点: action_playback, 话题: {self.gripper_topic}")
            self.logger.info(f"ROS发布器创建: {self.gripper_topic}")
            return True

        except Exception as e:
            self.logger.error(f"ROS设置失败: {e}")
            return False

    def start_playback(self, interactive: bool = False) -> bool:
        """开始回放

        Args:
            interactive: 是否启用交互式控制（支持暂停/恢复）
        """
        if not self.episode_data:
            self.logger.error("没有加载数据，无法开始回放")
            return False

        if not self.setup_zmq():
            return False

        if not self.setup_ros():
            return False

        self.running = True
        self.paused = False
        self.current_frame = 0

        self.logger.info(f"开始回放，共 {len(self.joint_positions)} 帧，速度: {self.speed}x")

        if interactive:
            self.logger.info("交互模式：按 'p' 暂停/恢复，'q' 退出")

        try:
            if interactive:
                self._interactive_playback_loop()
            else:
                self._playback_loop()
            return True

        except Exception as e:
            self.logger.error(f"回放过程中出错: {e}")
            return False

        finally:
            self.cleanup()

    def _playback_loop(self):
        """主回放循环 - 基于固定30Hz帧率"""
        start_time = time.time()
        frame_interval = (1.0 / DEFAULT_FRAME_RATE) / self.speed  # 考虑速度倍数

        while self.running and self.current_frame < len(self.joint_positions):
            if self.paused:
                time.sleep(0.1)
                continue

            frame_start_time = time.time()

            # 发送当前帧的命令
            self._send_commands(self.current_frame)

            # 进度报告
            if self.current_frame % 10 == 0:
                progress = (self.current_frame + 1) / len(self.joint_positions) * 100
                elapsed = time.time() - start_time
                self.logger.info(f"回放进度: {self.current_frame + 1}/{len(self.joint_positions)} ({progress:.1f}%) - 经过时间: {elapsed:.2f}s")

            self.current_frame += 1

            # 等待到下一帧的时间（固定30Hz）
            frame_end_time = time.time()
            actual_frame_time = frame_end_time - frame_start_time
            if actual_frame_time < frame_interval:
                sleep_time = frame_interval - actual_frame_time
                time.sleep(min(sleep_time, 0.1))  # 最多睡100ms，避免完全阻塞

        self.logger.info("回放完成")

    def _interactive_playback_loop(self):
        """交互式回放循环，支持键盘控制 - 基于固定30Hz帧率"""
        # 保存终端设置
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            # 设置终端为原始模式
            tty.setcbreak(sys.stdin.fileno())

            start_time = time.time()
            frame_interval = (1.0 / DEFAULT_FRAME_RATE) / self.speed  # 考虑速度倍数
            self.logger.info("开始交互式回放...")

            while self.running and self.current_frame < len(self.joint_positions):
                frame_start_time = time.time()

                # 检查键盘输入
                if self._check_keyboard_input():
                    break

                if self.paused:
                    time.sleep(0.1)
                    continue

                # 发送当前帧的命令
                self._send_commands(self.current_frame)

                # 进度报告
                if self.current_frame % 10 == 0:
                    progress = (self.current_frame + 1) / len(self.joint_positions) * 100
                    elapsed = time.time() - start_time
                    self.logger.info(f"回放进度: {self.current_frame + 1}/{len(self.joint_positions)} ({progress:.1f}%) - 经过时间: {elapsed:.2f}s")

                self.current_frame += 1

                # 等待到下一帧的时间（固定30Hz），同时检查键盘输入
                frame_end_time = time.time()
                actual_frame_time = frame_end_time - frame_start_time
                if actual_frame_time < frame_interval:
                    remaining_time = frame_interval - actual_frame_time
                    if self._wait_with_keyboard_check(min(remaining_time, 0.1)):
                        break

            if self.current_frame >= len(self.joint_positions):
                self.logger.info("回放完成")
            else:
                self.logger.info("回放被用户中断")

        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _check_keyboard_input(self) -> bool:
        """检查键盘输入，返回是否应该退出"""
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == 'p' or key == 'P':
                    if self.paused:
                        self.resume()
                    else:
                        self.pause()
                elif key == 'q' or key == 'Q':
                    self.logger.info("用户请求退出")
                    return True
        except:
            pass
        return False

    def _wait_with_keyboard_check(self, wait_time: float) -> bool:
        """等待指定时间，同时检查键盘输入"""
        start_time = time.time()
        while time.time() - start_time < wait_time:
            if self._check_keyboard_input():
                return True
            time.sleep(0.01)  # 10ms 检查一次
        return False

    def _send_commands(self, frame_idx: int):
        """发送指定帧的命令"""
        # 发送关节命令
        if frame_idx < len(self.joint_positions):
            joint_pos = self.joint_positions[frame_idx]
            self._send_joint_command(joint_pos)

        # 发送夹爪命令
        if frame_idx < len(self.gripper_commands):
            gripper_cmd = self.gripper_commands[frame_idx]
            self._send_gripper_command(gripper_cmd)

    def _send_joint_command(self, joint_positions: np.ndarray):
        """通过ZMQ发送关节命令"""
        try:
            # 发送7个关节角度 (float64)
            msg = joint_positions.astype(np.float64).tobytes()
            self.cmd_publisher.send(msg)
            self.logger.debug(f"发送关节命令: {joint_positions}")

        except Exception as e:
            self.logger.error(f"发送关节命令失败: {e}")

    def _send_gripper_command(self, gripper_position: float):
        """通过ROS发送夹爪命令"""
        if self.ros_node is None or self.gripper_publisher is None:
            self.logger.warning("ROS节点或发布器未初始化")
            return

        try:
            msg = JointState()
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg.name = ['gripper_joint']
            msg.position = [float(gripper_position)]
            msg.velocity = [0.0]
            msg.effort = [0.0]

            self.gripper_publisher.publish(msg)
            self.logger.debug(f"发送夹爪命令: {gripper_position}")

        except Exception as e:
            self.logger.error(f"发送夹爪命令失败: {e}")

    def pause(self):
        """暂停回放"""
        self.paused = True
        self.logger.info("回放已暂停")

    def resume(self):
        """恢复回放"""
        self.paused = False
        self.logger.info("回放已恢复")

    def stop(self):
        """停止回放"""
        self.running = False
        self.logger.info("回放已停止")

    def cleanup(self):
        """清理资源"""
        self.running = False

        if self.zmq_context:
            try:
                self.zmq_context.term()
            except:
                pass

        if rclpy and rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass

    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在停止...")
        self.stop()

    def get_info(self) -> Dict[str, Any]:
        """获取回放信息"""
        return {
            'data_path': str(self.data_path),
            'total_frames': len(self.joint_positions),
            'current_frame': self.current_frame,
            'speed': self.speed,
            'running': self.running,
            'paused': self.paused,
            'duration': self.timestamps[-1] if self.timestamps else 0.0
        }


def main():
    parser = argparse.ArgumentParser(description='Franka机械臂动作回放')
    parser.add_argument('--data_path', '-d', required=True, help='data.pkl文件路径')
    parser.add_argument('--speed', '-s', type=float, default=DEFAULT_PLAYBACK_SPEED,
                       help=f'回放速度倍数 (默认: {DEFAULT_PLAYBACK_SPEED})')
    parser.add_argument('--zmq_address', '-z', default=DEFAULT_ZMQ_CMD_ADDRESS,
                       help=f'ZMQ命令地址 (默认: {DEFAULT_ZMQ_CMD_ADDRESS})')
    parser.add_argument('--gripper_topic', '-g', default=DEFAULT_ROS_GRIPPER_TOPIC,
                       help=f'ROS夹爪话题 (默认: {DEFAULT_ROS_GRIPPER_TOPIC})')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='启用交互模式（支持键盘控制：p-暂停/恢复，q-退出）')

    args = parser.parse_args()

    # 设置日志
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='[%(levelname)s] %(message)s'
    )

    # 创建回放器
    playback = ActionPlayback(
        data_path=args.data_path,
        speed=args.speed,
        zmq_address=args.zmq_address,
        gripper_topic=args.gripper_topic
    )

    # 加载数据
    if not playback.load_data():
        sys.exit(1)

    # 开始回放
    success = playback.start_playback(interactive=args.interactive)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()