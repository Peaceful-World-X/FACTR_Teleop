#!/usr/bin/env python3
"""
Franka-FACTR Bridge using franky library
使用 franky Python 库替代 C++ 桥接，保持与 FACTR 的 ZMQ 通信
"""

import zmq
import numpy as np
import threading
import time
import signal
import sys
from typing import Optional

try:
    from franky import Robot, JointMotion, ReferenceType
    FRANKY_AVAILABLE = True
except ImportError as e:
    print("ERROR: franky library not installed!")
    print(f"Import error: {e}")
    FRANKY_AVAILABLE = False
    sys.exit(1)


# 配置参数
ROBOT_IP = "10.0.10.2"  # Franka 机器人 IP
CMD_SUB_ADDRESS = "tcp://192.168.40.200:2098"  # 接收命令
STATE_PUB_ADDRESS = "tcp://*:3099"  # 发布状态
TORQUE_PUB_ADDRESS = "tcp://*:3087"  # 发布力矩

# 初始位置（程序启动时机械臂会先移动到此位置）
INITIAL_POSITION = np.array([
    0.00461679,   # q[0]
    0.00581697,   # q[1]
    -0.00479847,  # q[2]
    -1.56943803,  # q[3]
    0.00456447,   # q[4]
    1.56894485,   # q[5]
    1.56894485    # q[6]
], dtype=np.float64)

# 控制参数
CONTROL_FREQUENCY = 100.0  # Hz (状态读取频率)
STATE_PUB_FREQUENCY = 100.0  # Hz


class FrankaFactrBridge:
    """使用 franky 的 Franka-FACTR 桥接类"""
    
    def __init__(self):
        self.running = False
        self.robot: Optional[Robot] = None
        
        # 线程安全的数据共享
        self.lock = threading.Lock()
        self.target_position = np.zeros(7)
        self.current_position = np.zeros(7)
        self.current_velocity = np.zeros(7)
        self.current_torque = np.zeros(7)
        self.has_new_target = False
        self.motion_running = False  # 运动是否正在执行
        self.motion_lock = threading.Lock()  # 运动对象的锁
        
        # ZMQ sockets
        self.cmd_subscriber = None
        self.state_publisher = None
        self.torque_publisher = None
        self.zmq_context = None
        
    def setup_zmq(self):
        """设置 ZMQ 通信"""
        self.zmq_context = zmq.Context()
        
        # 命令接收 (SUB)
        self.cmd_subscriber = self.zmq_context.socket(zmq.SUB)
        self.cmd_subscriber.connect(CMD_SUB_ADDRESS)
        self.cmd_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_subscriber.setsockopt(zmq.CONFLATE, 1)  # 只保留最新消息
        print(f"[ZMQ] Command subscriber connected to {CMD_SUB_ADDRESS}")
        
        # 状态发布 (PUB)
        self.state_publisher = self.zmq_context.socket(zmq.PUB)
        self.state_publisher.bind(STATE_PUB_ADDRESS)
        print(f"[ZMQ] State publisher bound to {STATE_PUB_ADDRESS}")
        
        # 力矩发布 (PUB)
        self.torque_publisher = self.zmq_context.socket(zmq.PUB)
        self.torque_publisher.bind(TORQUE_PUB_ADDRESS)
        print(f"[ZMQ] Torque publisher bound to {TORQUE_PUB_ADDRESS}")
        
        # 等待订阅者连接
        time.sleep(1)
        
    def zmq_command_receiver(self):
        """ZMQ 命令接收线程"""
        print("[ZMQ Receiver] Thread started")
        count = 0
        
        while self.running:
            try:
                # 非阻塞接收
                try:
                    message = self.cmd_subscriber.recv(zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.001)  # 1ms
                    continue
                
                # 解析数据：7个double (56 bytes)
                if len(message) == 7 * 8:
                    data = np.frombuffer(message, dtype=np.float64)
                    
                    with self.lock:
                        self.target_position = data.copy()
                        self.has_new_target = True
                    
                    count += 1
                    if count <= 5 or count % 10 == 0:  # 前5条和每10条打印一次
                        print(f"[ZMQ Receiver] Received {count} commands. Latest: {data}")
                        print(f"  Joint 6: {data[6]:.6f} rad ({np.rad2deg(data[6]):.2f} deg)")
                else:
                    print(f"[ZMQ Receiver] Invalid message size: {len(message)} bytes (expected: {7 * 8})")
                    
            except Exception as e:
                print(f"[ZMQ Receiver] Error: {e}")
                time.sleep(0.1)
                
    def zmq_state_publisher(self):
        """ZMQ 状态发布线程"""
        print("[ZMQ Publisher] Thread started")
        count = 0
        last_print_time = time.time()
        
        while self.running:
            try:
                # 读取当前状态
                with self.lock:
                    positions = self.current_position.copy()
                    velocities = self.current_velocity.copy()
                    torques = self.current_torque.copy()
                
                # 发布状态 (14个float64: 7位置 + 7速度) - 统一使用float64
                state_msg = np.concatenate([positions, velocities]).astype(np.float64)
                self.state_publisher.send(state_msg.tobytes(), zmq.NOBLOCK)
                
                # 发布力矩 (7个float64) - 统一使用float64
                torque_msg = torques.astype(np.float64)
                self.torque_publisher.send(torque_msg.tobytes(), zmq.NOBLOCK)
                
                count += 1
                
                # 每5秒打印一次
                current_time = time.time()
                if current_time - last_print_time >= 5.0:
                    print(f"[ZMQ Publisher] Published {count} messages")
                    print(f"  Positions: {positions}")
                    print(f"  Velocities: {velocities}")
                    print(f"  Torques: {torques}")
                    last_print_time = current_time
                
                time.sleep(1.0 / STATE_PUB_FREQUENCY)
                
            except zmq.Again:
                pass  # 非阻塞发送失败，继续
            except Exception as e:
                print(f"[ZMQ Publisher] Error: {e}")
                time.sleep(0.1)
    
    def control_loop(self):
        """
        机器人控制循环（使用 franky）
        采用"一次Move，持续Update"模式，避免在循环中创建线程
        """
        print("[Control] Starting control loop...")
        
        # 第一步：读取当前状态
        try:
            current_state = self.robot.current_joint_state
            current_q = np.array(current_state.position)
            current_dq = np.array(current_state.velocity)
            
            print(f"[Control] Current joint positions: {current_q}")
            print(f"[Control] Current joint positions (deg): {np.rad2deg(current_q)}")
        except Exception as e:
            print(f"[Control] Failed to read current state: {e}")
            self._recover_from_error()
            return
        
        # 第二步：移动到初始位置（在开始接收ZMQ命令之前）
        print("\n" + "=" * 60)
        print("[Control] Step 1: Moving robot to initial position...")
        print("=" * 60)
        print(f"Target initial position (rad): {INITIAL_POSITION}")
        print(f"Target initial position (deg): {np.rad2deg(INITIAL_POSITION)}")
        
        # 检查是否需要移动（如果已经在初始位置附近，可能不需要移动）
        position_diff = np.abs(current_q - INITIAL_POSITION)
        max_diff = np.max(position_diff)
        print(f"Current position difference from target: max = {max_diff:.6f} rad ({np.rad2deg(max_diff):.4f} deg)")
        
        if max_diff > 0.01:  # 如果差异大于 0.01 rad (约 0.57 度)，则移动
            try:
                print("Moving to initial position...")
                motion = JointMotion(INITIAL_POSITION.tolist())
                self.robot.move(motion)
                print("✓ Robot reached initial position")
                
                # 再次读取状态确认
                final_state = self.robot.current_joint_state
                final_q = np.array(final_state.position)
                print(f"Final joint positions: {final_q}")
                print(f"Final joint positions (deg): {np.rad2deg(final_q)}")
            except Exception as e:
                print(f"[Control] Failed to move to initial position: {e}")
                self._recover_from_error()
                return
        else:
            print("Robot is already close to initial position, skipping movement.")
        
        # 第三步：初始化共享数据
        try:
            initial_state = self.robot.current_joint_state
            initial_q = np.array(initial_state.position)
            initial_dq = np.array(initial_state.velocity)
            with self.lock:
                self.target_position = initial_q.copy()
                self.current_position = initial_q.copy()
                self.current_velocity = initial_dq.copy()
                # franky 的 JointState 不包含力矩信息，暂时设为 0
                self.current_torque = np.zeros(7)
            
            print(f"\n[Control] Initialized shared data with current positions: {initial_q}")
        except Exception as e:
            print(f"[Control] Failed to initialize shared data: {e}")
            self._recover_from_error()
            return
        
        print("\n" + "=" * 60)
        print("[Control] Step 2: Ready to receive ZMQ commands...")
        print("=" * 60 + "\n")
        
        # 启动运动执行线程（只启动一次，持续运行）
        motion_thread = threading.Thread(target=self._motion_executor, daemon=True)
        motion_thread.start()
        
        print("[Control] Motion executor thread started. Waiting for commands...")
        
        try:
            while self.running:
                # 读取当前状态
                try:
                    state = self.robot.current_joint_state
                    with self.lock:
                        self.current_position = np.array(state.position)
                        self.current_velocity = np.array(state.velocity)
                        # franky 的 JointState 不包含力矩信息，暂时设为 0
                        self.current_torque = np.zeros(7)
                except Exception as e:
                    print(f"[Control] State read error: {e}")
                    # 状态读取错误通常不是致命错误，继续运行
                
                time.sleep(1.0 / CONTROL_FREQUENCY)
                
        except KeyboardInterrupt:
            print("[Control] Interrupted")
        except Exception as e:
            print(f"[Control] Fatal error: {e}")
            import traceback
            traceback.print_exc()
            self._recover_from_error()
        finally:
            # 停止机器人
            self.running = False
            try:
                self.robot.stop()
            except:
                pass
    
    def _motion_executor(self):
        """
        运动执行器（在独立线程中持续运行）
        采用"一次Move，持续Update"模式：检查新目标，执行运动
        """
        last_target = None
        
        while self.running:
            try:
                # 检查是否有新目标
                target = None
                with self.lock:
                    if self.has_new_target:
                        # 确保转换为 numpy 数组
                        target = np.array(self.target_position, dtype=np.float64).copy()
                        self.has_new_target = False
                
                # 如果有新目标且与上次不同，执行新运动
                if target is not None:
                    # 如果是第一次（last_target 为 None），或者目标与上次不同
                    if last_target is None or not np.allclose(target, last_target, atol=1e-6):
                        try:
                            # 打印目标位置信息（用于调试）
                            if last_target is None:
                                print(f"[Motion Executor] First target received: {target}")
                                print(f"  Joint 6: {target[6]:.6f} rad ({np.rad2deg(target[6]):.2f} deg)")
                            else:
                                diff = target - last_target
                                print(f"[Motion Executor] New target received (diff from last): {diff}")
                                print(f"  Target Joint 6: {target[6]:.6f} rad ({np.rad2deg(target[6]):.2f} deg)")
                            
                            # 停止当前运动（如果正在运行）
                            with self.motion_lock:
                                if self.motion_running:
                                    try:
                                        self.robot.stop()
                                    except:
                                        pass
                                    self.motion_running = False
                                    time.sleep(0.01)  # 短暂等待，确保停止完成
                            
                            # 创建新运动（直接使用目标位置，让 franky 的 Ruckig OTG 处理平滑）
                            # 注意：不进行手动插值，完全依赖 Ruckig
                            # JointMotion 接受一个列表或数组，包含7个关节位置
                            motion = JointMotion(target.tolist())
                            
                            # 执行运动（阻塞调用，直到运动完成或被中断）
                            with self.motion_lock:
                                self.motion_running = True
                            
                            print(f"[Motion Executor] Executing motion to target...")
                            self.robot.move(motion)
                            print(f"[Motion Executor] Motion completed.")
                            
                            with self.motion_lock:
                                self.motion_running = False
                            
                            last_target = target.copy()
                            
                        except Exception as e:
                            with self.motion_lock:
                                self.motion_running = False
                            
                            # 运动执行错误（可能是被新命令中断，这是正常的）
                            error_str = str(e).lower()
                            if "interrupted" not in error_str and "cancel" not in error_str and "stop" not in error_str:
                                print(f"[Control] Motion execution error: {e}")
                                self._recover_from_error()
                
                # 短暂休眠，避免CPU占用过高
                time.sleep(0.01)  # 100Hz检查频率
                
            except Exception as e:
                print(f"[Control] Motion executor error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _recover_from_error(self):
        """错误恢复：尝试从机器人错误中恢复"""
        print("[Control] Attempting error recovery...")
        try:
            # 尝试自动错误恢复
            if hasattr(self.robot, 'automatic_error_recovery'):
                self.robot.automatic_error_recovery()
            elif hasattr(self.robot, 'recover_from_errors'):
                self.robot.recover_from_errors()
            else:
                # 如果franky没有自动恢复方法，尝试停止并重新连接
                try:
                    self.robot.stop()
                except:
                    pass
                time.sleep(0.5)
        except Exception as e:
            print(f"[Control] Error recovery failed: {e}")
    
    def start(self):
        """启动桥接"""
        print("=" * 60)
        print("Franka-FACTR Bridge (using franky)")
        print("=" * 60)
        
        # 连接机器人
        print(f"[Main] Connecting to robot at {ROBOT_IP}...")
        try:
            self.robot = Robot(ROBOT_IP)
            # 设置相对动力学因子（降低速度和加速度）
            self.robot.relative_dynamics_factor = 0.1
            print("[Main] Robot connected successfully")
        except Exception as e:
            print(f"[Main] Failed to connect to robot: {e}")
            return False
        
        # 设置 ZMQ
        try:
            self.setup_zmq()
        except Exception as e:
            print(f"[Main] Failed to setup ZMQ: {e}")
            return False
        
        # 启动线程
        self.running = True
        
        cmd_thread = threading.Thread(target=self.zmq_command_receiver, daemon=True)
        pub_thread = threading.Thread(target=self.zmq_state_publisher, daemon=True)
        
        cmd_thread.start()
        pub_thread.start()
        
        print("[Main] All threads started. Starting control loop...")
        print("[Main] Press Ctrl+C to exit")
        
        # 运行控制循环（主线程）
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("\n[Main] Shutting down...")
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        """停止桥接"""
        print("[Main] Stopping...")
        self.running = False
        
        # 关闭 ZMQ
        if self.cmd_subscriber:
            self.cmd_subscriber.close()
        if self.state_publisher:
            self.state_publisher.close()
        if self.torque_publisher:
            self.torque_publisher.close()
        if self.zmq_context:
            self.zmq_context.term()
        
        # 停止机器人
        if self.robot:
            try:
                self.robot.stop()
            except:
                pass
        
        print("[Main] Stopped")


def signal_handler(sig, frame):
    """信号处理"""
    print("\n[Signal] Received interrupt signal")
    sys.exit(0)


if __name__ == "__main__":
    if not FRANKY_AVAILABLE:
        sys.exit(1)
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 创建并启动桥接
    bridge = FrankaFactrBridge()
    bridge.start()

