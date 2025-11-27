#!/usr/bin/env python3
"""
Franka-FACTR Bridge (Asynchronous Mode)
专为 500Hz 实时密集轨迹流设计的接口程序
"""

import zmq
import numpy as np
import threading
import time
import signal
import sys
from typing import Optional

try:
    from franky import Robot, JointMotion
    FRANKY_AVAILABLE = True
except ImportError as e:
    print("ERROR: franky library not installed!")
    print(f"Import error: {e}")
    sys.exit(1)

# ================= 配置参数 =================
ROBOT_IP = "10.0.10.2"          # Franka 机器人 IP
CMD_SUB_ADDRESS = "tcp://192.168.40.200:2098"   # 接收 FACTR 命令 (SUB)
STATE_PUB_ADDRESS = "tcp://*:3099"              # 发布状态 (PUB)
TORQUE_PUB_ADDRESS = "tcp://*:3087"             # 发布力矩 (PUB)

# 初始位置 (用于启动时归位)
INITIAL_POSITION = np.array([
    0.00461679, 0.00581697, -0.00479847, -1.56943803, 
    0.00456447, 1.56894485, 1.56894485
], dtype=np.float64)

# 安全限制
# 建议从 0.2 开始测试，稳定后可逐步提高到 0.5 - 1.0 以减少跟踪延迟
RELATIVE_DYNAMICS_FACTOR = 0.2 

# 状态发布频率
STATE_PUB_FREQUENCY = 100.0  # Hz

class FrankaFactrBridge:
    def __init__(self):
        self.running = False
        self.robot: Optional[Robot] = None
        self.zmq_context = None
        self.cmd_subscriber = None
        self.state_publisher = None
        self.torque_publisher = None
        
        # 共享锁（仅用于保护非原子操作，但在异步架构中依赖较少）
        self.lock = threading.Lock()

    def setup_zmq(self):
        """初始化 ZMQ 通信"""
        self.zmq_context = zmq.Context()
        
        # 1. 命令接收 (SUB)
        self.cmd_subscriber = self.zmq_context.socket(zmq.SUB)
        self.cmd_subscriber.connect(CMD_SUB_ADDRESS)
        self.cmd_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        # CONFLATE=1 是关键：只保留队列中最新的一条消息，丢弃旧消息
        # 保证机器人永远去追最新的目标，而不是执行积压的过期目标
        self.cmd_subscriber.setsockopt(zmq.CONFLATE, 1)
        
        # 2. 状态发布 (PUB)
        self.state_publisher = self.zmq_context.socket(zmq.PUB)
        self.state_publisher.bind(STATE_PUB_ADDRESS)
        
        # 3. 力矩发布 (PUB)
        self.torque_publisher = self.zmq_context.socket(zmq.PUB)
        self.torque_publisher.bind(TORQUE_PUB_ADDRESS)
        
        print(f"[ZMQ] Connected to CMD: {CMD_SUB_ADDRESS}")
        print(f"[ZMQ] Bound PUBs: State={STATE_PUB_ADDRESS}, Torque={TORQUE_PUB_ADDRESS}")
        time.sleep(0.5)

    def state_publisher_loop(self):
        """
        后台线程：以固定频率读取并发布机器人状态
        """
        print("[Pub Thread] Started")
        while self.running and self.robot:
            try:
                start_time = time.time()
                
                # 直接从 robot 对象读取状态快照
                # 注意：libfranka 的状态读取通常是线程安全的
                state = self.robot.state
                
                # --- 1. 获取位置和速度 ---
                # 兼容性处理：不同版本的 franky 可能属性名不同
                q = np.array(getattr(state, 'q', []))
                dq = np.array(getattr(state, 'dq', []))
                
                if len(q) == 0: # 如果读取失败
                    # 尝试用 current_joint_state 备选
                    js = self.robot.current_joint_state
                    q = np.array(js.position)
                    dq = np.array(js.velocity)

                # --- 2. 获取力矩 ---
                # 优先获取滤波后的外部力矩 (用于力控/碰撞检测)
                # 其次是测量力矩 (包含重力)
                if hasattr(state, 'tau_ext_hat_filtered'):
                    tau = np.array(state.tau_ext_hat_filtered)
                elif hasattr(state, 'tau_J'):
                    tau = np.array(state.tau_J)
                else:
                    tau = np.zeros(7)

                # --- 3. 发送 ZMQ ---
                # 状态: 14个 float32 (7 pos + 7 vel)
                state_msg = np.concatenate([q, dq]).astype(np.float32)
                self.state_publisher.send(state_msg.tobytes(), zmq.NOBLOCK)
                
                # 力矩: 7个 float32
                torque_msg = tau.astype(np.float32)
                self.torque_publisher.send(torque_msg.tobytes(), zmq.NOBLOCK)
                
                # --- 4. 频率控制 ---
                elapsed = time.time() - start_time
                sleep_time = (1.0 / STATE_PUB_FREQUENCY) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                # 状态发布错误不应中断主程序，打印即可
                # print(f"[Pub Error] {e}") 
                time.sleep(0.1)

    def control_loop(self):
        """
        主控制循环：单线程接收并抢占式执行运动
        """
        print("[Control] Moving to initial position (Blocking)...")
        # 1. 先阻塞式移动到初始位置，确保安全
        try:
            init_motion = JointMotion(INITIAL_POSITION.tolist())
            self.robot.move(init_motion) # 默认 asynchronous=False
            print("[Control] Reached initial position.")
        except Exception as e:
            print(f"[Fatal] Failed to reach initial position: {e}")
            return

        print("="*60)
        print(f"[Control] STARTING ASYNC LOOP - DYNAMICS: {RELATIVE_DYNAMICS_FACTOR}")
        print("[Control] Robot is ready for FACTR streaming.")
        print("="*60)

        # 计数器用于调试
        cmd_count = 0
        last_debug_time = time.time()
        
        # 确保进入循环前，ZMQ 缓冲区清空
        try:
            while True:
                self.cmd_subscriber.recv(zmq.NOBLOCK)
        except zmq.Again:
            pass

        while self.running:
            try:
                # --- A. 接收指令 (非阻塞) ---
                try:
                    # 获取最新的一条指令 (因 CONFLATE=1)
                    message = self.cmd_subscriber.recv(zmq.NOBLOCK)
                    
                    if len(message) == 7 * 8: # 7 doubles
                        target_array = np.frombuffer(message, dtype=np.float64)
                        
                        # --- B. 发送给机器人 (异步抢占) ---
                        # 核心：asynchronous=True
                        # 这会立即返回，底层控制器会平滑过渡到新目标
                        motion = JointMotion(target_array.tolist())
                        self.robot.move(motion, asynchronous=True)
                        
                        cmd_count += 1
                    
                except zmq.Again:
                    # 没有新指令时，什么都不做
                    # 机器人会继续执行上一个指令直到完成（或停在最后位置）
                    # 对于 500Hz 流，这里通常不会等太久
                    # 稍微 sleep 避免 CPU 100% 空转 (可选)
                    # time.sleep(0.0001) 
                    pass

                # --- 调试打印 (每2秒) ---
                if time.time() - last_debug_time > 2.0:
                    # print(f"[Control] Processed {cmd_count} commands. Loop running...")
                    last_debug_time = time.time()

            except Exception as e:
                # --- C. 异常处理 ---
                # 重要：异步模式下的运动错误（如速度超限、奇异点）
                # 会在下一次调用 move 时抛出。
                print(f"\n[Error] Motion Exception caught: {e}")
                
                # 尝试恢复策略：
                # 1. 打印错误
                # 2. 如果是严重错误，机器人可能已经停止
                # 3. 尝试自动恢复 (如果支持)
                try:
                    if hasattr(self.robot, 'automatic_error_recovery'):
                        print("[Recovery] Triggering automatic error recovery...")
                        self.robot.automatic_error_recovery()
                    elif hasattr(self.robot, 'recover_from_errors'):
                        self.robot.recover_from_errors()
                except Exception as rec_e:
                    print(f"[Recovery] Failed: {rec_e}")
                
                # 给一点时间让错误清除
                time.sleep(0.5)
                
                # 重新移动到当前位置（防止跳变）
                # 或者直接继续循环，等待 FACTR 的下一个点（FACTR 应该也会收到错误并重置）

    def start(self):
        print("--- Franka-FACTR Bridge Initializing ---")
        
        # 1. 连接机器人
        try:
            print(f"Connecting to Robot: {ROBOT_IP}")
            self.robot = Robot(ROBOT_IP)
            self.robot.relative_dynamics_factor = RELATIVE_DYNAMICS_FACTOR
            print("Robot Connected.")
        except Exception as e:
            print(f"[Fatal] Robot connection failed: {e}")
            return

        # 2. 设置 ZMQ
        try:
            self.setup_zmq()
        except Exception as e:
            print(f"[Fatal] ZMQ setup failed: {e}")
            return

        self.running = True

        # 3. 启动状态发布线程 (Daemon)
        pub_thread = threading.Thread(target=self.state_publisher_loop, daemon=True)
        pub_thread.start()

        # 4. 进入主控制循环 (Blocking)
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("\n[Main] User interrupted.")
        except Exception as e:
            print(f"[Main] Unexpected error: {e}")
        finally:
            self.stop()

    def stop(self):
        print("[Main] Stopping bridge...")
        self.running = False
        
        # 停止 ZMQ
        if self.zmq_context:
            self.zmq_context.term()
            
        # 停止机器人
        if self.robot:
            try:
                # 等待可能的异步运动结束，然后停止
                # 忽略这里的错误，因为我们正在关闭
                try:
                    self.robot.join_motion() 
                except: 
                    pass
                self.robot.stop()
            except Exception as e:
                print(f"Error stopping robot: {e}")
        
        print("[Main] Bridge stopped.")

def signal_handler(sig, frame):
    print("\n[Signal] SIGINT received. Exiting...")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    bridge = FrankaFactrBridge()
    bridge.start()