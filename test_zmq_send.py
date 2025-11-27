#!/usr/bin/env python3
"""
ZMQ 消息发送和接收测试脚本
- 发送模式：测试 Franka 端的 ZMQ SUB 接收功能
- 接收模式：测试 Franka 端的 ZMQ PUB 发送功能
"""

import zmq
import numpy as np
import time
import sys

def send_joint_positions(address, rate=10.0):
    """发送关节位置命令 (7个double)"""
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(address)
    
    print(f"ZMQ Publisher started on {address}")
    print("Sending joint positions (7 doubles) at {:.1f} Hz".format(rate))
    print("Press Ctrl+C to stop")
    
    # 等待订阅者连接
    time.sleep(1)
    
    # Franka 初始位置（从终端读取的实际初始位置）
    initial_positions = np.array([
        0.0048,    # q[0]
        0.0051,    # q[1]
        -0.0045,   # q[2]
        -1.5690,   # q[3]
        0.0052,    # q[4]
        1.5691,    # q[5]
        1.5749     # q[6] - 初始位置
    ], dtype=np.float64)
    
    # 关节 6 的目标位置：初始位置 + 10度 (约 0.1745 弧度)
    target_joint6 = initial_positions[6] + np.deg2rad(10.0)  # 约 1.7494 rad
    
    print(f"\nInitial positions (rad): {initial_positions}")
    print(f"Initial positions (deg): {np.rad2deg(initial_positions)}")
    print(f"\nTarget joint 6 position: {target_joint6:.4f} rad ({np.rad2deg(target_joint6):.2f} deg)")
    print(f"Joint 6 movement: +{np.rad2deg(target_joint6 - initial_positions[6]):.2f} degrees")
    print(f"\nOther joints will remain at initial positions.\n")
    
    try:
        count = 0
        while True:
            # 保持其他关节在初始位置，只改变关节 6
            positions = initial_positions.copy()
            positions[6] = target_joint6  # 关节 6 转动 10 度
            
            # 发送消息
            socket.send(positions.tobytes())
            count += 1
            
            if count == 1:
                print(f"Sending command: {positions}")
                print(f"  Joint 6: {positions[6]:.4f} rad ({np.rad2deg(positions[6]):.2f} deg)")
            
            if count % 10 == 0:
                print(f"Sent {count} messages...")
            
            time.sleep(1.0 / rate)
            
    except KeyboardInterrupt:
        print(f"\nStopped. Total messages sent: {count}")
    finally:
        socket.close()
        context.term()


def receive_robot_state(state_address=None, torque_address=None):
    """接收机器人状态和力矩数据"""
    if not state_address and not torque_address:
        print("ERROR: At least one address (state or torque) must be provided!")
        return
    
    context = zmq.Context()
    
    # 订阅状态数据（如果提供）
    state_socket = None
    if state_address:
        state_socket = context.socket(zmq.SUB)
        print(f"[State] Attempting to connect to {state_address}...")
        state_socket.connect(state_address)
        state_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        state_socket.setsockopt(zmq.CONFLATE, 1)  # 只保留最新消息
        
        print(f"[State] ZMQ Subscriber connected to {state_address}")
        print(f"[State] Expected message size: {14 * 4} bytes (14 floats: 7 positions + 7 velocities)")
    
    # 订阅力矩数据（如果提供）
    torque_socket = None
    if torque_address:
        torque_socket = context.socket(zmq.SUB)
        print(f"[Torque] Attempting to connect to {torque_address}...")
        torque_socket.connect(torque_address)
        torque_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        torque_socket.setsockopt(zmq.CONFLATE, 1)
        print(f"[Torque] ZMQ Subscriber connected to {torque_address}")
        print(f"[Torque] Expected message size: {7 * 4} bytes (7 floats: torques)")
    
    print("Waiting for messages...")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    # 设置非阻塞模式
    poller = zmq.Poller()
    if state_socket:
        poller.register(state_socket, zmq.POLLIN)
    if torque_socket:
        poller.register(torque_socket, zmq.POLLIN)
    
    state_count = 0
    torque_count = 0
    last_print_time = time.time()
    
    try:
        while True:
            # 轮询消息
            socks = dict(poller.poll(timeout=100))  # 100ms 超时
            
            # 接收状态消息
            if state_socket and state_socket in socks and socks[state_socket] == zmq.POLLIN:
                try:
                    message = state_socket.recv(zmq.NOBLOCK)
                    if len(message) == 14 * 8:  # 14个float64，每个8字节
                        data = np.frombuffer(message, dtype=np.float64)
                        positions = data[0:7]
                        velocities = data[7:14]
                        
                        state_count += 1
                        current_time = time.time()
                        
                        # 每秒打印一次
                        if current_time - last_print_time >= 1.0:
                            print(f"\n[State #{state_count}] Received at {time.strftime('%H:%M:%S')}")
                            print(f"  Positions: {positions}")
                            print(f"  Velocities: {velocities}")
                            last_print_time = current_time
                    elif len(message) == 14 * 4:  # 14个float32，兼容旧格式
                        data = np.frombuffer(message, dtype=np.float32)
                        positions = data[0:7]
                        velocities = data[7:14]
                        state_count += 1
                        current_time = time.time()
                        if current_time - last_print_time >= 1.0:
                            print(f"\n[State #{state_count}] Received (float32) at {time.strftime('%H:%M:%S')}")
                            print(f"  Positions: {positions}")
                            print(f"  Velocities: {velocities}")
                            last_print_time = current_time
                    elif len(message) == 7 * 8:  # 7个float64，可能是力矩数据
                        # 如果收到 56 字节（float64），可能是力矩数据被错误地发送到了状态地址
                        torques = np.frombuffer(message, dtype=np.float64)
                        print(f"[State] WARNING: Received torque data (56 bytes float64) on state socket!")
                        print(f"  This might indicate a configuration error.")
                        print(f"  Torques: {torques}")
                        print(f"  Please check if you're connecting to the correct address.")
                    elif len(message) == 7 * 4:  # 7个float32，可能是力矩数据
                        torques = np.frombuffer(message, dtype=np.float32)
                        print(f"[State] WARNING: Received torque data (28 bytes float32) on state socket!")
                        print(f"  This might indicate a configuration error.")
                        print(f"  Torques: {torques}")
                        print(f"  Please check if you're connecting to the correct address.")
                    else:
                        print(f"[State] Invalid message size: {len(message)} bytes (expected: {14 * 8} for state float64 or {14 * 4} for state float32 or {7 * 8} for torque float64 or {7 * 4} for torque float32)")
                except zmq.Again:
                    pass
            
            # 接收力矩消息
            if torque_socket and torque_socket in socks and socks[torque_socket] == zmq.POLLIN:
                try:
                    message = torque_socket.recv(zmq.NOBLOCK)
                    if len(message) == 7 * 8:  # 7个float64，每个8字节
                        torques = np.frombuffer(message, dtype=np.float64)
                        torque_count += 1
                        
                        current_time = time.time()
                        if current_time - last_print_time >= 1.0:
                            print(f"[Torque #{torque_count}] Received")
                            print(f"  Torques: {torques}")
                            last_print_time = current_time
                    elif len(message) == 7 * 4:  # 7个float32，兼容旧格式
                        torques = np.frombuffer(message, dtype=np.float32)
                        torque_count += 1
                        current_time = time.time()
                        if current_time - last_print_time >= 1.0:
                            print(f"[Torque #{torque_count}] Received (float32)")
                            print(f"  Torques: {torques}")
                            last_print_time = current_time
                    elif len(message) == 14 * 8:  # 14个float64，可能是状态数据
                        # 如果收到 112 字节，可能是状态数据被错误地发送到了力矩地址
                        data = np.frombuffer(message, dtype=np.float64)
                        print(f"[Torque] WARNING: Received state data (112 bytes float64) on torque socket!")
                        print(f"  This might indicate a configuration error.")
                        print(f"  Positions: {data[0:7]}, Velocities: {data[7:14]}")
                    elif len(message) == 14 * 4:  # 14个float32，可能是状态数据
                        data = np.frombuffer(message, dtype=np.float32)
                        print(f"[Torque] WARNING: Received state data (56 bytes float32) on torque socket!")
                        print(f"  This might indicate a configuration error.")
                        print(f"  Positions: {data[0:7]}, Velocities: {data[7:14]}")
                    else:
                        print(f"[Torque] Invalid message size: {len(message)} bytes (expected: {7 * 8} for torque float64 or {7 * 4} for torque float32 or {14 * 8} for state float64 or {14 * 4} for state float32)")
                except zmq.Again:
                    pass
            
            # 如果没有消息，稍微休眠避免CPU占用过高
            if not socks:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print(f"\n\nStopped.")
        print(f"Total state messages received: {state_count}")
        if torque_socket:
            print(f"Total torque messages received: {torque_count}")
    finally:
        state_socket.close()
        if torque_socket:
            torque_socket.close()
        context.term()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Send mode:  python test_zmq_send.py send [address] [rate]")
        print("  Receive mode: python test_zmq_send.py recv [state_address] [torque_address]")
        print("")
        print("Examples:")
        print("  # Send commands to Franka")
        print("  python test_zmq_send.py send tcp://*:2098 10.0")
        print("")
        print("  # Receive state from Franka")
        print("  python test_zmq_send.py recv tcp://10.0.10.1:3099")
        print("")
        print("  # Receive both state and torque from Franka")
        print("  python test_zmq_send.py recv tcp://10.0.10.1:3099 tcp://10.0.10.1:3087")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    
    if mode == "send":
        # 发送模式
        default_address = "tcp://*:2098"
        default_rate = 10.0
        
        address = sys.argv[2] if len(sys.argv) > 2 else default_address
        rate = float(sys.argv[3]) if len(sys.argv) > 3 else default_rate
        
        print("=" * 50)
        print("ZMQ Test Publisher (Send Mode)")
        print("=" * 50)
        print(f"Address: {address}")
        print(f"Rate: {rate} Hz")
        print("=" * 50)
        
        send_joint_positions(address, rate)
        
    elif mode == "recv":
        # 接收模式
        default_state_address = "tcp://10.0.10.1:3099"
        default_torque_address = "tcp://10.0.10.1:3087"
        
        # 如果只提供一个地址，根据端口判断是状态还是力矩
        if len(sys.argv) == 3:
            # 只提供了一个地址
            provided_address = sys.argv[2]
            if ":3099" in provided_address:
                # 提供的是状态地址
                state_address = provided_address
                torque_address = None
            elif ":3087" in provided_address:
                # 提供的是力矩地址
                state_address = None
                torque_address = provided_address
            else:
                # 无法判断，默认作为状态地址
                state_address = provided_address
                torque_address = None
        elif len(sys.argv) >= 4:
            # 提供了两个地址
            state_address = sys.argv[2]
            torque_address = sys.argv[3]
        else:
            # 使用默认值
            state_address = default_state_address
            torque_address = default_torque_address
        
        print("=" * 50)
        print("ZMQ Test Subscriber (Receive Mode)")
        print("=" * 50)
        if state_address:
            print(f"State address: {state_address}")
        if torque_address:
            print(f"Torque address: {torque_address}")
        if not state_address and not torque_address:
            print("ERROR: No valid address provided!")
            sys.exit(1)
        print("=" * 50)
        
        receive_robot_state(state_address, torque_address)
        
    else:
        print(f"Unknown mode: {mode}")
        print("Use 'send' or 'recv'")
        sys.exit(1)

