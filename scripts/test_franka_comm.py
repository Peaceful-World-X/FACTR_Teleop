#!/usr/bin/env python3
"""
Franka FCI 通信诊断工具

用于测试和诊断 FACTR 与 Franka 之间的 ZMQ 通信连接。
"""

import zmq
import numpy as np
import time
import sys
from typing import Optional

class FrankaCommDiagnostic:
    def __init__(self, robot_side: str = "right"):
        """
        初始化诊断工具
        
        Args:
            robot_side: "left" 或 "right"
        """
        self.robot_side = robot_side
        
        # 从配置文件导入地址
        try:
            sys.path.insert(0, '/home/cytoderm/projects/FACTR_Teleop/src/python_utils')
            from python_utils.global_configs import (
                sim_desktop_ip_address,
                franka_left_real_zmq_addresses,
                franka_right_real_zmq_addresses
            )
        except ImportError:
            # 如果导入失败，使用默认值
            sim_desktop_ip_address = "192.168.40.200"
            franka_left_real_zmq_addresses = {
                "joint_state_sub":  "tcp://172.16.0.1:5099",
                "joint_torque_sub": "tcp://172.16.0.1:5087",
                "joint_pos_cmd_pub": "tcp://192.168.40.200:4098",
            }
            franka_right_real_zmq_addresses = {
                "joint_state_sub":  "tcp://10.0.10.2:3099",
                "joint_torque_sub": "tcp://10.0.10.2:3087",
                "joint_pos_cmd_pub": "tcp://192.168.40.200:2098",
            }
        
        if robot_side == "left":
            self.zmq_addresses = franka_left_real_zmq_addresses
        elif robot_side == "right":
            self.zmq_addresses = franka_right_real_zmq_addresses
        else:
            raise ValueError("robot_side must be 'left' or 'right'")
        
        self.desktop_ip = sim_desktop_ip_address
        self.context = zmq.Context()
    
    def test_command_publisher(self):
        """测试命令发布端（FACTR → Franka）"""
        print(f"\n{'='*60}")
        print(f"测试 1: 命令发布器 (FACTR → Franka)")
        print(f"{'='*60}")
        
        address = self.zmq_addresses["joint_pos_cmd_pub"]
        print(f"绑定地址: {address}")
        
        try:
            pub = self.context.socket(zmq.PUB)
            pub.bind(address)
            print(" Socket 创建成功")
            
            time.sleep(1)  # 等待连接建立
            
            # 发送测试数据
            test_positions = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 1.57])
            
            print(f"\n发送测试位置命令:")
            print(f"  {test_positions}")
            
            for i in range(5):
                pub.send(test_positions.astype(np.float64).tobytes())
                print(f"  第 {i+1} 次发送完成")
                time.sleep(0.5)
            
            print("\n 命令发布测试完成")
            print("  如果 Franka 控制端正在运行，它应该能收到这些命令")
            
            pub.close()
            return True
            
        except Exception as e:
            print(f"✗ 错误: {e}")
            return False
    
    def test_state_subscriber(self, timeout: int = 5):
        """测试状态订阅端（Franka → FACTR）"""
        print(f"\n{'='*60}")
        print(f"测试 2: 状态订阅器 (Franka → FACTR)")
        print(f"{'='*60}")
        
        address = self.zmq_addresses["joint_state_sub"]
        print(f"连接地址: {address}")
        
        try:
            sub = self.context.socket(zmq.SUB)
            sub.connect(address)
            sub.setsockopt(zmq.SUBSCRIBE, b'')
            sub.setsockopt(zmq.RCVTIMEO, timeout * 1000)  # 超时时间（毫秒）
            print("✓ Socket 创建并连接成功")
            
            print(f"\n等待接收状态数据（超时 {timeout} 秒）...")
            
            try:
                message = sub.recv()
                data = np.frombuffer(message, dtype=np.float32)
                
                if len(data) >= 7:
                    print("\n✓ 成功接收到状态数据:")
                    print(f"  位置: {data[:7]}")
                    if len(data) >= 14:
                        print(f"  速度: {data[7:14]}")
                    return True
                else:
                    print(f"✗ 接收到的数据长度不正确: {len(data)}")
                    return False
                    
            except zmq.Again:
                print(f"\n✗ 超时: 在 {timeout} 秒内没有收到数据")
                print("  请确认:")
                print("  1. Franka 控制端程序正在运行")
                print("  2. 网络连接正常")
                print(f"  3. Franka 控制端正在向 {address} 发布数据")
                return False
            
            sub.close()
            
        except Exception as e:
            print(f"✗ 错误: {e}")
            return False
    
    def test_torque_subscriber(self, timeout: int = 5):
        """测试力矩订阅端（Franka → FACTR）"""
        print(f"\n{'='*60}")
        print(f"测试 3: 力矩订阅器 (Franka → FACTR)")
        print(f"{'='*60}")
        
        address = self.zmq_addresses["joint_torque_sub"]
        print(f"连接地址: {address}")
        
        try:
            sub = self.context.socket(zmq.SUB)
            sub.connect(address)
            sub.setsockopt(zmq.SUBSCRIBE, b'')
            sub.setsockopt(zmq.RCVTIMEO, timeout * 1000)
            print("✓ Socket 创建并连接成功")
            
            print(f"\n等待接收力矩数据（超时 {timeout} 秒）...")
            
            try:
                message = sub.recv()
                data = np.frombuffer(message, dtype=np.float32)
                
                if len(data) >= 7:
                    print("\n✓ 成功接收到力矩数据:")
                    print(f"  外部力矩: {data[:7]}")
                    return True
                else:
                    print(f"✗ 接收到的数据长度不正确: {len(data)}")
                    return False
                    
            except zmq.Again:
                print(f"\n✗ 超时: 在 {timeout} 秒内没有收到数据")
                return False
            
            sub.close()
            
        except Exception as e:
            print(f"✗ 错误: {e}")
            return False
    
    def test_network_connectivity(self):
        """测试网络连通性"""
        print(f"\n{'='*60}")
        print(f"测试 0: 网络连通性")
        print(f"{'='*60}")
        
        import subprocess
        
        # 从地址中提取 IP
        state_address = self.zmq_addresses["joint_state_sub"]
        franka_ip = state_address.split("//")[1].split(":")[0]
        
        print(f"\n目标 Franka 控制端 IP: {franka_ip}")
        
        try:
            result = subprocess.run(
                ['ping', '-c', '3', franka_ip],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                print(f"✓ 网络连接正常")
                # 提取延迟信息
                output_lines = result.stdout.split('\n')
                for line in output_lines:
                    if 'time=' in line or 'rtt' in line:
                        print(f"  {line.strip()}")
                return True
            else:
                print(f"✗ 无法连接到 {franka_ip}")
                print(f"  请检查:")
                print(f"  1. Franka 控制端是否开机")
                print(f"  2. 网络配置是否正确")
                print(f"  3. 防火墙设置")
                return False
                
        except subprocess.TimeoutExpired:
            print(f"✗ Ping 超时")
            return False
        except Exception as e:
            print(f"✗ 错误: {e}")
            return False
    
    def run_all_tests(self):
        """运行所有诊断测试"""
        print(f"\n{'#'*60}")
        print(f"# Franka FCI 通信诊断工具")
        print(f"# 机器人侧: {self.robot_side.upper()}")
        print(f"{'#'*60}")
        
        # 显示配置信息
        print(f"\n配置信息:")
        print(f"  FACTR 工作站 IP: {self.desktop_ip}")
        for key, value in self.zmq_addresses.items():
            print(f"  {key}: {value}")
        
        results = []
        
        # 测试 0: 网络连通性
        results.append(("网络连通性", self.test_network_connectivity()))
        
        # 测试 1: 命令发布
        results.append(("命令发布", self.test_command_publisher()))
        
        # 测试 2: 状态订阅
        results.append(("状态订阅", self.test_state_subscriber(timeout=5)))
        
        # 测试 3: 力矩订阅
        results.append(("力矩订阅", self.test_torque_subscriber(timeout=5)))
        
        # 总结
        print(f"\n{'='*60}")
        print(f"测试总结")
        print(f"{'='*60}")
        
        for name, result in results:
            status = "✓ 通过" if result else "✗ 失败"
            print(f"  {name:20s}: {status}")
        
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        print(f"\n总计: {passed}/{total} 测试通过")
        
        if passed == total:
            print("\n✓ 所有测试通过！系统通信正常。")
        else:
            print("\n✗ 部分测试失败，请检查上述错误信息。")
        
        return passed == total


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Franka FCI 通信诊断工具")
    parser.add_argument(
        "--side",
        choices=["left", "right"],
        default="right",
        help="机器人侧: left 或 right（默认: right）"
    )
    parser.add_argument(
        "--test",
        choices=["all", "network", "command", "state", "torque"],
        default="all",
        help="要运行的测试（默认: all）"
    )
    
    args = parser.parse_args()
    
    diagnostic = FrankaCommDiagnostic(robot_side=args.side)
    
    if args.test == "all":
        success = diagnostic.run_all_tests()
    elif args.test == "network":
        success = diagnostic.test_network_connectivity()
    elif args.test == "command":
        success = diagnostic.test_command_publisher()
    elif args.test == "state":
        success = diagnostic.test_state_subscriber()
    elif args.test == "torque":
        success = diagnostic.test_torque_subscriber()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
