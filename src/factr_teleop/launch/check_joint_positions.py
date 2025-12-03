#!/usr/bin/env python3
"""
工具脚本：读取 leader arm 的当前关节角度
用于帮助将 leader arm 移动到校准位置
"""
import sys
import os
import numpy as np
import math
import time

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(project_root, 'src', 'factr_teleop'))
dynamixel_sdk_path = os.path.join(project_root, 'src', 'factr_teleop', 'factr_teleop', 'dynamixel', 'python', 'src')
if os.path.exists(dynamixel_sdk_path):
    sys.path.insert(0, dynamixel_sdk_path)

from factr_teleop.factr_teleop import find_ttyusb
from factr_teleop.dynamixel.driver import DynamixelDriver
import yaml


class SimpleTeleopWrapper:
    """
    简化的包装类，用于调用 FACTRTeleop 的 get_leader_joint_states() 方法
    不初始化 ROS2 节点，只初始化必要的属性
    包含 _get_dynamixel_offsets 方法，可以执行校准过程
    """
    def __init__(self, driver, config, joint_offsets=None, do_calibration=True):
        self.driver = driver
        self.num_arm_joints = config["arm_teleop"]["num_arm_joints"]
        self.joint_signs = np.array(config["dynamixel"]["joint_signs"])
        self.calibration_joint_pos = np.array(config["arm_teleop"]["initialization"]["calibration_joint_pos"])
        self.dt = 1 / config["controller"]["frequency"]
        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0
        
        # 如果提供了 joint_offsets，使用它；否则初始化为零
        if joint_offsets is not None:
            self.joint_offsets = joint_offsets
        else:
            # 初始化为 8 个元素（7个关节 + 1个夹爪）
            self.joint_offsets = np.zeros(8)
        
        # 如果需要校准，执行校准过程
        if do_calibration:
            self._get_dynamixel_offsets(verbose=True)
    
    def _get_dynamixel_offsets(self, verbose=True):
        """
        Calibrates the Dynamixel servos with respect to the Franka arm to ensure the joint
        position readings of the leader arm correspond to those of the follower arm.
        
        这是从 factr_teleop.py 中复制的真实实现
        """
        # warm up
        for _ in range(10):
            self.driver.get_positions_and_velocities()
        
        def _get_error(calibration_joint_pos, offset, index, joint_state):
            joint_sign_i = self.joint_signs[index]
            joint_i = joint_sign_i * (joint_state[index] - offset)
            start_i = calibration_joint_pos[index]
            return np.abs(joint_i - start_i)

        # get arm offsets
        self.joint_offsets = []
        curr_joints, _ = self.driver.get_positions_and_velocities()
        for i in range(self.num_arm_joints):
            best_offset = 0
            best_error = 1e9
            # intervals of pi/2
            for offset in np.linspace(-20 * np.pi, 20 * np.pi, 20 * 4 + 1):  
                error = _get_error(self.calibration_joint_pos, offset, i, curr_joints)
                if error < best_error:
                    best_error = error
                    best_offset = offset
            self.joint_offsets.append(best_offset)

        # get gripper offset:
        curr_gripper_joint = curr_joints[-1]
        self.joint_offsets.append(curr_gripper_joint)
        self.joint_offsets = np.asarray(self.joint_offsets)
    
    def get_leader_joint_states(self):
        self.gripper_pos_prev = self.gripper_pos
        joint_pos, joint_vel = self.driver.get_positions_and_velocities()
        joint_pos_arm = (
            joint_pos[0:self.num_arm_joints] - self.joint_offsets[0:self.num_arm_joints]
        ) * self.joint_signs[0:self.num_arm_joints]
        self.gripper_pos = (joint_pos[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]
        joint_vel_arm = joint_vel[0:self.num_arm_joints] * self.joint_signs[0:self.num_arm_joints]
        
        gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        return joint_pos_arm, joint_vel_arm, self.gripper_pos, gripper_vel


def main():
    # 固定配置
    CONFIG_FILE = "grav_comp_demo.yaml"
    REFRESH_RATE = 10.0  # Hz
    
    # 加载配置文件
    config_path = os.path.join(
        project_root,
        f"src/factr_teleop/factr_teleop/configs/{CONFIG_FILE}"
    )
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    dynamixel_config = config['dynamixel']
    dynamixel_port = "/dev/serial/by-id/" + dynamixel_config['dynamixel_port']
    
    # 获取端口
    ttyusb_name = find_ttyusb(dynamixel_port)
    port = "/dev/" + ttyusb_name
    
    # 获取配置
    joint_signs = np.array(dynamixel_config['joint_signs']) # 运动方向
    calibration_pos = np.array(config['arm_teleop']['initialization']['calibration_joint_pos'])
    initial_match_pos = np.array(config['arm_teleop']['initialization']['initial_match_joint_pos'])
    arm_limits_max = np.array(config['arm_teleop']['arm_joint_limits_max'])
    arm_limits_min = np.array(config['arm_teleop']['arm_joint_limits_min'])
    gripper_limit_min = 0.0
    gripper_limit_max = config['gripper_teleop']['actuation_range']
    # 包含7个关节 + 1个夹爪，共8个电机
    num_motors = len(dynamixel_config['servo_types'])
    ids = list(range(1, num_motors + 1))  # 从1到num_motors（通常是8：7个关节+1个夹爪）
    servo_types = dynamixel_config['servo_types']
    
    # 创建驱动
    try:
        driver = DynamixelDriver(
            ids=ids,
            servo_types=servo_types,
            port=port,
            baudrate=4000000
        )
    except Exception as e:
        print(f"错误: 无法连接到 Dynamixel 设备", file=sys.stderr)
        print(f"详细信息: {e}", file=sys.stderr)
        sys.exit(1)
    
    # 创建 SimpleTeleopWrapper 实例   
    teleop_wrapper = SimpleTeleopWrapper(driver, config, joint_offsets=None, do_calibration=True)
    
    # 实时显示关节角度
    sleep_time = 1.0 / REFRESH_RATE
    
    try:
        while True:
            # 方法1：原始编码器读数
            joint_pos_raw, _ = driver.get_positions_and_velocities()
            
            # 方法2：真实调用 get_leader_joint_states() 获取校准后的位置
            joint_pos_arm, _, gripper_pos_calibrated, _ = teleop_wrapper.get_leader_joint_states()
            
            os.system('clear' if os.name != 'nt' else 'cls')
            print("=" * 100)
            print("关节角度详细信息 (按 Ctrl+C 退出)")
            print("=" * 100)
            
            # 表头
            print(f"{'关节':<12} {'原始':<12} {'校准后':<10} {'偏移':<8} {'calibration':<12} {'initial_match':<15} {'限制范围':<12}")
            print("-" * 100)
            
            calibrated_radians = []
            # 显示7个关节
            for i in range(7):
                # 原始编码器读数（方法1）- 显示为度数
                raw_rad = joint_pos_raw[i]
                raw_deg = math.degrees(raw_rad)
                
                # 校准后的位置（方法2）
                calibrated_rad = joint_pos_arm[i]
                calibrated_deg = math.degrees(calibrated_rad)
                calibrated_radians.append(calibrated_rad)  # 用于底部显示
                
                # 偏移量（从 SimpleTeleopWrapper 中获取）
                offset_rad = teleop_wrapper.joint_offsets[i] if i < len(teleop_wrapper.joint_offsets) else 0.0
                offset_deg = math.degrees(offset_rad)
                
                # 校准目标位置
                calibration_target_rad = calibration_pos[i]
                calibration_target_deg = math.degrees(calibration_target_rad)
                
                # 初始匹配目标位置
                match_target_rad = initial_match_pos[i]
                match_target_deg = math.degrees(match_target_rad)
                
                # 限制范围
                limit_min_deg = math.degrees(arm_limits_min[i])
                limit_max_deg = math.degrees(arm_limits_max[i])
                limit_range_str = f"[{limit_min_deg:>6.1f}°, {limit_max_deg:>6.1f}°]"
                
                print(f"关节{i+1:<2}  {raw_deg:>12.2f}°  {calibrated_deg:>12.2f}°  {offset_deg:>12.2f}°  "
                      f"{calibration_target_deg:>12.2f}°  {match_target_deg:>12.2f}°  {limit_range_str:<25}")
            
            # 显示夹爪（关节8）
            gripper_raw_rad = joint_pos_raw[-1]  # 夹爪是最后一个
            gripper_raw_deg = math.degrees(gripper_raw_rad)
            # 注意：底部数组不包含夹爪，只显示7个关节的校准后值
            
            gripper_calibrated_deg = math.degrees(gripper_pos_calibrated)
            
            # 夹爪偏移量
            gripper_offset_rad = teleop_wrapper.joint_offsets[-1] if len(teleop_wrapper.joint_offsets) > 7 else 0.0
            gripper_offset_deg = math.degrees(gripper_offset_rad)
            
            # 夹爪限制范围
            gripper_limit_min_deg = math.degrees(gripper_limit_min)
            gripper_limit_max_deg = math.degrees(gripper_limit_max)
            gripper_limit_range_str = f"[{gripper_limit_min_deg:>6.1f}°, {gripper_limit_max_deg:>6.1f}°]"
            
            print(f"夹爪    {gripper_raw_deg:>12.2f}°  {gripper_calibrated_deg:>12.2f}°  {gripper_offset_deg:>12.2f}°  "
                  f"{'N/A':>12}  {'N/A':>14}  {gripper_limit_range_str:<25}")
            
            print("-" * 100)
            print("\n校准后(弧度) (7个关节，不含夹爪):")
            angles_str = "[" + ", ".join([f"{angle:.4f}" for angle in calibrated_radians]) + "]"
            print(angles_str)
            time.sleep(sleep_time)
        
    except KeyboardInterrupt:
        print("\n已退出")
    finally:
        driver.close()

if __name__ == "__main__":
    main()
