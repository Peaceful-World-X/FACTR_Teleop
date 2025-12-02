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

import os
import time
import yaml
import subprocess
import numpy as np
import pinocchio as pin
from abc import ABC, abstractmethod

from rclpy.node import Node
from python_utils.utils import get_workspace_root
from factr_teleop.dynamixel.driver import DynamixelDriver


def find_ttyusb(port_name):
    """查找底层的 ttyUSB 设备。

    给定在配置中使用的设备名（例如由 /dev/serial/by-id 提供的符号链接名），
    返回实际的 ttyUSB 设备名（例如 "ttyUSB0"）。如果找不到或解析失败，抛出异常并给出详细信息。
    """
    base_path = "/dev/serial/by-id/"
    full_path = os.path.join(base_path, port_name)
    if not os.path.exists(full_path):
        raise Exception(f"Port '{port_name}' does not exist in {base_path}.")
    try:
        resolved_path = os.readlink(full_path)
        actual_device = os.path.basename(resolved_path)
        if actual_device.startswith("ttyUSB"):
            return actual_device
        else:
            raise Exception(
                f"The port '{port_name}' does not correspond to a ttyUSB device. It links to {resolved_path}."
            )
    except Exception as e:
        raise Exception(f"Unable to resolve the symbolic link for '{port_name}'. {e}")


class FACTRTeleop(Node, ABC):
    """FACTR 遥操作基类（leader 侧），用于实现力反馈远程控制。

    该类实现了 leader 侧机器人的主控制循环，包含重力补偿、空域（null-space）调节、摩擦补偿以及力反馈等功能。
    作为基类使用时，子类需要实现抽象方法以建立 leader 与 follower（例如 Franka）之间的通信，以及提供夹爪的力反馈处理。
    """
    def __init__(self):
        super().__init__('factr_teleop')

        config_file_name = self.declare_parameter('config_file', '').get_parameter_value().string_value
        config_path = os.path.join(get_workspace_root(), f"src/factr_teleop/factr_teleop/configs/{config_file_name}")
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.name = self.config["name"]
        self.dt = 1 / self.config["controller"]["frequency"]
        
        self._prepare_dynamixel()
        self._prepare_inverse_dynamics()

    # leader 机械臂相关参数
        self.num_arm_joints = self.config["arm_teleop"]["num_arm_joints"]
        self.safety_margin = self.config["arm_teleop"]["arm_joint_limits_safety_margin"]
        self.arm_joint_limits_max = np.array(self.config["arm_teleop"]["arm_joint_limits_max"]) - self.safety_margin
        self.arm_joint_limits_min = np.array(self.config["arm_teleop"]["arm_joint_limits_min"]) + self.safety_margin
        self.calibration_joint_pos = np.array(self.config["arm_teleop"]["initialization"]["calibration_joint_pos"])
        self.initial_match_joint_pos = np.array(self.config["arm_teleop"]["initialization"]["initial_match_joint_pos"])
        assert self.num_arm_joints == len(self.arm_joint_limits_max) == len(self.arm_joint_limits_min), \
            "num_arm_joints and the length of arm joint limits must be the same"
        assert self.num_arm_joints == len(self.calibration_joint_pos) == len(self.initial_match_joint_pos), \
            "num_arm_joints and the length of calibration_joint_pos and initial_match_joint_pos must be the same"
        
    # leader 夹爪相关参数
        self.gripper_limit_min = 0.0
        self.gripper_limit_max = self.config["gripper_teleop"]["actuation_range"]
        self.gripper_pos_prev = 0.0
        self.gripper_pos = 0.0

    # 重力补偿相关
        self.enable_gravity_comp = self.config["controller"]["gravity_comp"]["enable"]
        self.gravity_comp_modifier = self.config["controller"]["gravity_comp"]["gain"]
        self.tau_g = np.zeros(self.num_arm_joints)
    # 摩擦补偿相关
        self.stiction_comp_enable_speed = self.config["controller"]["static_friction_comp"]["enable_speed"]
        self.stiction_comp_gain = self.config["controller"]["static_friction_comp"]["gain"]
        self.stiction_dither_flag = np.ones((self.num_arm_joints), dtype=bool)
    # 关节极限保护（障碍势）
        self.joint_limit_kp = self.config["controller"]["joint_limit_barrier"]["kp"]
        self.joint_limit_kd = self.config["controller"]["joint_limit_barrier"]["kd"]
    # 空间（null-space）调节相关
        self.null_space_joint_target = np.array(self.config["controller"]["null_space_regulation"]["null_space_joint_target"])
        self.null_space_kp = self.config["controller"]["null_space_regulation"]["kp"]
        self.null_space_kd = self.config["controller"]["null_space_regulation"]["kd"]
    # 力矩反馈相关
        self.enable_torque_feedback = self.config["controller"]["torque_feedback"]["enable"]
        self.torque_feedback_gain = self.config["controller"]["torque_feedback"]["gain"]
        self.torque_feedback_motor_scalar = self.config["controller"]["torque_feedback"]["motor_scalar"]
        self.torque_feedback_damping = self.config["controller"]["torque_feedback"]["damping"]
    # 夹爪反馈相关
        self.enable_gripper_feedback = self.config["controller"]["gripper_feedback"]["enable"]
        
        # 下面方法需要在子类中实现，用于建立 leader 与 follower 之间的通信
        self.set_up_communication()

        # calibrate the leader arm joints before starting
        self._get_dynamixel_offsets()
        # ensure the leader and the follower arms have the same joint positions before starting
        self._match_start_pos()
        # start the control loop
        self.timer = self.create_timer(self.dt, self.control_loop_callback)


    def _prepare_dynamixel(self):
        """初始化并配置用于控制 Dynamixel 舵机的驱动。

        会检查串口延迟计时器是否满足实时控制要求，并将电机设置为扭矩模式。
        如果串口设备不存在则记录并返回。
        """
        self.servo_types = self.config["dynamixel"]["servo_types"]
        self.num_motors = len(self.servo_types)
        self.joint_signs = np.array(self.config["dynamixel"]["joint_signs"], dtype=float)
        assert self.num_motors == len(self.joint_signs), \
            "The number of motors and the number of joint signs must be the same"
        self.dynamixel_port = "/dev/serial/by-id/" + self.config["dynamixel"]["dynamixel_port"]

    # 检查对应 ttyUSB 设备的 latency_timer 是否为 1
    # 如果不是 1，则无法保证控制循环在 200Hz 以上稳定运行，可能导致不良行为。
    # 如需设置为 1，可运行：
    # echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB{NUM}/latency_timer
        ttyUSBx = find_ttyusb(self.dynamixel_port)
        command = f"cat /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"        
        result = subprocess.run(command, shell=True, capture_output=True, text=True, check=True)
        ttyUSB_latency_timer = int(result.stdout)
        if ttyUSB_latency_timer != 1:
            raise Exception(
                f"Please ensure the latency timer of {ttyUSBx} is 1. Run: \n \
                echo 1 | sudo tee /sys/bus/usb-serial/devices/{ttyUSBx}/latency_timer"
            )

        joint_ids = np.arange(self.num_motors) + 1
        try:
            self.driver = DynamixelDriver(
                joint_ids, self.servo_types, self.dynamixel_port
            )
        except FileNotFoundError:
            self.get_logger().info(f"Port {self.dynamixel_port} not found. Please check the connection.")
            return
        self.driver.set_torque_mode(False)
        # set operating mode to current mode
        self.driver.set_operating_mode(0)
        # enable torque
        self.driver.set_torque_mode(True)

    def _prepare_inverse_dynamics(self):
        """基于 URDF 构建 leader 机械臂的动力学模型，用于重力补偿与空域调节的计算。"""
        self.leader_urdf = os.path.join(
            'src/factr_teleop/factr_teleop/urdf/', 
            self.config["arm_teleop"]["leader_urdf"]
        )
        workspace_root = get_workspace_root()
        urdf_model_path = os.path.join(workspace_root, self.leader_urdf)
        urdf_model_dir = os.path.join(workspace_root, os.path.dirname(urdf_model_path))
        self.pin_model, _, _ = pin.buildModelsFromUrdf(filename=urdf_model_path, package_dirs=urdf_model_dir)
        self.pin_data = self.pin_model.createData()

    def _get_dynamixel_offsets(self, verbose=True):
        """校准 Dynamixel 与 Franka 之间的角度偏移，使 leader 读取与 follower 对齐。

        启动前应手动将 leader 机械臂移动到与 follower 的校准位置大致一致的位置（每关节 ±90° 范围内）。
        """
    # 预热读取舵机状态
        for _ in range(10):
            self.driver.get_positions_and_velocities()
        
        def _get_error(calibration_joint_pos, offset, index, joint_state):
            joint_sign_i = self.joint_signs[index]
            joint_i = joint_sign_i * (joint_state[index] - offset)
            start_i = calibration_joint_pos[index]
            return np.abs(joint_i - start_i)

    # 计算每个关节的偏移量
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

    # 计算夹爪的偏移
        curr_gripper_joint = curr_joints[-1]
        self.joint_offsets.append(curr_gripper_joint)

        self.joint_offsets = np.asarray(self.joint_offsets)
        if verbose:
            print(self.joint_offsets)
            print("最佳偏移 (rad)             : ", [f"{x:.3f}" for x in self.joint_offsets])
            print(
                "以 π/2 为单位的近似偏移: ["
                + ", ".join([f"{int(np.round(x/(np.pi/2)))}*π/2" for x in self.joint_offsets])
                + " ]",
            )
    
    def _match_start_pos(self):
        """等待用户将 leader 机械臂手动移动到与 follower 大致相同的起始位姿，保证启动时两臂一致。"""
        curr_pos, _, _, _ = self.get_leader_joint_states()
        while (np.linalg.norm(curr_pos - self.initial_match_joint_pos[0:self.num_arm_joints]) > 0.6):
            current_joint_error = np.linalg.norm(
                curr_pos - self.initial_match_joint_pos[0:self.num_arm_joints]
            )
            self.get_logger().info(
                f"FACTR TELEOP {self.name}: Please match starting joint pos. Current error: {current_joint_error}"
            )
            curr_pos, _, _, _ = self.get_leader_joint_states()
            time.sleep(0.5)
        self.get_logger().info(f"FACTR TELEOP {self.name}: 起始关节位置匹配完成。")

    def shut_down(self):
        """节点关闭时，禁用 leader 机械臂与夹爪的扭矩输出。"""
        self.set_leader_joint_torque(np.zeros(self.num_arm_joints), 0.0)
        self.driver.set_torque_mode(False)

    def get_leader_joint_states(self):
        """返回 leader 机械臂与夹爪当前的关节位置与速度，按 follower 的关节约定进行对齐。"""
        self.gripper_pos_prev = self.gripper_pos
        joint_pos, joint_vel = self.driver.get_positions_and_velocities()
        joint_pos_arm = (
            joint_pos[0:self.num_arm_joints] - self.joint_offsets[0:self.num_arm_joints]
        ) * self.joint_signs[0:self.num_arm_joints]
        self.gripper_pos = (joint_pos[-1] - self.joint_offsets[-1]) * self.joint_signs[-1]
        joint_vel_arm = joint_vel[0:self.num_arm_joints] * self.joint_signs[0:self.num_arm_joints]
        
        gripper_vel = (self.gripper_pos - self.gripper_pos_prev) / self.dt
        return joint_pos_arm, joint_vel_arm, self.gripper_pos, gripper_vel
    
    def set_leader_joint_pos(self, goal_joint_pos, goal_gripper_pos):
        """使用 PD 控制将 leader 机械臂与夹爪移动到指定的关节目标位姿。

        该方法通过插值步进逼近目标位置并基于 PD 控制计算扭矩，用于对齐起始位姿或做慢速校正。

        注意：该函数默认不在主遥操作循环中使用。为保证稳定性，Dynamixel 的 latency 必须足够低（建议 200 Hz 以上）。
        """
        interpolation_step_size = np.ones(7)*self.config["controller"]["interpolation_step_size"]
        kp = self.config["controller"]["joint_position_control"]["kp"]
        kd = self.config["controller"]["joint_position_control"]["kd"]

        curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_leader_joint_states()
        while (np.linalg.norm(curr_pos - goal_joint_pos) > 0.1):
            next_joint_pos_target = np.where(
                np.abs(curr_pos - goal_joint_pos) > interpolation_step_size, 
                curr_pos + interpolation_step_size*np.sign(goal_joint_pos-curr_pos),
                goal_joint_pos,
            )
            torque = -kp*(curr_pos-next_joint_pos_target)-kd*(curr_vel)
            gripper_torque = -kp*(curr_gripper_pos-goal_gripper_pos)-kd*(curr_gripper_vel)
            self.set_leader_joint_torque(torque, gripper_torque)
            curr_pos, curr_vel, curr_gripper_pos, curr_gripper_vel = self.get_leader_joint_states()
    
    def set_leader_joint_torque(self, arm_torque, gripper_torque):
        """对 leader 机械臂与夹爪施加关节扭矩。"""
        arm_gripper_torque = np.append(arm_torque, gripper_torque)
        self.driver.set_torque(arm_gripper_torque*self.joint_signs)


    def joint_limit_barrier(self, arm_joint_pos, arm_joint_vel, gripper_joint_pos, gripper_joint_vel):
        """计算关节极限的斥力扭矩，防止 leader 越过 follower 的物理关节限制。

        实现了一个简化的控制律：当接近或超过边界时施加与距离和速度成比例的斥力扭矩以保护机械臂。
        """
        exceed_max_mask = arm_joint_pos > self.arm_joint_limits_max
        tau_l = (-self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_max) \
            - self.joint_limit_kd * arm_joint_vel) * exceed_max_mask
        exceed_min_mask = arm_joint_pos < self.arm_joint_limits_min
        tau_l += (-self.joint_limit_kp * (arm_joint_pos - self.arm_joint_limits_min) \
            - self.joint_limit_kd * arm_joint_vel) * exceed_min_mask
        
        if gripper_joint_pos > self.gripper_limit_max:
            tau_l_gripper = -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_max) \
                - self.joint_limit_kd * gripper_joint_vel
        elif gripper_joint_pos < self.gripper_limit_min:
            tau_l_gripper = -self.joint_limit_kp * (gripper_joint_pos - self.gripper_limit_min) \
                - self.joint_limit_kd * gripper_joint_vel
        else:
            tau_l_gripper = 0.0
        return tau_l, tau_l_gripper

    def gravity_compensation(self, arm_joint_pos, arm_joint_vel):
        """使用逆动力学（RNEA）计算重力补偿扭矩，并按比例因子调整补偿强度。"""
        self.tau_g = pin.rnea(
            self.pin_model, self.pin_data, 
            arm_joint_pos, arm_joint_vel, np.zeros_like(arm_joint_vel)
        )
        self.tau_g *= self.gravity_comp_modifier 
        return self.tau_g

    def friction_compensation(self, arm_joint_vel):
        """计算用于补偿静摩擦的关节扭矩。

        该实现只包含静摩擦补偿（用于抵消舵机静态摩擦），不包含动摩擦补偿。
        """
        tau_ss = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(arm_joint_vel[i]) < self.stiction_comp_enable_speed:
                if self.stiction_dither_flag[i]:
                    tau_ss[i] += self.stiction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_ss[i] -= self.stiction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither_flag[i] = ~self.stiction_dither_flag[i]
        return tau_ss
    
    def null_space_regulation(self, arm_joint_pos, arm_joint_vel):
        """计算用于空域调节的关节扭矩，以在冗余情形下实现次要目标（不影响主要任务）。"""
        J = pin.computeJointJacobian(
            self.pin_model, self.pin_data, arm_joint_pos, self.num_arm_joints
        )
        J_dagger = np.linalg.pinv(J)
        null_space_projector = np.eye(self.num_arm_joints) - J_dagger @ J
        q_error = arm_joint_pos - self.null_space_joint_target[0:self.num_arm_joints]
        tau_n = null_space_projector @ (-self.null_space_kp*q_error-self.null_space_kd*arm_joint_vel)
        return tau_n
    
    def torque_feedback(self, external_torque, arm_joint_vel):
        """根据 follower 端的外部关节力矩计算 leader 侧的力反馈扭矩。"""
        tau_ff = -1.0*self.torque_feedback_gain/self.torque_feedback_motor_scalar * external_torque
        tau_ff -= self.torque_feedback_damping*arm_joint_vel
        return tau_ff

    def control_loop_callback(self):
        """leader 侧主控制循环。

        说明：控制循环最高可运行到 500 Hz，但较低频率（例如 200 Hz）在适当调参下仍可获得良好表现。
        要支持 500 Hz，需将 Dynamixel 波特率设置为 4 Mbps 且 Return Delay Time 置为 0。
        """
        leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel = self.get_leader_joint_states()

        torque_arm = np.zeros(self.num_arm_joints)
        torque_l, torque_gripper = self.joint_limit_barrier(
            leader_arm_pos, leader_arm_vel, leader_gripper_pos, leader_gripper_vel
        )
        torque_arm += torque_l
        torque_arm += self.null_space_regulation(leader_arm_pos, leader_arm_vel)

        if self.enable_gravity_comp:
            torque_arm += self.gravity_compensation(leader_arm_pos, leader_arm_vel)
            torque_arm += self.friction_compensation(leader_arm_vel)
        
        if self.enable_torque_feedback:
            external_joint_torque = self.get_leader_arm_external_joint_torque()
            torque_arm += self.torque_feedback(external_joint_torque, leader_arm_vel)
        
        if self.enable_gripper_feedback:
            gripper_feedback = self.get_leader_gripper_feedback()
            torque_gripper += self.gripper_feedback(leader_gripper_pos, leader_gripper_vel, gripper_feedback)

        self.set_leader_joint_torque(torque_arm, torque_gripper)
        self.update_communication(leader_arm_pos, leader_gripper_pos)


    @abstractmethod
    def set_up_communication(self):
        """在子类中实现：用于初始化 leader 与 follower 之间的通信（仅在构造时调用一次）。

        例如：可以创建用于接收 follower 外部力矩的订阅器，以及用于发送位置目标的发布器等。
        """
        pass


    @abstractmethod
    def get_leader_arm_external_joint_torque(self):
        """在子类中实现：获取 follower 端的外部关节力矩，用于 leader 侧的力反馈计算。"""
        pass


    @abstractmethod
    def get_leader_gripper_feedback(self):
        """在子类中实现：获取 follower 夹爪的反馈数据（例如位置或力），用于 leader 夹爪的力反馈逻辑。"""
        pass


    @abstractmethod
    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        """在子类中实现：处理夹爪反馈并返回要施加在 leader 夹爪上的扭矩（力反馈计算）。"""
        pass


    @abstractmethod
    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        """在控制循环的每次迭代中被调用：负责将 leader 的关节目标等通信给 follower。

        子类需实现具体的消息发送/发布逻辑。
        """
        pass
