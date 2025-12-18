#!/usr/bin/env python3
"""Robotiq 2F-85 夹爪 ROS 2 桥接节点。

该节点通过 Modbus RTU 协议处理 ROS 2 话题与物理 Robotiq 2F-85 夹爪之间的通信。
它订阅夹爪命令并发布当前夹爪状态。

"""

import logging
import math
import struct
import sys
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

# --- 配置常量 ---

# 串口连接
DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_BAUDRATE = 115200
DEFAULT_SLAVE_ID = 0x0009
DEFAULT_SERIAL_TIMEOUT = 0.5  # 秒，过短可能导致读取响应失败，过长会让初始化看起来“卡住”

# Modbus 寄存器地址
REG_CMD_START = 0x03E8    # 1000: 动作请求
REG_STAT_START = 0x07D0   # 2000: 夹爪状态

# 物理限制（米）
# 规格: 0.0m (闭合) ~ 0.085m (张开)
# Robotiq 寄存器: 0(张开) ~ 255(闭合)
WIDTH_MIN = 0.0
WIDTH_MAX = 0.085

# FACTR 侧命令范围（弧度）
# 0.0 弧度 = 完全闭合, 0.8 弧度 = 完全张开
FACTR_CMD_MIN_RAD = 0.0
FACTR_CMD_MAX_RAD = 0.8

# 控制模式
# "relative": 相对控制（归一化到百分比后映射）
# "absolute": 绝对控制（直接使用 Leader 位置值，单位：米）
CONTROL_MODE_RELATIVE = "relative"
CONTROL_MODE_ABSOLUTE = "absolute"

# 寄存器限制（来自规格书第 8.3 和 10 节）
# 规格: 0 (张开) ~ 255 (闭合)
# 注意: 此映射相对于标准"位置"是反向的
REG_POS_OPEN = 3
REG_POS_CLOSED = 230
REG_SPEED_MIN = 0
REG_SPEED_MAX = 255
REG_FORCE_MIN = 0
REG_FORCE_MAX = 255

# 控制标志位
BIT_ACT = 0   # 激活
BIT_GTO = 3   # 执行移动
BIT_ATR = 4   # 自动释放

# 状态标志位
BIT_gACT = 0
BIT_gGTO = 3
# gSTA 是位 4-5, gOBJ 是位 6-7


class RobotiqGripperHardware:
    """处理与 Robotiq 2F-85 夹爪的低层 Modbus RTU 通信。"""

    def __init__(self, port: str = DEFAULT_PORT, slave_id: int = DEFAULT_SLAVE_ID, baudrate: int = DEFAULT_BAUDRATE, timeout: float = DEFAULT_SERIAL_TIMEOUT):
        """使用原始 Modbus RTU 通过串口初始化硬件接口。

        Args:
            port: 串口路径（例如 '/dev/ttyUSB2'）。
            slave_id: Modbus 从站 ID（默认 9）。
            baudrate: 串口波特率（默认 115200）。
            timeout: 串口超时时间（秒）
        """
        self.port = port
        self.slave_id = slave_id
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        # 使用可重入锁，避免在初始化流程中嵌套调用（initialize -> reset_gripper -> emergency_release）
        # 造成的死锁。
        self.lock = threading.RLock()
        self.logger = logging.getLogger("RobotiqHW")
        self.logger.setLevel(logging.INFO)

    def connect(self) -> bool:
        """打开底层串口。"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            return True
        except serial.SerialException as exc:
            self.logger.error(f"Failed to open serial port {self.port}: {exc}")
            self.serial = None
            return False

    def disconnect(self):
        """关闭连接。"""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except serial.SerialException:
                # 忽略关闭时的错误
                pass

    #计算 Modbus RTU CRC16（低位在前）。
    def _compute_crc(self, data: bytes) -> int:
    
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    #将物理值（米）线性映射到寄存器值。
    def _map_to_register(self, value: float, phy_min: float, phy_max: float, 
                        reg_open: int, reg_closed: int) -> int:
        """将物理值（米）线性映射到寄存器值。
        
        Args:
            value: 目标宽度（米）（0.0 = 闭合, 0.085 = 张开）
            phy_min: 最小物理宽度（0.0）
            phy_max: 最大物理宽度（0.085）
            reg_open: 张开状态下的寄存器值（0）
            reg_closed: 闭合状态下的寄存器值（255）
        """
        # 限制输入范围
        value = max(phy_min, min(phy_max, value))
        
        # 线性映射: 
        # ratio = (value - phy_min) / (phy_max - phy_min)
        # reg_val = reg_closed + ratio * (reg_open - reg_closed)
        
        ratio = (value - phy_min) / (phy_max - phy_min)
        reg_val = int(reg_closed + ratio * (reg_open - reg_closed))
        
        return max(0, min(255, reg_val))

    #将寄存器值线性映射到物理值。
    def _map_from_register(self, reg_val: int, phy_min: float, phy_max: float, 
                          reg_open: int, reg_closed: int) -> float:
        """将寄存器值线性映射到物理值。"""
        reg_val = max(0, min(255, reg_val))
        
        # ratio = (reg_val - reg_closed) / (reg_open - reg_closed)
        # value = phy_min + ratio * (phy_max - phy_min)
        
        if reg_open == reg_closed:
            return phy_min
            
        ratio = (reg_val - reg_closed) / (reg_open - reg_closed)
        return phy_min + ratio * (phy_max - phy_min)

    #清除可能的错误状态。
    def emergency_release(self) -> bool:
        """执行紧急释放序列，清除可能的错误状态。
        
        步骤：
        1. 发送 Act=0 命令（去激活）以清除错误状态
        2. 等待一段时间让夹爪复位
        3. 验证状态已清除
        
        Returns:
            成功返回 True
        """
        with self.lock:
            if not self._write_command(act=0, gto=0, r_pr=0, r_sp=0, r_fr=0):
                self.logger.warning("紧急释放: 去激活命令发送失败")
                return False
            
            time.sleep(0.3)
            
            status = self._read_status_raw()
            if status:
                g_flt = status["gFLT"]
                if g_flt != 0:
                    self.logger.warning(f"紧急释放后仍检测到故障代码: gFLT={g_flt}")
            
            return True
    
    #完全复位夹爪，清除激活位。
    def reset_gripper(self) -> bool:
        """完全复位夹爪，清除激活位。

        发送复位命令：09 10 03 E8 00 03 06 00 00 00 00 00 00 [CRC]
        将所有寄存器设置为0，相当于：
        - ACT = 0 (去激活)
        - GTO = 0 (不执行移动)
        - ATR = 0 (不自动释放)
        - rPR = 0 (位置请求)
        - rSP = 0 (速度)
        - rFR = 0 (力度)

        Returns:
            成功返回 True
        """
        if self.serial is None or not self.serial.is_open:
            self.logger.error("串口未打开；无法复位夹爪。")
            return False

        with self.lock:
            # 构建复位命令帧：将所有6个字节设置为0
            # [id][func=0x10][addr_hi][addr_lo][qty_hi][qty_lo][byte_count][data...][crc_lo][crc_hi]
            pdu = bytearray()
            pdu.append(self.slave_id & 0xFF)
            pdu.append(0x10)  # 写多个寄存器
            pdu.extend(struct.pack(">H", REG_CMD_START))
            pdu.extend(struct.pack(">H", 3))  # 3 个寄存器
            pdu.append(6)  # 6 个数据字节
            pdu.extend([0, 0, 0, 0, 0, 0])  # 所有数据字节为0

            crc = self._compute_crc(bytes(pdu))
            frame = bytes(pdu) + struct.pack("<H", crc)

            try:
                # 发送复位命令
                self.serial.reset_input_buffer()
                self.serial.write(frame)
                self.serial.flush()

                # 读取响应
                resp = self.serial.read(8)
                if len(resp) != 8:
                    self.logger.warning(
                        f"复位命令: 期望 8 字节响应，收到 {len(resp)} 字节。"
                    )
                    return False

                # 验证响应
                data_no_crc = resp[:-2]
                crc_bytes = resp[-2:]
                expected_crc = self._compute_crc(data_no_crc)
                recv_crc = int.from_bytes(crc_bytes, byteorder="little")

                if expected_crc != recv_crc:
                    self.logger.warning("复位命令: 响应中 CRC 不匹配。")
                    return False

                if data_no_crc[0] != (self.slave_id & 0xFF) or data_no_crc[1] != 0x10:
                    self.logger.warning("复位命令: 意外的单元 ID 或功能码。")
                    return False

                return True

            except serial.SerialException as exc:
                self.logger.warning(f"复位时异常: {exc}")
                return False

    def initialize(self) -> bool:
        """运行完整的夹爪激活序列。

        完整流程：
        1. 完全复位夹爪（清除激活位）
        2. 执行紧急释放序列（清除可能的错误状态）
        3. 发送激活命令 (Act=1)
        4. 等待状态变为 gACT=1, gFLT=0

        Returns:
            成功返回 True
        """
        with self.lock:
            # 步骤 1: 完全复位夹爪
            reset_result = self.reset_gripper()
            if not reset_result:
                self.logger.warning("夹爪复位失败，但继续尝试激活...")

            # 步骤 2: 紧急释放（清除错误状态）
            release_result = self.emergency_release()
            if not release_result:
                self.logger.warning("紧急释放失败，但继续尝试激活...")

            # 步骤 3: 发送激活命令
            activate_result = self._write_command(act=1, gto=0, r_pr=REG_POS_OPEN, r_sp=255, r_fr=150)
            if not activate_result:
                self.logger.error("夹爪激活失败：Modbus 命令发送失败")
                return False
            
            # 步骤 4: 等待状态变为激活且无故障
            timeout = 3.0
            start_time = time.time()
            check_interval = 0.1

            check_count = 0
            while time.time() - start_time < timeout:
                check_count += 1
                status = self._read_status_raw()

                if status is None:
                    time.sleep(check_interval)
                    continue

                g_act = status["gACT_byte"] & 0x01
                g_sta = (status["gACT_byte"] >> 4) & 0x03
                g_flt = status["gFLT"]

                # 检查是否达到目标状态
                if g_act == 1 and g_flt == 0:
                    self.logger.info(f"夹爪激活成功 (gSTA={g_sta}, gACT={g_act}, gFLT={g_flt})")
                    return True

                # 如果检测到故障，记录警告
                if g_flt != 0:
                    self.logger.warning(f"激活过程中检测到故障: gFLT={g_flt}, gSTA={g_sta}, gACT={g_act}")

                time.sleep(check_interval)
            
            # 超时后检查最终状态
            status = self._read_status_raw()
            if status:
                g_act = status["gACT_byte"] & 0x01
                g_sta = (status["gACT_byte"] >> 4) & 0x03
                g_flt = status["gFLT"]
                self.logger.error(
                    f"夹爪激活超时！最终状态: gSTA={g_sta}, gACT={g_act}, gFLT={g_flt}"
                )
            else:
                self.logger.error("夹爪激活超时：无法读取状态")
            
            return False

    def _write_command(self, act: int, gto: int, r_pr: int, r_sp: int, r_fr: int) -> bool:
        """将 6 字节命令写入夹爪寄存器。

        Args:
            act: 0 或 1（激活）
            gto: 0 或 1（执行移动）
            r_pr: 0-255（位置请求）
            r_sp: 0-255（速度）
            r_fr: 0-255（力度）
        
        Returns:
            成功返回 True。
        """
        if self.serial is None or not self.serial.is_open:
            self.logger.error("串口未打开；无法写入命令。")
            return False

        # 构建字节（匹配用户验证的帧）:
        # 寄存器 1000: [动作请求, 保留]
        # 寄存器 1001: [保留, 位置请求]
        # 寄存器 1002: [速度, 力度]
        byte0 = 0
        if act:
            byte0 |= (1 << BIT_ACT)
        if gto:
            byte0 |= (1 << BIT_GTO)

        byte1 = 0
        byte2 = 0
        byte3 = r_pr
        byte4 = r_sp
        byte5 = r_fr

        # 手动构建 Modbus RTU 帧:
        # [id][func=0x10][addr_hi][addr_lo][qty_hi][qty_lo][byte_count][data...][crc_lo][crc_hi]
        pdu = bytearray()
        pdu.append(self.slave_id & 0xFF)
        pdu.append(0x10)  # 写多个寄存器
        pdu.extend(struct.pack(">H", REG_CMD_START))
        pdu.extend(struct.pack(">H", 3))  # 3 个寄存器
        pdu.append(6)  # 6 个数据字节
        pdu.extend([byte0, byte1, byte2, byte3, byte4, byte5])

        crc = self._compute_crc(bytes(pdu))
        frame = bytes(pdu) + struct.pack("<H", crc)

        try:
            # 清除任何残留输入，发送帧，并读取回显响应。
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            self.serial.flush()

            # 用户观察到的预期响应示例:
            # 09 10 03 E8 00 03 01 30  (8 字节)
            resp = self.serial.read(8)
            if len(resp) != 8:
                self.logger.warning(
                    f"Modbus 写入: 期望 8 字节响应，收到 {len(resp)} 字节。"
                )
                return False

            # 基本完整性检查: 单元 ID、功能码、CRC
            data_no_crc = resp[:-2]
            crc_bytes = resp[-2:]
            expected_crc = self._compute_crc(data_no_crc)
            recv_crc = int.from_bytes(crc_bytes, byteorder="little")
            if expected_crc != recv_crc:
                self.logger.warning("Modbus 写入: 响应中 CRC 不匹配。")
                return False

            if data_no_crc[0] != (self.slave_id & 0xFF) or data_no_crc[1] != 0x10:
                self.logger.warning("Modbus 写入: 意外的单元 ID 或功能码。")
                return False

            return True
        except serial.SerialException as exc:
            self.logger.warning(f"写入时异常: {exc}")
            return False

    def _read_status_raw(self) -> Optional[dict]:
        """读取状态寄存器并返回原始字节/值。"""
        if self.serial is None or not self.serial.is_open:
            self.logger.error("串口未打开；无法读取状态。")
            return None

        # 构建 Modbus RTU 帧: 读保持寄存器
        # [id][func=0x03][addr_hi][addr_lo][qty_hi][qty_lo][crc_lo][crc_hi]
        try:
            pdu = bytearray()
            pdu.append(self.slave_id & 0xFF)
            pdu.append(0x03)  # 读保持寄存器
            pdu.extend(struct.pack(">H", REG_STAT_START))
            pdu.extend(struct.pack(">H", 3))  # 3 个寄存器 => 6 个数据字节

            crc = self._compute_crc(bytes(pdu))
            frame = bytes(pdu) + struct.pack("<H", crc)

            self.serial.reset_input_buffer()
            self.serial.write(frame)
            self.serial.flush()

            # 响应应为: [id][func=0x03][byte_count=6][data(6)][crc(2)]
            header = self.serial.read(3)
            if len(header) != 3:
                self.logger.warning(
                    f"Modbus 读取: 期望 3 字节头部，收到 {len(header)} 字节。"
                )
                return None

            unit_id = header[0]
            func = header[1]
            byte_count = header[2]

            if unit_id != (self.slave_id & 0xFF) or func != 0x03:
                self.logger.warning("Modbus 读取: 意外的单元 ID 或功能码。")
                # 继续但标记为失败
                return None

            if byte_count != 6:
                self.logger.warning(
                    f"Modbus 读取: 意外的字节计数 {byte_count}，期望 6。"
                )
                return None

            rest = self.serial.read(byte_count + 2)
            if len(rest) != byte_count + 2:
                self.logger.warning(
                    f"Modbus 读取: 期望 {byte_count + 2} 字节，收到 {len(rest)}。"
                )
                return None

            data = rest[:byte_count]
            crc_bytes = rest[byte_count:]

            expected_crc = self._compute_crc(header + data)
            recv_crc = int.from_bytes(crc_bytes, byteorder="little")
            if expected_crc != recv_crc:
                self.logger.warning("Modbus 读取: 响应中 CRC 不匹配。")
                return None

            if len(data) != 6:
                self.logger.warning("Modbus 读取: 状态数据长度不匹配。")
                return None

            b0, b1, b2, b3, b4, b5 = data

            return {
                "gACT_byte": b0,
                "gFLT": b2,
                "gPR": b3,
                "gPO": b4,
                "gCU": b5,
            }
        except serial.SerialException as exc:
            self.logger.warning(f"读取时异常: {exc}")
            return None

    def get_state(self) -> Tuple[float, float, bool]:
        """读取夹爪状态。

        Returns:
            元组 (position_meters, current_amps, is_moving)
        """
        with self.lock:
            raw = self._read_status_raw()
            if not raw:
                return 0.0, 0.0, False
            
            # 解析位置 (gPO)
            # 映射: 寄存器 0(张开) -> 0.085m, 255(闭合) -> 0.0m
            pos_m = self._map_from_register(
                raw['gPO'], 
                WIDTH_MIN, WIDTH_MAX, 
                REG_POS_OPEN, REG_POS_CLOSED
            )

            # 解析电流 (gCU)
            # 规格 4.5: 0.1 A / 单位
            current_a = raw['gCU'] * 0.1

            # 解析运动状态 (gOBJ)
            # gOBJ (位 6-7): 0=运动中, 1/2=已停止
            g_obj = (raw['gACT_byte'] >> 6) & 0x03
            is_moving = (g_obj == 0)

            return pos_m, current_a, is_moving
    
    def get_status_detailed(self) -> Optional[dict]:
        """读取详细的夹爪状态信息。
        
        Returns:
            包含详细状态信息的字典，如果读取失败返回 None
        """
        with self.lock:
            raw = self._read_status_raw()
            if not raw:
                return None
            
            g_act_byte = raw['gACT_byte']
            g_act = g_act_byte & 0x01  # 位 0: 激活状态
            g_gto = (g_act_byte >> 3) & 0x01  # 位 3: 执行移动
            g_sta = (g_act_byte >> 4) & 0x03  # 位 4-5: 夹爪状态
            g_obj = (g_act_byte >> 6) & 0x03  # 位 6-7: 对象检测
            
            return {
                'gACT': g_act,  # 0=未激活, 1=激活
                'gGTO': g_gto,  # 0=停止, 1=执行移动
                'gSTA': g_sta,  # 0=复位中, 1=配置中, 2=未激活, 3=激活
                'gOBJ': g_obj,  # 0=运动中, 1=已停止(无对象), 2=已停止(有对象), 3=未知
                'gFLT': raw['gFLT'],  # 故障代码 (0=无故障)
                'gPR': raw['gPR'],  # 位置请求
                'gPO': raw['gPO'],  # 当前位置
                'gCU': raw['gCU'],  # 电流
            }

    def set_target(self, width_m: float, force_percent: float = 50.0, speed_percent: float = 100.0):
        """向夹爪发送移动命令。

        Args:
            width_m: 目标开口宽度（米）。
            force_percent: 夹持力度（0-100）。
            speed_percent: 夹持速度（0-100）。
        """
        # 转换为寄存器值
        reg_pos = self._map_to_register(
            width_m, 
            WIDTH_MIN, WIDTH_MAX, 
            REG_POS_OPEN, REG_POS_CLOSED
        )
        
        reg_force = self._map_to_register(
            force_percent, 
            0.0, 100.0, 
            # 注意：_map_to_register 的 reg_open 参数对应于物理最大值时的寄存器值，
            # 因此传入 REG_FORCE_MAX (对应 100%) 作为 reg_open，REG_FORCE_MIN 作为 reg_closed，
            # 保证 force_percent 从 0->100 映射为寄存器从 0->255（非反向）。
            REG_FORCE_MAX, REG_FORCE_MIN
        )
        
        reg_speed = self._map_to_register(
            speed_percent, 
            0.0, 100.0, 
            # 同上：确保 speed_percent 越大，寄存器值越大（非反向）
            REG_SPEED_MAX, REG_SPEED_MIN
        )

        with self.lock:
            # 移动时始终发送 Act=1, Gto=1
            self._write_command(1, 1, reg_pos, reg_speed, reg_force)


class GripperBridgeNode(Node):
    """ROS 2 节点，桥接 ROS 话题与 Robotiq 夹爪。"""

    def __init__(self):
        super().__init__('gripper_bridge_node')
        
        # 声明参数
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('slave_id', DEFAULT_SLAVE_ID)
        self.declare_parameter('poll_rate', 50.0)  # 赫兹
        self.declare_parameter('serial_timeout', DEFAULT_SERIAL_TIMEOUT)  # 串口超时时间
        self.declare_parameter('control_mode', CONTROL_MODE_ABSOLUTE)  # 控制模式: "relative" 或 "absolute"
        self.declare_parameter('leader_gripper_min_rad', 0.0)  # Leader 夹爪最小位置（弧度）
        self.declare_parameter('leader_gripper_max_rad', 0.8)  # Leader 夹爪最大位置（弧度）
        self.declare_parameter('skip_init', False)  # 诊断用：跳过初始化
        self.declare_parameter('robot_name', 'right') # 机器人名称 (left/right)
        # 默认速度/力度（百分比 0-100）
        self.declare_parameter('default_speed_percent', 100.0)
        self.declare_parameter('default_force_percent', 50.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        slave_id = self.get_parameter('slave_id').value
        self.poll_rate = self.get_parameter('poll_rate').value
        serial_timeout = self.get_parameter('serial_timeout').value
        self.control_mode = self.get_parameter('control_mode').value
        self.leader_gripper_min_rad = self.get_parameter('leader_gripper_min_rad').value
        self.leader_gripper_max_rad = self.get_parameter('leader_gripper_max_rad').value
        self.skip_init = self.get_parameter('skip_init').value
        self.robot_name = self.get_parameter('robot_name').value
        self.default_speed_percent = float(self.get_parameter('default_speed_percent').value)
        self.default_force_percent = float(self.get_parameter('default_force_percent').value)

        self.get_logger().info(f"初始化夹爪桥接节点，端口: {port} (ID: {slave_id}, 波特率: {baudrate})...")
        self.get_logger().info(f"控制模式: {self.control_mode}")
        self.get_logger().info(f"机器人名称: {self.robot_name}")
        if self.control_mode == CONTROL_MODE_ABSOLUTE:
            self.get_logger().info(f"  Leader 夹爪范围: [{self.leader_gripper_min_rad:.3f}, {self.leader_gripper_max_rad:.3f}] 弧度")
            self.get_logger().info(f"  Follower 夹爪范围: [{WIDTH_MIN:.3f}, {WIDTH_MAX:.3f}] 米")

        # 初始化硬件
        self.hw = RobotiqGripperHardware(port, slave_id, baudrate, timeout=serial_timeout)
        if not self.hw.connect():
            self.get_logger().error("无法连接到夹爪串口。")
            # 我们不在这里退出以保持节点存活用于诊断，
            # 但功能将失败。
            # 或者，可以使用 sys.exit(1)
        elif self.skip_init:
            self.get_logger().warn("skip_init=true，跳过硬件初始化（诊断模式）")
            self.is_activated = True
        else:
            self.get_logger().info("初始化夹爪...")
            try:
                init_success = self.hw.initialize()
                if init_success:
                    self.get_logger().info("夹爪初始化成功")
                    self.is_activated = True
                else:
                    self.get_logger().error("夹爪初始化失败")
                    self.is_activated = False
            except Exception as e:
                self.get_logger().error(f"初始化过程中发生异常: {e}")
                self.is_activated = False

        # ROS 订阅者
        cmd_topic = f'/factr_teleop/{self.robot_name}/cmd_gripper_pos'
        self.sub_cmd = self.create_subscription(
            JointState,
            cmd_topic,
            self.cmd_callback,
            1
        )
        self.get_logger().info(f"已订阅 {cmd_topic}")

        # ROS 发布者
        self.pub_state = self.create_publisher(
            JointState,
            '/bridge/obs_gripper_state',
            10
        )

        # 控制循环定时器
        self.timer = self.create_timer(1.0 / self.poll_rate, self.control_loop)

        # 内部状态
        self.last_target_pos = None
        self.lock = threading.Lock()
        self.last_error_check_time = 0.0
        self.error_check_interval = 1.0  # 每秒检查一次错误状态
        self.is_activated = False  # 跟踪激活状态
        self.has_received_command = False  # 是否已收到第一个命令
        self.leader_ratio = 0.0  # 0=闭合, 1=张开
        # 速度/力度：以百分比（0-100）存储，初始为参数值
        self.last_speed_percent = float(self.default_speed_percent)
        self.last_force_percent = float(self.default_force_percent)

    def cmd_callback(self, msg: JointState):
        """夹爪命令回调函数。
        
        根据控制模式处理命令：
        
        - "relative" 模式（相对控制）:
          期望 msg.position[0] 为 FACTR 侧夹爪角度（弧度），其中:
          - leader_gripper_min_rad => 完全闭合 (0.0 米)
          - leader_gripper_max_rad => 完全张开 (0.085 米)
          角度归一化到 0.0–1.0 百分比，然后映射到物理宽度。
        
        - "absolute" 模式（绝对控制）:
          期望 msg.position[0] 为 Leader 夹爪位置（弧度），直接映射到 Follower 夹爪位置（米）。
          使用线性映射：Leader [min_rad, max_rad] -> Follower [WIDTH_MIN, WIDTH_MAX]
          实现一一对应的绝对位置控制。
        """
        if not msg.position:
            return

        try:
            #leader发送的值为校准后弧度，开机时夹爪角度为0度
            cmd_value = float(msg.position[0])
            
            if self.control_mode == CONTROL_MODE_ABSOLUTE:
                # 绝对控制模式：Leader 位置直接映射到 Follower 位置
                # 限制到 Leader 夹爪范围
                cmd_value = max(self.leader_gripper_min_rad, min(self.leader_gripper_max_rad, cmd_value))
                
                # 线性映射：Leader [min_rad, max_rad] -> Follower [WIDTH_MIN, WIDTH_MAX]
                if self.leader_gripper_max_rad > self.leader_gripper_min_rad:
                    # 归一化到 [0, 1]
                    ratio = (cmd_value - self.leader_gripper_min_rad) / (
                        self.leader_gripper_max_rad - self.leader_gripper_min_rad
                    )
                    # 映射到 Follower 物理宽度范围
                    target_m = WIDTH_MIN + ratio * (WIDTH_MAX - WIDTH_MIN)
                else:
                    target_m = WIDTH_MIN
                
                # 限制到 Follower 物理范围
                target_m = max(WIDTH_MIN, min(WIDTH_MAX, target_m))
                leader_ratio = max(0.0, min(1.0, ratio if self.leader_gripper_max_rad > self.leader_gripper_min_rad else 0.0))
                
            else:
                # 相对控制模式（保持原有逻辑）
                cmd_angle = cmd_value
                cmd_angle = max(FACTR_CMD_MIN_RAD, min(FACTR_CMD_MAX_RAD, cmd_angle))
                
                if FACTR_CMD_MAX_RAD > FACTR_CMD_MIN_RAD:
                    ratio = (cmd_angle - FACTR_CMD_MIN_RAD) / (
                        FACTR_CMD_MAX_RAD - FACTR_CMD_MIN_RAD
                    )
                else:
                    ratio = 0.0
                
                target_m = WIDTH_MIN + ratio * (WIDTH_MAX - WIDTH_MIN)
                leader_ratio = max(0.0, min(1.0, ratio))
            
            # 解析可选的速度/力度指示（优先使用 msg.velocity/msg.effort）
            speed_percent = float(self.default_speed_percent)
            force_percent = float(self.default_force_percent)
            if hasattr(msg, 'velocity') and msg.velocity:
                try:
                    v = float(msg.velocity[0])
                    # 如果用户使用 0-1 的归一化值，则转为百分比
                    if abs(v) <= 1.0:
                        v = v * 100.0
                    speed_percent = max(0.0, min(100.0, v))
                except Exception:
                    pass
            if hasattr(msg, 'effort') and msg.effort:
                try:
                    f = float(msg.effort[0])
                    if abs(f) <= 1.0:
                        f = f * 100.0
                    force_percent = max(0.0, min(100.0, f))
                except Exception:
                    pass

            with self.lock:
                self.last_target_pos = target_m
                self.has_received_command = True  # 标记已收到命令
                self.leader_ratio = leader_ratio
                self.last_speed_percent = speed_percent
                self.last_force_percent = force_percent
                
        except Exception as e:
            self.get_logger().warning(f"收到无效命令: {e}")

    def _check_and_recover_from_error(self) -> bool:
        """检查夹爪错误状态，如果检测到错误则执行恢复。
        
        Returns:
            如果夹爪处于正常状态返回 True，如果检测到错误并尝试恢复返回 False
        """
        current_time = time.time()
        if current_time - self.last_error_check_time < self.error_check_interval:
            return True  # 未到检查时间
        
        self.last_error_check_time = current_time
        
        # 读取详细状态
        status = self.hw.get_status_detailed()
        if not status:
            # 无法读取状态，可能是通信问题
            return True  # 不视为致命错误，继续运行
        
        g_act = status['gACT']
        g_sta = status['gSTA']
        g_flt = status['gFLT']
        
        # 检查是否处于激活状态
        is_activated = (g_sta == 3 and g_act == 1)
        
        # 如果之前是激活的，但现在不是，或者检测到故障
        if self.is_activated and not is_activated:
            self.get_logger().warning(
                f"检测到夹爪失活！状态: gSTA={g_sta}, gACT={g_act}, gFLT={g_flt}"
            )
            self.is_activated = False
            
            # 如果检测到故障，执行恢复
            if g_flt != 0:
                self.get_logger().error(f"检测到夹爪故障 (gFLT={g_flt})，执行紧急恢复...")
                if self.hw.initialize():
                    self.get_logger().info("夹爪恢复成功")
                    self.is_activated = True
                    return True
                else:
                    self.get_logger().error("夹爪恢复失败")
                    return False
        
        # 如果之前未激活，但现在激活了
        if not self.is_activated and is_activated:
            self.get_logger().info("夹爪已激活")
            self.is_activated = True
        
        # 如果检测到故障但仍在激活状态，记录警告
        if g_flt != 0 and is_activated:
            self.get_logger().warning(f"夹爪处于激活状态但检测到故障代码: gFLT={g_flt}")
        
        return True
    
    #主循环: 检查错误 -> 读取状态 -> 发布 -> 写入最新命令。
    def control_loop(self):
        
        # 0. 定期检查错误状态并恢复
        if not self._check_and_recover_from_error():
            # 如果恢复失败，跳过本次循环
            return
        
        # 1. 读取并发布状态
        try:
            pos, current, is_moving = self.hw.get_state()
            # 归一化 follower 开度: 0=闭合,1=张开
            follower_ratio = 0.0
            if WIDTH_MAX > WIDTH_MIN:
                follower_ratio = (pos - WIDTH_MIN) / (WIDTH_MAX - WIDTH_MIN)
            follower_ratio = max(0.0, min(1.0, follower_ratio))
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # 发布两路开度（0-1）
            msg.name = ['leader_gripper_ratio', 'follower_gripper_ratio']
            with self.lock:
                leader_ratio = self.leader_ratio
            msg.position = [leader_ratio, follower_ratio]
            
            self.pub_state.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"读取/发布状态时出错: {e}")

        # 2. 写入命令（如果已更新且夹爪已激活）
        with self.lock:
            target = self.last_target_pos
            has_received = self.has_received_command
        
        # 只有在收到第一个命令后才发送位置命令，避免初始化时意外移动
        if target is not None and self.is_activated and has_received:
            # 优化: 在实际系统中，我们可能只在目标显著改变时
            # 或以低于读取的频率写入。
            # 但是，Modbus RTU 是请求-响应模式。
            # 如果我们共享总线，通常交替进行读/写。
            # 这里如果有目标，每个周期都写入。
            # 注意: Robotiq 规格说明命令在寄存器改变时处理。
            # 持续重写通常没问题，但会消耗带宽。
            try:
                # 使用从命令中读取/默认参数指定的力度和速度
                self.hw.set_target(target, force_percent=self.last_force_percent, speed_percent=self.last_speed_percent)
            except Exception as e:
                self.get_logger().warning(f"写入命令时出错: {e}")
        elif target is not None and not self.is_activated:
            # 有命令但夹爪未激活，记录但不发送命令
            self.get_logger().debug("有命令但夹爪未激活，跳过命令发送")
        elif target is not None and not has_received:
            # 有目标但还未收到第一个命令，等待 Leader 发送命令
            self.get_logger().debug("等待 Leader 夹爪命令...")


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("正在关闭夹爪桥接节点...")
        try:
            node.hw.disconnect()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

