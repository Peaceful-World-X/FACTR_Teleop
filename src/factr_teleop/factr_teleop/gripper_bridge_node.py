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
DEFAULT_PORT = "/dev/ttyUSB2"
DEFAULT_BAUDRATE = 115200
DEFAULT_SLAVE_ID = 0x0009

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

    def __init__(self, port: str = DEFAULT_PORT, slave_id: int = DEFAULT_SLAVE_ID, baudrate: int = DEFAULT_BAUDRATE):
        """使用原始 Modbus RTU 通过串口初始化硬件接口。

        Args:
            port: 串口路径（例如 '/dev/ttyUSB2'）。
            slave_id: Modbus 从站 ID（默认 9）。
            baudrate: 串口波特率（默认 115200）。
        """
        self.port = port
        self.slave_id = slave_id
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.logger = logging.getLogger("RobotiqHW")

    def connect(self) -> bool:
        """打开底层串口。"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
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

    def _compute_crc(self, data: bytes) -> int:
        """计算 Modbus RTU CRC16（低位在前）。"""
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

    def initialize(self) -> bool:
        """运行最小化的夹爪激活序列。

        此实现匹配已知可用的用户命令:

        - 09 10 03 E8 00 03 06 01 00 00 00 FF 96 B3 7F

        对应参数:
        - Act = 1, GTO = 0, Pos = 0, Speed = 255, Force = 150
        """
        with self.lock:
            # 单一激活命令，不进行前置复位。这反映了用户通过串口终端验证的行为。
            self.logger.info("激活夹爪（单一激活命令）...")
            if not self._write_command(act=1, gto=0, r_pr=0, r_sp=255, r_fr=150):
                self.logger.error("夹爪激活 Modbus 命令失败。")
                return False

            # 可选: 尝试通过状态寄存器确认激活，但如果命令本身已成功发送，
            # 不将缺少响应视为致命错误。
            timeout = 2.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self._read_status_raw()
                if status:
                    g_sta = (status["gACT_byte"] >> 4) & 0x03
                    if g_sta == 3:
                        self.logger.info("夹爪激活完成（gSTA=3）。")
                        return True
                time.sleep(0.1)

            self.logger.info("夹爪激活命令已发送（无明确的 gSTA 确认）。")
            return True

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
            REG_FORCE_MIN, REG_FORCE_MAX
        )
        
        reg_speed = self._map_to_register(
            speed_percent, 
            0.0, 100.0, 
            REG_SPEED_MIN, REG_SPEED_MAX
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

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        slave_id = self.get_parameter('slave_id').value
        self.poll_rate = self.get_parameter('poll_rate').value

        self.get_logger().info(f"初始化夹爪桥接节点，端口: {port} (ID: {slave_id}, 波特率: {baudrate})...")

        # 初始化硬件
        self.hw = RobotiqGripperHardware(port, slave_id, baudrate)
        if not self.hw.connect():
            self.get_logger().error("无法连接到夹爪串口。")
            # 我们不在这里退出以保持节点存活用于诊断，
            # 但功能将失败。
            # 或者，可以使用 sys.exit(1)
        else:
            self.get_logger().info("串口已连接。初始化夹爪...")
            if self.hw.initialize():
                self.get_logger().info("夹爪初始化成功。")
            else:
                self.get_logger().error("夹爪初始化失败。")

        # ROS 订阅者
        self.sub_cmd = self.create_subscription(
            JointState,
            '/factr_teleop/right/cmd_gripper_pos',
            self.cmd_callback,
            1
        )
        self.get_logger().info("已订阅 /factr_teleop/right/cmd_gripper_pos")

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

    def cmd_callback(self, msg: JointState):
        """夹爪命令回调函数。
        
        期望 msg.position[0] 为 FACTR 侧夹爪角度（弧度），其中:

        - 0.0 弧度 => 完全闭合
        - FACTR_CMD_MAX_RAD（默认 0.8） => 完全张开

        角度首先映射到 0.0–1.0 百分比，然后映射到
        物理 Robotiq 开口宽度（0.0–0.085 米）。
        """
        if not msg.position:
            return

        try:
            cmd_angle = float(msg.position[0])

            # 限制到配置的 FACTR 命令范围。
            cmd_angle = max(FACTR_CMD_MIN_RAD, min(FACTR_CMD_MAX_RAD, cmd_angle))

            # 归一化到 0.0–1.0 百分比。
            if FACTR_CMD_MAX_RAD > FACTR_CMD_MIN_RAD:
                ratio = (cmd_angle - FACTR_CMD_MIN_RAD) / (
                    FACTR_CMD_MAX_RAD - FACTR_CMD_MIN_RAD
                )
            else:
                ratio = 0.0

            # 将百分比映射到 Robotiq 物理开口宽度。
            target_m = WIDTH_MIN + ratio * (WIDTH_MAX - WIDTH_MIN)
            
            with self.lock:
                self.last_target_pos = target_m
        except Exception as e:
            self.get_logger().warning(f"收到无效命令: {e}")

    def control_loop(self):
        """主循环: 读取状态 -> 发布 -> 写入最新命令。"""
        # 1. 读取并发布状态
        try:
            pos, current, is_moving = self.hw.get_state()
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['gripper_finger_joint']
            msg.position = [pos]
            msg.effort = [current]  # 使用 effort 字段表示电流（安培）
            # msg.velocity 如需要可以估算，但硬件不直接提供
            
            self.pub_state.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"读取/发布状态时出错: {e}")

        # 2. 写入命令（如果已更新）
        with self.lock:
            target = self.last_target_pos
        
        if target is not None:
            # 优化: 在实际系统中，我们可能只在目标显著改变时
            # 或以低于读取的频率写入。
            # 但是，Modbus RTU 是请求-响应模式。
            # 如果我们共享总线，通常交替进行读/写。
            # 这里如果有目标，每个周期都写入。
            # 注意: Robotiq 规格说明命令在寄存器改变时处理。
            # 持续重写通常没问题，但会消耗带宽。
            try:
                # 默认力度/速度
                self.hw.set_target(target, force_percent=50.0, speed_percent=100.0)
            except Exception as e:
                self.get_logger().warning(f"写入命令时出错: {e}")


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

