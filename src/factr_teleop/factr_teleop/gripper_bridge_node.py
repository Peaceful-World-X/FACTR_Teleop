#!/usr/bin/env python3
"""Robotiq 2F-85 Gripper ROS 2 Bridge.

This node handles communication between ROS 2 topics and the physical Robotiq 2F-85
gripper via Modbus RTU. It subscribes to gripper commands and publishes the
current gripper state.

Technical Reference: gripper_technical_spec.md
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

# --- Configuration Constants ---

# Serial Connection
DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_BAUDRATE = 115200
DEFAULT_SLAVE_ID = 0x0009

# Modbus Registers (from spec)
REG_CMD_START = 0x03E8    # 1000: Action Request
REG_STAT_START = 0x07D0   # 2000: Gripper Status

# Physical limits (meters)
# Spec: 0.0m (Closed) ~ 0.085m (Open)
# Robotiq Register: 0(Open) ~ 255(Closed)
WIDTH_MIN = 0.0
WIDTH_MAX = 0.085

# Register limits (from spec section 8.3 & 10)
# Spec: 0 (Open) ~ 255 (Closed)
# NOTE: This mapping is inverted relative to standard "position"
REG_POS_OPEN = 3
REG_POS_CLOSED = 230
REG_SPEED_MIN = 0
REG_SPEED_MAX = 255
REG_FORCE_MIN = 0
REG_FORCE_MAX = 255

# Control Flags
BIT_ACT = 0   # Activation
BIT_GTO = 3   # Go To
BIT_ATR = 4   # Auto-Release

# Status Flags
BIT_gACT = 0
BIT_gGTO = 3
# gSTA is bits 4-5, gOBJ is bits 6-7


class RobotiqGripperHardware:
    """Handles low-level Modbus RTU communication with the Robotiq 2F-85 gripper."""

    def __init__(self, port: str = DEFAULT_PORT, slave_id: int = DEFAULT_SLAVE_ID, baudrate: int = DEFAULT_BAUDRATE):
        """Initializes the hardware interface using raw Modbus RTU over serial.

        Args:
            port: The serial port path (e.g., '/dev/ttyUSB0').
            slave_id: The Modbus Slave ID (default 9).
            baudrate: Serial baudrate (default 115200).
        """
        self.port = port
        self.slave_id = slave_id
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.logger = logging.getLogger("RobotiqHW")

    def connect(self) -> bool:
        """Opens the underlying serial port."""
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
        """Closes the connection."""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except serial.SerialException:
                # Ignore errors on shutdown.
                pass

    def _compute_crc(self, data: bytes) -> int:
        """Computes Modbus RTU CRC16 (LSB first)."""
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
        """Maps a physical value (meters) to a register value linearly.
        
        Args:
            value: Target width in meters (0.0 = closed, 0.085 = open)
            phy_min: Min physical width (0.0)
            phy_max: Max physical width (0.085)
            reg_open: Register value at Open state (0)
            reg_closed: Register value at Closed state (255)
        """
        # Clamp input
        value = max(phy_min, min(phy_max, value))
        
        # Linear mapping: 
        # ratio = (value - phy_min) / (phy_max - phy_min)
        # reg_val = reg_closed + ratio * (reg_open - reg_closed)
        
        ratio = (value - phy_min) / (phy_max - phy_min)
        reg_val = int(reg_closed + ratio * (reg_open - reg_closed))
        
        return max(0, min(255, reg_val))

    def _map_from_register(self, reg_val: int, phy_min: float, phy_max: float, 
                          reg_open: int, reg_closed: int) -> float:
        """Maps a register value to a physical value linearly."""
        reg_val = max(0, min(255, reg_val))
        
        # ratio = (reg_val - reg_closed) / (reg_open - reg_closed)
        # value = phy_min + ratio * (phy_max - phy_min)
        
        if reg_open == reg_closed:
            return phy_min
            
        ratio = (reg_val - reg_closed) / (reg_open - reg_closed)
        return phy_min + ratio * (phy_max - phy_min)

    def initialize(self) -> bool:
        """Runs the minimal gripper activation sequence.

        This implementation matches the known-working user command:

        - 09 10 03 E8 00 03 06 01 00 00 00 FF 96 B3 7F

        Which corresponds to:
        - Act = 1, GTO = 0, Pos = 0, Speed = 255, Force = 150
        """
        with self.lock:
            # Single activation command, no prior reset. This mirrors the
            # behavior verified by the user via a serial terminal.
            self.logger.info("Activating gripper (single activation command)...")
            if not self._write_command(act=1, gto=0, r_pr=0, r_sp=255, r_fr=150):
                self.logger.error("Gripper activation Modbus command failed.")
                return False

            # Optional: try to confirm activation via status register, but do
            # not treat a missing response as fatal if the command itself
            # went out successfully.
            timeout = 2.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self._read_status_raw()
                if status:
                    g_sta = (status["gACT_byte"] >> 4) & 0x03
                    if g_sta == 3:
                        self.logger.info("Gripper activation completed (gSTA=3).")
                        return True
                time.sleep(0.1)

            self.logger.info("Gripper activation command sent (no explicit gSTA confirmation).")
            return True

    def _write_command(self, act: int, gto: int, r_pr: int, r_sp: int, r_fr: int) -> bool:
        """Writes the 6-byte command to the gripper registers.

        Args:
            act: 0 or 1 (Activation)
            gto: 0 or 1 (Go To)
            r_pr: 0-255 (Position Request)
            r_sp: 0-255 (Speed)
            r_fr: 0-255 (Force)
        
        Returns:
            True if successful.
        """
        if self.serial is None or not self.serial.is_open:
            self.logger.error("Serial port is not open; cannot write command.")
            return False

        # Construct bytes (matches user-validated frames):
        # Reg 1000: [Action Request, Reserved]
        # Reg 1001: [Reserved, Position Request]
        # Reg 1002: [Speed, Force]
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

        # Build Modbus RTU frame manually:
        # [id][func=0x10][addr_hi][addr_lo][qty_hi][qty_lo][byte_count][data...][crc_lo][crc_hi]
        pdu = bytearray()
        pdu.append(self.slave_id & 0xFF)
        pdu.append(0x10)  # Write Multiple Registers
        pdu.extend(struct.pack(">H", REG_CMD_START))
        pdu.extend(struct.pack(">H", 3))  # 3 registers
        pdu.append(6)  # 6 data bytes
        pdu.extend([byte0, byte1, byte2, byte3, byte4, byte5])

        crc = self._compute_crc(bytes(pdu))
        frame = bytes(pdu) + struct.pack("<H", crc)

        try:
            # Clear any stale input, send frame, and read echo response.
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            self.serial.flush()

            # Expected response example observed by user:
            # 09 10 03 E8 00 03 01 30  (8 bytes)
            resp = self.serial.read(8)
            if len(resp) != 8:
                self.logger.warning(
                    f"Modbus write: expected 8-byte response, got {len(resp)} bytes."
                )
                return False

            # Basic sanity checks: unit id, function code, CRC
            data_no_crc = resp[:-2]
            crc_bytes = resp[-2:]
            expected_crc = self._compute_crc(data_no_crc)
            recv_crc = int.from_bytes(crc_bytes, byteorder="little")
            if expected_crc != recv_crc:
                self.logger.warning("Modbus write: CRC mismatch in response.")
                return False

            if data_no_crc[0] != (self.slave_id & 0xFF) or data_no_crc[1] != 0x10:
                self.logger.warning("Modbus write: unexpected unit id or function code.")
                return False

            return True
        except serial.SerialException as exc:
            self.logger.warning(f"Exception during write: {exc}")
            return False

    def _read_status_raw(self) -> Optional[dict]:
        """Reads the status registers and returns raw bytes/values."""
        if self.serial is None or not self.serial.is_open:
            self.logger.error("Serial port is not open; cannot read status.")
            return None

        # Build Modbus RTU frame: Read Holding Registers
        # [id][func=0x03][addr_hi][addr_lo][qty_hi][qty_lo][crc_lo][crc_hi]
        try:
            pdu = bytearray()
            pdu.append(self.slave_id & 0xFF)
            pdu.append(0x03)  # Read Holding Registers
            pdu.extend(struct.pack(">H", REG_STAT_START))
            pdu.extend(struct.pack(">H", 3))  # 3 registers => 6 data bytes

            crc = self._compute_crc(bytes(pdu))
            frame = bytes(pdu) + struct.pack("<H", crc)

            self.serial.reset_input_buffer()
            self.serial.write(frame)
            self.serial.flush()

            # Response should be: [id][func=0x03][byte_count=6][data(6)][crc(2)]
            header = self.serial.read(3)
            if len(header) != 3:
                self.logger.warning(
                    f"Modbus read: expected 3-byte header, got {len(header)} bytes."
                )
                return None

            unit_id = header[0]
            func = header[1]
            byte_count = header[2]

            if unit_id != (self.slave_id & 0xFF) or func != 0x03:
                self.logger.warning("Modbus read: unexpected unit id or function code.")
                # Continue but mark as failure
                return None

            if byte_count != 6:
                self.logger.warning(
                    f"Modbus read: unexpected byte count {byte_count}, expected 6."
                )
                return None

            rest = self.serial.read(byte_count + 2)
            if len(rest) != byte_count + 2:
                self.logger.warning(
                    f"Modbus read: expected {byte_count + 2} bytes, got {len(rest)}."
                )
                return None

            data = rest[:byte_count]
            crc_bytes = rest[byte_count:]

            expected_crc = self._compute_crc(header + data)
            recv_crc = int.from_bytes(crc_bytes, byteorder="little")
            if expected_crc != recv_crc:
                self.logger.warning("Modbus read: CRC mismatch in response.")
                return None

            if len(data) != 6:
                self.logger.warning("Modbus read: status data length mismatch.")
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
            self.logger.warning(f"Exception during read: {exc}")
            return None

    def get_state(self) -> Tuple[float, float, bool]:
        """Reads gripper state.

        Returns:
            Tuple of (position_meters, current_amps, is_moving)
        """
        with self.lock:
            raw = self._read_status_raw()
            if not raw:
                return 0.0, 0.0, False
            
            # Parse Position (gPO)
            # Map: Register 0(Open) -> 0.085m, 255(Closed) -> 0.0m
            pos_m = self._map_from_register(
                raw['gPO'], 
                WIDTH_MIN, WIDTH_MAX, 
                REG_POS_OPEN, REG_POS_CLOSED
            )

            # Parse Current (gCU)
            # Spec 4.5: 0.1 A / unit
            current_a = raw['gCU'] * 0.1

            # Parse Motion Status (gOBJ)
            # gOBJ (bits 6-7): 0=Moving, 1/2=Stopped
            g_obj = (raw['gACT_byte'] >> 6) & 0x03
            is_moving = (g_obj == 0)

            return pos_m, current_a, is_moving

    def set_target(self, width_m: float, force_percent: float = 50.0, speed_percent: float = 100.0):
        """Sends a move command to the gripper.

        Args:
            width_m: Target opening width in meters.
            force_percent: Grip force (0-100).
            speed_percent: Grip speed (0-100).
        """
        # Convert to registers
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
            # Always send Act=1, Gto=1 when moving
            self._write_command(1, 1, reg_pos, reg_speed, reg_force)


class GripperBridgeNode(Node):
    """ROS 2 Node bridging ROS topics to the Robotiq Gripper."""

    def __init__(self):
        super().__init__('gripper_bridge_node')
        
        # Declare Parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('slave_id', DEFAULT_SLAVE_ID)
        self.declare_parameter('poll_rate', 50.0)  # Hz

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        slave_id = self.get_parameter('slave_id').value
        self.poll_rate = self.get_parameter('poll_rate').value

        self.get_logger().info(f"Initializing Gripper Bridge on {port} (ID: {slave_id}, Baud: {baudrate})...")

        # Initialize Hardware
        self.hw = RobotiqGripperHardware(port, slave_id, baudrate)
        if not self.hw.connect():
            self.get_logger().error("Could not connect to gripper serial port.")
            # We don't exit here to keep the node alive for diagnostics, 
            # but functionalities will fail.
            # Alternatively, sys.exit(1)
        else:
            self.get_logger().info("Serial connected. Initializing gripper...")
            if self.hw.initialize():
                self.get_logger().info("Gripper initialized successfully.")
            else:
                self.get_logger().error("Gripper initialization failed.")

        # ROS Subscribers
        self.sub_cmd = self.create_subscription(
            JointState,
            '/factr_teleop/right/cmd_gripper_pos',
            self.cmd_callback,
            1
        )
        self.get_logger().info("Subscribed to /factr_teleop/right/cmd_gripper_pos")

        # ROS Publishers
        self.pub_state = self.create_publisher(
            JointState,
            '/bridge/obs_gripper_state',
            10
        )

        # Control Loop Timer
        self.timer = self.create_timer(1.0 / self.poll_rate, self.control_loop)

        # Internal State
        self.last_target_pos = None
        self.lock = threading.Lock()

    def cmd_callback(self, msg: JointState):
        """Callback for gripper commands.
        
        Expects msg.position[0] to be the target width in meters.
        """
        if not msg.position:
            return

        try:
            target_m = float(msg.position[0])
            
            # Basic safety clamping
            target_m = max(WIDTH_MIN, min(WIDTH_MAX, target_m))
            
            with self.lock:
                self.last_target_pos = target_m
        except Exception as e:
            self.get_logger().warning(f"Invalid command received: {e}")

    def control_loop(self):
        """Main loop: Read state -> Publish -> Write latest command."""
        # 1. Read and Publish State
        try:
            pos, current, is_moving = self.hw.get_state()
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['gripper_finger_joint']
            msg.position = [pos]
            msg.effort = [current]  # Using effort field for current (Amps)
            # msg.velocity can be estimated if needed, but not provided by HW directly
            
            self.pub_state.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Error reading/publishing state: {e}")

        # 2. Write Command (if updated)
        with self.lock:
            target = self.last_target_pos
        
        if target is not None:
            # Optimization: In a real system, we might only write if target changed 
            # significantly or at a lower rate than reading. 
            # However, Modbus RTU is request-response.
            # If we share the bus, we usually alternate Read/Write.
            # Here we write every cycle if we have a target.
            # Note: Robotiq spec says command is processed when register changes.
            # Constant rewriting is generally okay but consumes bandwidth.
            try:
                # Default force/speed
                self.hw.set_target(target, force_percent=50.0, speed_percent=100.0)
            except Exception as e:
                self.get_logger().warning(f"Error writing command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down gripper bridge...")
        try:
            node.hw.disconnect()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

