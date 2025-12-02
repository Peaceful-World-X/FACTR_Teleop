#!/usr/bin/env python3
"""
Simple script to scan for Robotiq Grippers on serial ports.
It tries common baudrates and Slave IDs.
"""

import sys
import time
try:
    from pymodbus.client.sync import ModbusSerialClient
except ImportError:
    try:
        from pymodbus.client import ModbusSerialClient
    except ImportError:
        print("Error: pymodbus is not installed. Please install it (pip install pymodbus).")
        sys.exit(1)

def scan():
    # Common configurations
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyACM0']
    bauds = [115200, 9600, 19200, 57600]
    ids = [9, 1, 2, 11, 0]  # 9 is factory default

    print("=== Robotiq Gripper Scanner ===")
    print("Scanning ports, baudrates, and slave IDs...")

    for port in ports:
        # Check if port exists by trying to open it briefly
        try:
            client_check = ModbusSerialClient(method='rtu', port=port)
            if not client_check.connect():
                continue # Port doesn't exist or is locked
            client_check.close()
        except:
            continue

        print(f"\nChecking Port: {port}")
        
        for baud in bauds:
            # Initialize client for this port/baud
            client = ModbusSerialClient(
                method='rtu',
                port=port,
                stopbits=1,
                bytesize=8,
                parity='N',
                baudrate=baud,
                timeout=0.1 # Fast timeout for scanning
            )
            
            if not client.connect():
                continue
                
            print(f"  Baudrate {baud}: Testing IDs {ids}...", end='', flush=True)
            
            for unit_id in ids:
                # Try to read Status Register (0x07D0 / 2000)
                # Reading 1 register is enough to check existence
                try:
                    result = client.read_holding_registers(0x07D0, 1, unit=unit_id)
                    if result and not result.isError():
                        print(f"\n\n>>> FOUND GRIPPER! <<<")
                        print(f"    Port:     {port}")
                        print(f"    Baudrate: {baud}")
                        print(f"    Slave ID: {unit_id}")
                        print(">>> ---------------- <<<")
                        client.close()
                        return
                except Exception:
                    pass
            
            print(" Done.")
            client.close()

    print("\nScan complete. No gripper found with standard configurations.")
    print("Check power, cabling, and USB permissions (sudo chmod 666 /dev/ttyUSB*).")

if __name__ == "__main__":
    scan()

