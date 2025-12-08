#!/usr/bin/env python3
"""
相机序列号检测脚本

该脚本用于检测系统中连接的 ZED 和 RealSense 相机的序列号。
使用方法: python3 check_camera_serials.py
"""

def check_realsense_serial():
    """检测 RealSense 相机序列号"""
    print("\n🔍 检测 RealSense 相机...")
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) > 0:
            print(f"✅ 找到 {len(devices)} 个 RealSense 相机:")
            for i, dev in enumerate(devices):
                serial = dev.get_info(rs.camera_info.serial_number)
                name = dev.get_info(rs.camera_info.name)
                firmware = dev.get_info(rs.camera_info.firmware_version)
                print(f"  {i+1}. {name}")
                print(f"     序列号: {serial}")
                print(f"     固件版本: {firmware}")
        else:
            print("❌ 未找到 RealSense 相机")

    except ImportError:
        print("❌ pyrealsense2 未安装")
        print("   安装命令: pip install pyrealsense2")
    except Exception as e:
        print(f"❌ RealSense 检测出错: {e}")

def check_zed_serial():
    """检测 ZED 相机序列号"""
    print("\n🔍 检测 ZED 相机...")
    try:
        import pyzed.sl as sl
        cameras = sl.Camera.get_device_list()

        if cameras:
            print(f"✅ 找到 {len(cameras)} 个 ZED 相机:")
            for i, cam in enumerate(cameras):
                print(f"  {i+1}. 序列号: {cam.serial_number}")
                print(f"     型号: {cam.camera_model}")
        else:
            print("❌ 未找到 ZED 相机")

    except ImportError:
        print("❌ pyzed 未安装")
        print("   安装命令: pip install pyzed")
    except Exception as e:
        print(f"❌ ZED 检测出错: {e}")

def main():
    print("📷 相机序列号检测工具")
    print("=" * 40)

    check_realsense_serial()
    check_zed_serial()

    print("\n💡 提示:")
    print("   - 如果相机未被检测到，请检查 USB 连接")
    print("   - 确保相机驱动已正确安装")
    print("   - RealSense 需要 librealsense SDK")
    print("   - ZED 需要 ZED SDK")

if __name__ == "__main__":
    main()
