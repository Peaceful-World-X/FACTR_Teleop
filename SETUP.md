# FACTR Teleop 项目初始化指南

本文档将指导您完成项目的完整初始化设置。

## 前置要求

1. **ROS 2** - 已检测到您安装了 ROS 2 Foxy
2. **Python 3** - 已检测到 Python 3.12.7
3. **Ubuntu/Linux** 系统

## 初始化步骤

### 1. 设置 ROS 2 环境

每次打开新终端时，需要 source ROS 2 环境：

```bash
source /opt/ros/foxy/setup.bash
```

### 2. 安装系统依赖

#### 安装 Pinocchio（机器人动力学库）

```bash
sudo apt update
sudo apt install ros-foxy-pinocchio
```

如果上述命令失败，可以尝试通过 pip 安装：

```bash
pip install --user pin
```

### 3. 安装 Python 依赖

#### 安装基础 Python 包

```bash
pip install --user pyserial numpy pyyaml zmq
```

#### 安装 Dynamixel SDK

```bash
cd src/factr_teleop/factr_teleop/dynamixel/python
pip install --user -e .
cd ../../../../..
```

### 4. 构建 ROS 2 工作空间

在项目根目录执行：

```bash
colcon build --symlink-install
```

这将创建以下目录：
- `build/` - 构建文件
- `install/` - 安装文件
- `log/` - 日志文件

### 5. Source 工作空间

构建完成后，source 工作空间：

```bash
source install/local_setup.bash
```

### 6. 配置 USB 设备权限（可选，用于 Dynamixel）

如果您需要使用 Dynamixel 设备，需要将用户添加到 `dialout` 组：

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

**注意**：添加用户到组后，需要重新登录或运行 `newgrp dialout` 才能生效。

### 7. 设置 USB 延迟定时器（用于 Dynamixel）

如果使用 Dynamixel 设备，需要设置 USB 延迟定时器为 1：

```bash
# 1. 查找您的 USB 设备
ls /dev/serial/by-id/

# 2. 解析设备路径（替换 <device-name> 为实际设备名）
readlink -f /dev/serial/by-id/<device-name>

# 3. 设置延迟定时器（替换 ttyUSBx 为实际设备，如 ttyUSB0）
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSBx/latency_timer
```

**注意**：每次重新插拔 USB 设备后，都需要重新设置延迟定时器。

## 验证安装

### 测试 ROS 2 环境

```bash
ros2 --help
```

### 测试 Python 依赖

```bash
python3 -c "import numpy, yaml, zmq, pinocchio; print('所有依赖已安装')"
```

### 测试 Dynamixel SDK

```bash
python3 -c "from dynamixel_sdk import *; print('Dynamixel SDK 已安装')"
```

### 测试项目构建

```bash
ros2 pkg list | grep factr
```

应该看到：
- `factr_teleop`
- `bc`
- `cameras`
- `python_utils`

## 快速启动脚本

为了方便，您可以创建一个启动脚本 `setup_env.sh`：

```bash
#!/bin/bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
```

然后每次使用时：

```bash
source setup_env.sh
```

## 常见问题

### 问题 1: `colcon: command not found`

**解决方案**：
```bash
sudo apt install python3-colcon-common-extensions
```

### 问题 2: `No module named 'dynamixel_sdk'`

**解决方案**：
```bash
cd src/factr_teleop/factr_teleop/dynamixel/python
pip install --user -e .
```

### 问题 3: `Permission denied: /dev/ttyUSB0`

**解决方案**：
```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

### 问题 4: `No module named 'pinocchio'`

**解决方案**：
```bash
sudo apt install ros-foxy-pinocchio
# 或
pip install --user pin
```

## 下一步

初始化完成后，您可以：

1. **运行重力补偿演示**：
   ```bash
   ros2 launch launch/factr_teleop_grav_comp_demo.py
   ```

2. **检查关节位置**：
   ```bash
   python3 launch/check_joint_positions.py
   ```

3. **查看详细文档**：
   - 主 README: `README.md`
   - Launch 文件说明: `launch/README.md`
   - FACTR Teleop 详细说明: `src/factr_teleop/README.md`

