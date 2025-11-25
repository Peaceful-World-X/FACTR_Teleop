# Franka FCI 模式与 FACTR Teleop 通信集成指南

本指南详细说明如何通过 Franka 的 FCI (Franka Control Interface) 模式实现与 FACTR Teleop 系统的通信连接。

## 目录
1. [系统架构概述](#系统架构概述)
2. [通信协议说明](#通信协议说明)
3. [FCI 控制端实现](#fci-控制端实现)
4. [网络配置](#网络配置)
5. [完整实现流程](#完整实现流程)
6. [调试与测试](#调试与测试)

---

## 系统架构概述

### 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                    FACTR Teleop 系统                         │
│  ┌────────────────┐         ┌──────────────────┐            │
│  │  Leader Arm    │────────▶│ factr_teleop_    │            │
│  │  (Dynamixel)   │  读取    │ franka_zmq.py   │            │
│  └────────────────┘  位置    └──────────────────┘            │
│                                     │ ZMQ                    │
│                                     │ Publish                │
└─────────────────────────────────────┼────────────────────────┘
                                      │
                              ZMQ over TCP/IP
                                      │
┌─────────────────────────────────────┼────────────────────────┐
│              Franka 控制端 (您需要实现)                       │
│                                     │                        │
│  ┌──────────────────────────────────▼──────────────────┐    │
│  │  FCI Control Program (C++)                          │    │
│  │  ┌─────────────┐    ┌──────────────┐              │    │
│  │  │ ZMQ Sub     │───▶│ Position     │              │    │
│  │  │ (接收位置)   │    │ Controller   │              │    │
│  │  └─────────────┘    └──────┬───────┘              │    │
│  │                            │                        │    │
│  │  ┌─────────────┐    ┌──────▼────────┐             │    │
│  │  │ ZMQ Pub     │◀───│ State Reader  │             │    │
│  │  │ (发送状态)   │    │ (关节状态/力矩)│             │    │
│  │  └─────────────┘    └───────────────┘             │    │
│  └────────────────────┬────────────────────────────────┘    │
│                       │ libfranka API                       │
│  ┌────────────────────▼────────────────────────────────┐    │
│  │         franka::Robot (FCI 接口)                    │    │
│  └─────────────────────────────────────────────────────┘    │
│                           │                                 │
└───────────────────────────┼─────────────────────────────────┘
                            │ Ethernet
                            ▼
                    ┌───────────────┐
                    │ Franka Robot  │
                    └───────────────┘
```

### 数据流向

1. **命令流** (FACTR → Franka):
   - Leader arm 关节位置 → ZMQ Publisher → ZMQ Subscriber → FCI Position Command → Franka Robot

2. **状态流** (Franka → FACTR):
   - Franka Robot → FCI State Reader → ZMQ Publisher → ZMQ Subscriber → ROS2 Topics

---

## 通信协议说明

### ZMQ 地址配置

在 `global_configs.py` 中定义的 ZMQ 通信地址：

```python
# FACTR 系统的 IP 地址（运行 ROS2 的工作站）
sim_desktop_ip_address = "192.168.40.200"

# Franka 控制端的 IP 地址
franka_right_ip_address = "10.0.10.2"    # 右臂
franka_left_ip_address = "172.16.0.1"    # 左臂

# 右臂 ZMQ 通信端点
franka_right_real_zmq_addresses = {
    # Franka 发送关节状态 (Publisher 在 Franka 端，Subscriber 在 FACTR 端)
    "joint_state_sub":  f"tcp://{franka_right_ip_address}:3099",
    
    # Franka 发送关节力矩 (Publisher 在 Franka 端，Subscriber 在 FACTR 端)
    "joint_torque_sub": f"tcp://{franka_right_ip_address}:3087",
    
    # FACTR 发送位置命令 (Publisher 在 FACTR 端，Subscriber 在 Franka 端)
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:2098",
}

# 左臂 ZMQ 通信端点（配置类似）
franka_left_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_left_ip_address}:5099",
    "joint_torque_sub": f"tcp://{franka_left_ip_address}:5087",
    "joint_pos_cmd_pub": f"tcp://{sim_desktop_ip_address}:4098",
}
```

### 数据格式

所有 ZMQ 消息使用 **NumPy array** 编码为二进制：

```python
# 发送端 (Python)
message = np.array([q1, q2, q3, q4, q5, q6, q7])  # 7 个关节位置/力矩
publisher.send(message.astype(np.float64).tobytes())

# 接收端 (Python)
message = socket.recv()
data = np.frombuffer(message).astype(np.float32)
```

**注意**：
- 发送时使用 `float64` (double)
- 接收时转换为 `float32` (float)
- C++ 端需要相应处理字节序和数据类型

---

## FCI 控制端实现

您需要在 **Franka 控制端** (通常是直接连接到 Franka 机器人的工作站) 实现一个 C++ 程序，使用 `libfranka` 库与机器人通信，同时通过 ZMQ 与 FACTR 系统通信。

### 1. 环境准备

#### 安装 libfranka

```bash
# 安装依赖
sudo apt-get install build-essential cmake git libpoco-dev libeigen3-dev

# 克隆 libfranka
cd ~
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka

# 选择与您的机器人固件兼容的版本
# 例如：git checkout 0.9.0

# 编译安装
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

#### 安装 ZMQ C++ 绑定

```bash
# 安装 ZMQ 库
sudo apt-get install libzmq3-dev

# 安装 cppzmq (C++ 绑定)
cd ~
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build && cd build
cmake ..
sudo make install
```

### 2. FCI 控制程序框架

创建 `franka_factr_bridge.cpp`:

```cpp
#include <iostream>
#include <thread>
#include <atomic>
#include <array>
#include <cstring>
#include <zmq.hpp>

#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/model.h>

// 配置参数
const std::string ROBOT_IP = "172.16.0.2";  // Franka 机器人 IP
const std::string CMD_SUB_ADDRESS = "tcp://192.168.40.200:4098";  // 接收位置命令
const std::string STATE_PUB_ADDRESS = "tcp://*:5099";  // 发布关节状态
const std::string TORQUE_PUB_ADDRESS = "tcp://*:5087"; // 发布关节力矩
const int CONTROL_RATE = 1000;  // 1000 Hz

// 全局变量（使用原子操作保证线程安全）
std::array<double, 7> g_target_position;
std::atomic<bool> g_new_command{false};
std::atomic<bool> g_running{true};

// ZMQ 命令接收线程
void zmqCommandReceiver() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    
    // 连接到 FACTR 系统的命令发布端点
    subscriber.connect(CMD_SUB_ADDRESS);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    subscriber.setsockopt(ZMQ_CONFLATE, 1);  // 只保留最新消息
    
    std::cout << "ZMQ Command Receiver started on " << CMD_SUB_ADDRESS << std::endl;
    
    while (g_running) {
        zmq::message_t message;
        
        // 非阻塞接收，超时 100ms
        if (subscriber.recv(message, zmq::recv_flags::dontwait)) {
            // 解析数据：Python 发送的是 float64 (8 bytes * 7)
            if (message.size() == 7 * sizeof(double)) {
                double* data = static_cast<double*>(message.data());
                for (int i = 0; i < 7; ++i) {
                    g_target_position[i] = data[i];
                }
                g_new_command = true;
            } else {
                std::cerr << "Received invalid message size: " << message.size() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ZMQ 状态发布线程
void zmqStatePublisher(franka::Robot& robot) {
    zmq::context_t context(1);
    zmq::socket_t state_publisher(context, ZMQ_PUB);
    zmq::socket_t torque_publisher(context, ZMQ_PUB);
    
    // 绑定发布端点
    state_publisher.bind(STATE_PUB_ADDRESS);
    torque_publisher.bind(TORQUE_PUB_ADDRESS);
    
    std::cout << "ZMQ State Publisher started on " << STATE_PUB_ADDRESS << std::endl;
    std::cout << "ZMQ Torque Publisher started on " << TORQUE_PUB_ADDRESS << std::endl;
    
    // 等待订阅者连接
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    while (g_running) {
        try {
            // 读取机器人状态
            franka::RobotState state = robot.readOnce();
            
            // 发布关节状态 (位置 + 速度，共 14 个 float32)
            std::array<float, 14> joint_state;
            for (size_t i = 0; i < 7; ++i) {
                joint_state[i] = static_cast<float>(state.q[i]);       // 位置
                joint_state[i + 7] = static_cast<float>(state.dq[i]);  // 速度
            }
            zmq::message_t state_msg(joint_state.data(), joint_state.size() * sizeof(float));
            state_publisher.send(state_msg, zmq::send_flags::dontwait);
            
            // 发布外部力矩 (7 个 float32)
            std::array<float, 7> joint_torque;
            for (size_t i = 0; i < 7; ++i) {
                joint_torque[i] = static_cast<float>(state.tau_ext_hat_filtered[i]);
            }
            zmq::message_t torque_msg(joint_torque.data(), joint_torque.size() * sizeof(float));
            torque_publisher.send(torque_msg, zmq::send_flags::dontwait);
            
        } catch (const franka::Exception& e) {
            std::cerr << "Franka exception in state publisher: " << e.what() << std::endl;
        }
        
        // 发布频率约 100 Hz（可根据需要调整）
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 平滑插值函数（避免突变）
std::array<double, 7> interpolatePosition(
    const std::array<double, 7>& current,
    const std::array<double, 7>& target,
    double alpha = 0.1) {
    
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; ++i) {
        result[i] = current[i] + alpha * (target[i] - current[i]);
    }
    return result;
}

int main(int argc, char** argv) {
    try {
        // 1. 连接机器人
        std::cout << "Connecting to Franka robot at " << ROBOT_IP << "..." << std::endl;
        franka::Robot robot(ROBOT_IP);
        
        // 2. 设置碰撞行为
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // lower_torque_thresholds
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // upper_torque_thresholds
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // lower_torque_thresholds_nominal
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},  // upper_torque_thresholds_nominal
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},        // lower_force_thresholds
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},        // upper_force_thresholds
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},        // lower_force_thresholds_nominal
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}}         // upper_force_thresholds_nominal
        );
        
        // 3. 读取初始状态
        franka::RobotState initial_state = robot.readOnce();
        for (size_t i = 0; i < 7; ++i) {
            g_target_position[i] = initial_state.q[i];
        }
        
        std::cout << "Initial joint positions: ";
        for (const auto& q : g_target_position) {
            std::cout << q << " ";
        }
        std::cout << std::endl;
        
        // 4. 启动 ZMQ 线程
        std::thread cmd_receiver_thread(zmqCommandReceiver);
        std::thread state_publisher_thread(zmqStatePublisher, std::ref(robot));
        
        std::cout << "ZMQ threads started. Waiting for commands..." << std::endl;
        
        // 5. 定义控制回调函数
        std::array<double, 7> current_target = g_target_position;
        
        auto motion_callback = [&](const franka::RobotState& state,
                                   franka::Duration period) -> franka::JointPositions {
            
            // 如果收到新命令，平滑过渡
            if (g_new_command) {
                current_target = interpolatePosition(current_target, g_target_position, 0.05);
                g_new_command = false;
            }
            
            // 返回目标关节位置
            franka::JointPositions output(current_target);
            
            // 可选：添加安全检查
            // if (hasCollision(state)) {
            //     std::cout << "Collision detected! Stopping motion." << std::endl;
            //     return franka::MotionFinished(output);
            // }
            
            return output;
        };
        
        // 6. 启动控制循环
        std::cout << "Starting motion control..." << std::endl;
        robot.control(motion_callback);
        
        // 7. 清理
        g_running = false;
        cmd_receiver_thread.join();
        state_publisher_thread.join();
        
        std::cout << "Program finished successfully." << std::endl;
        
    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        g_running = false;
        return -1;
    }
    
    return 0;
}
```

### 3. CMakeLists.txt

创建 `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.4)
project(franka_factr_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 libfranka
find_package(Franka REQUIRED)

# 查找 ZMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# 查找 cppzmq
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)

# 添加可执行文件
add_executable(franka_factr_bridge franka_factr_bridge.cpp)

# 包含目录
target_include_directories(franka_factr_bridge PRIVATE 
    ${Franka_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${CPPZMQ_INCLUDE_DIR}
)

# 链接库
target_link_libraries(franka_factr_bridge 
    Franka::Franka
    ${ZMQ_LIBRARIES}
    pthread
)
```

### 4. 编译运行

```bash
# 创建项目目录
mkdir -p ~/franka_factr_bridge
cd ~/franka_factr_bridge

# 复制上述文件到该目录

# 编译
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make

# 运行（需要实时权限）
sudo ./franka_factr_bridge
```

---

## 网络配置

### 1. 网络拓扑

```
┌─────────────────────┐              ┌──────────────────────┐
│  FACTR 工作站       │              │  Franka 控制端       │
│  (ROS2 + FACTR)     │              │  (FCI Program)       │
│                     │              │                      │
│  IP: 192.168.40.200 │◄────────────►│  IP: 10.0.10.1       │
│                     │   以太网     │                      │
│  端口: 2098 (Pub)   │              │  端口: 5099 (Pub)    │
│        4098 (Pub)   │              │        5087 (Pub)    │
└─────────────────────┘              └──────────┬───────────┘
                                                 │
                                                 │ 专用网络
                                                 │
                                     ┌───────────▼──────────┐
                                     │  Franka Robot        │
                                     │  IP: 172.16.0.2      │
                                     └──────────────────────┘
```

### 2. IP 地址配置

#### FACTR 工作站

1. 编辑 `global_configs.py`:

```python
sim_desktop_ip_address = "192.168.40.200"  # 修改为您的实际 IP
franka_right_ip_address = "10.0.10.1"      # Franka 控制端 IP
```

2. 确保网络可达：

```bash
# 测试连接
ping 10.0.10.1
```

#### Franka 控制端

1. 配置网络接口：

```bash
# 查看网卡
ip addr

# 假设连接 FACTR 工作站的网卡是 enp0s31f6
sudo ip addr add 10.0.10.1/24 dev enp0s31f6
```

2. 修改程序中的 IP 地址：

```cpp
const std::string ROBOT_IP = "172.16.0.2";  // Franka 机器人的实际 IP
const std::string CMD_SUB_ADDRESS = "tcp://192.168.40.200:4098";  // FACTR 工作站 IP
```

3. 测试网络连接：

```bash
# 测试到 FACTR 工作站
ping 192.168.40.200

# 测试到 Franka 机器人
ping 172.16.0.2
```

### 3. 防火墙配置

确保端口开放：

```bash
# 在 Franka 控制端（如果有防火墙）
sudo ufw allow 5099/tcp
sudo ufw allow 5087/tcp

# 在 FACTR 工作站（如果有防火墙）
sudo ufw allow 2098/tcp
sudo ufw allow 4098/tcp
```

---

## 完整实现流程

### 步骤 1: 准备 FACTR 系统

1. **配置网络地址**:

编辑 `/home/cytoderm/projects/FACTR_Teleop/src/python_utils/python_utils/global_configs.py`:

```python
sim_desktop_ip_address = "192.168.40.200"  # 您的工作站 IP
franka_right_ip_address = "10.0.10.1"      # Franka 控制端 IP
```

2. **重新构建工作空间**:

```bash
cd /home/cytoderm/projects/FACTR_Teleop
colcon build --symlink-install
source install/local_setup.bash
```

### 步骤 2: 实现 Franka 控制端

1. **在 Franka 控制端创建项目**:

```bash
mkdir -p ~/franka_factr_bridge
cd ~/franka_factr_bridge
```

2. **创建源文件**:
   - `franka_factr_bridge.cpp` (如上所示)
   - `CMakeLists.txt` (如上所示)

3. **编译程序**:

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### 步骤 3: 测试通信

1. **启动 Franka 控制端**:

```bash
cd ~/franka_factr_bridge/build
sudo ./franka_factr_bridge
```

应该看到输出：
```
Connecting to Franka robot at 172.16.0.2...
Initial joint positions: ...
ZMQ Command Receiver started on tcp://192.168.40.200:4098
ZMQ State Publisher started on tcp://*:5099
ZMQ Torque Publisher started on tcp://*:5087
Starting motion control...
```

2. **启动 FACTR 系统**:

```bash
cd /home/cytoderm/projects/FACTR_Teleop
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

# 启动遥操作节点
ros2 launch launch/factr_teleop.py
```

3. **验证通信**:

```bash
# 在另一个终端，检查 ROS 话题
ros2 topic list

# 应该看到:
# /franka/right/obs_franka_state
# /franka/right/obs_franka_torque
# /factr_teleop/right/cmd_franka_pos
```

```bash
# 监听 Franka 状态
ros2 topic echo /franka/right/obs_franka_state
```

### 步骤 4: 调整控制参数

根据实际情况调整 `franka_example.yaml`:

```yaml
controller:
  frequency: 500  # 控制频率
  
  torque_feedback:
    enable: True
    gain: 3.0      # 调整力反馈增益
    
  gravity_comp:
    enable: True
    gain: 0.99     # 调整重力补偿增益
```

---

## 调试与测试

### 1. ZMQ 通信测试

#### 测试命令接收（在 Franka 控制端）

创建测试脚本 `test_zmq_sub.cpp`:

```cpp
#include <iostream>
#include <zmq.hpp>

int main() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    
    subscriber.connect("tcp://192.168.40.200:4098");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    
    std::cout << "Listening for commands..." << std::endl;
    
    while (true) {
        zmq::message_t message;
        subscriber.recv(message);
        
        double* data = static_cast<double*>(message.data());
        std::cout << "Received: ";
        for (int i = 0; i < 7; ++i) {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
    }
    
    return 0;
}
```

#### 测试状态发送（在 FACTR 工作站）

创建测试脚本 `test_zmq_receive.py`:

```python
#!/usr/bin/env python3
import zmq
import numpy as np
import time

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://10.0.10.1:5099")
socket.setsockopt(zmq.SUBSCRIBE, b'')

print("Listening for robot state...")

while True:
    message = socket.recv()
    data = np.frombuffer(message, dtype=np.float32)
    print(f"Received state: {data[:7]}")  # 前 7 个是位置
    time.sleep(0.1)
```

```bash
python3 test_zmq_receive.py
```

### 2. 常见问题排查

#### 问题 1: 连接超时

```
Error: Connection timeout
```

**解决方案**:
- 检查网络连通性: `ping <target_ip>`
- 检查防火墙设置
- 验证 IP 地址和端口是否正确

#### 问题 2: 收不到数据

**检查列表**:
1. ZMQ 绑定地址是否正确
2. Publisher 是否在 Subscriber 之前启动
3. 是否设置了 `ZMQ_SUBSCRIBE` 选项

```cpp
// 必须设置订阅过滤器
subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
```

#### 问题 3: Franka 连接失败

```
franka::NetworkException: Connection to robot failed
```

**解决方案**:
- 确保机器人已解锁 (通过 Desk 界面)
- 检查机器人 IP 地址
- 确认没有其他程序占用 FCI 接口
- 检查网线连接

#### 问题 4: 机器人运动不平滑

**优化措施**:
1. 增加插值系数:
```cpp
current_target = interpolatePosition(current_target, g_target_position, 0.1);  // 增大 alpha
```

2. 添加速度和加速度限制:
```cpp
// 使用 libfranka 的 limitRate 功能
output = franka::limitRate(franka::kMaxJointVelocity, 
                           franka::kMaxJointAcceleration,
                           franka::kMaxJointJerk,
                           output, last_output);
```

### 3. 性能监控

添加性能监控代码：

```cpp
// 在控制回调中
static int counter = 0;
static auto last_time = std::chrono::steady_clock::now();

counter++;
if (counter % 1000 == 0) {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    double freq = 1000.0 / (duration / 1000.0);
    std::cout << "Control frequency: " << freq << " Hz" << std::endl;
    last_time = now;
}
```

### 4. 安全措施

添加安全检查：

```cpp
bool isSafePosition(const std::array<double, 7>& position) {
    const std::array<double, 7> q_min = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const std::array<double, 7> q_max = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    
    for (size_t i = 0; i < 7; ++i) {
        if (position[i] < q_min[i] || position[i] > q_max[i]) {
            return false;
        }
    }
    return true;
}

// 在控制回调中
if (!isSafePosition(current_target)) {
    std::cerr << "Unsafe position detected!" << std::endl;
    return franka::MotionFinished(franka::JointPositions(state.q));
}
```

---

## 高级功能

### 1. 阻抗控制

如果需要实现更高级的力控制：

```cpp
auto impedance_callback = [&](const franka::RobotState& state,
                              franka::Duration period) -> franka::Torques {
    // 计算目标力矩
    std::array<double, 7> tau_d;
    
    // 位置误差
    std::array<double, 7> q_error;
    for (size_t i = 0; i < 7; ++i) {
        q_error[i] = g_target_position[i] - state.q[i];
    }
    
    // PD 控制
    const std::array<double, 7> kp = {600, 600, 600, 600, 250, 150, 50};
    const std::array<double, 7> kd = {50, 50, 50, 50, 30, 25, 15};
    
    for (size_t i = 0; i < 7; ++i) {
        tau_d[i] = kp[i] * q_error[i] - kd[i] * state.dq[i];
    }
    
    // 添加重力补偿
    franka::Model model = robot.loadModel();
    std::array<double, 7> gravity = model.gravity(state);
    
    for (size_t i = 0; i < 7; ++i) {
        tau_d[i] += gravity[i];
    }
    
    return tau_d;
};

robot.control(impedance_callback);
```

### 2. 笛卡尔空间控制

```cpp
#include <Eigen/Dense>

// 使用 Franka 模型计算雅可比
franka::Model model = robot.loadModel();
std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

// 转换为 Eigen 矩阵
Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

// 计算笛卡尔空间误差
Eigen::Vector<double, 6> x_error;  // 位置和姿态误差
Eigen::Vector<double, 7> q_dot_target = jacobian.transpose() * x_error;  // 伪逆

// 积分得到位置命令
for (size_t i = 0; i < 7; ++i) {
    g_target_position[i] += q_dot_target[i] * period.toSec();
}
```

---

## 总结

通过本指南，您应该能够：

1. ✅ 理解 FACTR 与 Franka 之间的通信架构
2. ✅ 配置网络和 ZMQ 通信
3. ✅ 实现基于 libfranka 的 FCI 控制程序
4. ✅ 测试和调试整个系统
5. ✅ 实现安全和高级控制功能

**关键要点**:
- FACTR 通过 ZMQ 发送位置命令，接收状态和力矩
- Franka 控制端通过 libfranka 实现 FCI 接口
- 使用多线程分离 ZMQ 通信和 FCI 控制循环
- 添加插值和安全检查确保平滑和安全运行

**下一步**:
- 根据您的具体硬件配置修改 IP 地址
- 调整控制参数以获得最佳性能
- 添加您特定任务需要的功能

如有问题，请参考：
- [libfranka 文档](https://frankaemika.github.io/docs/)
- [ZMQ 指南](https://zguide.zeromq.org/)
- [FACTR 项目页面](https://jasonjzliu.com/factr/)
