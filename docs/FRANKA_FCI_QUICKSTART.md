# Franka FCI 快速入门指南

## 概述

本指南提供最简化的步骤，帮助您快速实现 FACTR Teleop 与 Franka 机器人的 FCI 通信。

## 系统架构简图

```
FACTR 工作站 (Python/ROS2)  ←→  Franka 控制端 (C++/libfranka)  →  Franka 机器人
    ZMQ 发送命令                      ZMQ 接收命令              FCI 接口
    ZMQ 接收状态                      ZMQ 发送状态
```

## 前置条件

### FACTR 工作站端
- ✅ ROS 2 已安装
- ✅ FACTR Teleop 已构建
- ✅ Python ZMQ 已安装

### Franka 控制端
- ✅ libfranka 已安装
- ✅ ZMQ C++ 库已安装
- ✅ 与 Franka 机器人网络连接正常

## 3 步快速设置

### 步骤 1: 配置 IP 地址

编辑 `src/python_utils/python_utils/global_configs.py`:

```python
# 您的 FACTR 工作站 IP
sim_desktop_ip_address = "192.168.40.200"

# Franka 控制端 IP（运行 FCI 程序的机器）
franka_right_ip_address = "10.0.10.1"  
franka_left_ip_address = "172.16.0.1"   # 如果有左臂
```

**重新构建**:
```bash
cd /home/cytoderm/projects/FACTR_Teleop
colcon build --symlink-install
```

### 步骤 2: 创建 Franka 控制程序

在 Franka 控制端创建最小化程序 `franka_zmq_bridge.cpp`:

```cpp
#include <iostream>
#include <thread>
#include <array>
#include <atomic>
#include <zmq.hpp>
#include <franka/robot.h>
#include <franka/exception.h>

// === 配置区域 ===
const std::string ROBOT_IP = "172.16.0.2";              // Franka 机器人 IP
const std::string CMD_ADDRESS = "tcp://192.168.40.200:2098";  // 接收命令的地址
const std::string STATE_ADDRESS = "tcp://*:3099";       // 发送状态的地址
const std::string TORQUE_ADDRESS = "tcp://*:3087";      // 发送力矩的地址

std::array<double, 7> g_target_q;
std::atomic<bool> g_running{true};

// ZMQ 命令接收线程
void commandThread() {
    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.connect(CMD_ADDRESS);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    sub.setsockopt(ZMQ_CONFLATE, 1);
    
    while (g_running) {
        zmq::message_t msg;
        if (sub.recv(msg, zmq::recv_flags::dontwait)) {
            if (msg.size() == 56) {  // 7 * 8 bytes
                double* data = static_cast<double*>(msg.data());
                for (int i = 0; i < 7; i++) g_target_q[i] = data[i];
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ZMQ 状态发布线程
void stateThread(franka::Robot& robot) {
    zmq::context_t ctx(1);
    zmq::socket_t state_pub(ctx, ZMQ_PUB);
    zmq::socket_t torque_pub(ctx, ZMQ_PUB);
    state_pub.bind(STATE_ADDRESS);
    torque_pub.bind(TORQUE_ADDRESS);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    while (g_running) {
        auto state = robot.readOnce();
        
        // 发送状态 (位置 + 速度)
        std::array<float, 14> state_data;
        for (int i = 0; i < 7; i++) {
            state_data[i] = state.q[i];
            state_data[i+7] = state.dq[i];
        }
        state_pub.send(zmq::buffer(state_data), zmq::send_flags::dontwait);
        
        // 发送力矩
        std::array<float, 7> torque_data;
        for (int i = 0; i < 7; i++) {
            torque_data[i] = state.tau_ext_hat_filtered[i];
        }
        torque_pub.send(zmq::buffer(torque_data), zmq::send_flags::dontwait);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    try {
        // 连接机器人
        franka::Robot robot(ROBOT_IP);
        
        // 设置碰撞阈值
        robot.setCollisionBehavior(
            {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
            {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
            {{30, 30, 30, 25, 25, 25}}, {{30, 30, 30, 25, 25, 25}},
            {{30, 30, 30, 25, 25, 25}}, {{30, 30, 30, 25, 25, 25}}
        );
        
        // 读取初始位置
        auto initial = robot.readOnce();
        g_target_q = initial.q;
        
        std::cout << "初始位置: ";
        for (auto q : g_target_q) std::cout << q << " ";
        std::cout << "\n启动 ZMQ 线程..." << std::endl;
        
        // 启动线程
        std::thread cmd_thread(commandThread);
        std::thread state_thread(stateThread, std::ref(robot));
        
        // 控制循环
        std::array<double, 7> current_q = g_target_q;
        
        auto control = [&](const franka::RobotState& state, franka::Duration) {
            // 平滑插值
            for (int i = 0; i < 7; i++) {
                current_q[i] += 0.05 * (g_target_q[i] - current_q[i]);
            }
            return franka::JointPositions(current_q);
        };
        
        std::cout << "开始运动控制..." << std::endl;
        robot.control(control);
        
        g_running = false;
        cmd_thread.join();
        state_thread.join();
        
    } catch (const franka::Exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
```

**CMakeLists.txt**:

```cmake
cmake_minimum_required(VERSION 3.4)
project(franka_zmq_bridge)
set(CMAKE_CXX_STANDARD 14)

find_package(Franka REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)

add_executable(franka_zmq_bridge franka_zmq_bridge.cpp)
target_include_directories(franka_zmq_bridge PRIVATE 
    ${Franka_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS} ${CPPZMQ_INCLUDE_DIR})
target_link_libraries(franka_zmq_bridge 
    Franka::Franka ${ZMQ_LIBRARIES} pthread)
```

**编译**:
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### 步骤 3: 运行系统

#### 终端 1 - Franka 控制端:
```bash
sudo ./franka_zmq_bridge
```

#### 终端 2 - FACTR 工作站:
```bash
cd /home/cytoderm/projects/FACTR_Teleop
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch launch/factr_teleop.py
```

#### 终端 3 - 验证通信:
```bash
# 查看话题
ros2 topic list

# 监听 Franka 状态
ros2 topic echo /franka/right/obs_franka_state
```

## 网络连接检查清单

```bash
# 1. 在 FACTR 工作站上测试到 Franka 控制端
ping 10.0.10.1

# 2. 在 Franka 控制端测试到 FACTR 工作站
ping 192.168.40.200

# 3. 在 Franka 控制端测试到机器人
ping 172.16.0.2
```

## ZMQ 端口映射

| 用途 | 发送端 | 地址 | 接收端 |
|------|--------|------|--------|
| 位置命令 | FACTR | 192.168.40.200:2098 | Franka |
| 关节状态 | Franka | 10.0.10.1:3099 | FACTR |
| 关节力矩 | Franka | 10.0.10.1:3087 | FACTR |

## 故障排查

### 问题: "Connection to robot failed"
- 确认机器人已通过 Desk 解锁
- 检查机器人 IP: `ping 172.16.0.2`
- 确认没有其他程序占用 FCI

### 问题: "收不到 ZMQ 消息"
- 检查网络: `ping <对方IP>`
- 检查防火墙: `sudo ufw status`
- 验证端口正确

### 问题: "机器人不动"
- 检查 ROS2 是否发布命令: `ros2 topic echo /factr_teleop/right/cmd_franka_pos`
- 查看 Franka 程序输出是否收到命令
- 确认目标位置与当前位置不同

## 测试脚本

### Python 测试发送命令:
```python
#!/usr/bin/env python3
import zmq
import numpy as np
import time

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:2098")
time.sleep(1)

# 发送测试位置
test_pos = np.array([0, 0, 0, -1.57, 0, 1.57, 0.785])
while True:
    pub.send(test_pos.astype(np.float64).tobytes())
    print(f"发送: {test_pos}")
    time.sleep(0.1)
```

### Python 测试接收状态:
```python
#!/usr/bin/env python3
import zmq
import numpy as np

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://10.0.10.1:3099")
sub.setsockopt(zmq.SUBSCRIBE, b'')

print("等待状态...")
while True:
    msg = sub.recv()
    data = np.frombuffer(msg, dtype=np.float32)
    print(f"位置: {data[:7]}")
    print(f"速度: {data[7:14]}")
```

## 安全提示

⚠️ **重要**: 
- 首次运行时保持急停按钮触手可及
- 确保机器人周围无障碍物
- 从小幅度运动开始测试
- 监控机器人行为是否平滑

## 下一步

一切正常后，您可以:
- 调整控制参数 (`franka_example.yaml`)
- 添加力反馈功能
- 集成您的应用逻辑

更详细的信息请参考 [完整集成指南](FRANKA_FCI_INTEGRATION_GUIDE.md)。
