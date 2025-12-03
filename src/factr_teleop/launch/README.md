# Launch 文件说明

本目录包含了 FACTR Teleop 系统的所有 ROS 2 launch 文件，用于启动不同的功能模块。

## 文件列表

### 1. `factr_teleop.py`
**完整的遥操作系统启动文件**。启动 leader arm 与 Franka follower arm 之间的完整遥操作系统，包括双向通信、力矩反馈和夹爪反馈功能。使用 `franka_example.yaml` 配置文件，需要同时连接 leader arm 和 Franka follower arm，适用于完整的遥操作任务。

### 2. `factr_teleop_grav_comp_demo.py`
**重力补偿演示启动文件**。启动仅包含 leader arm 的重力补偿演示程序，展示重力补偿和零空间调节功能，不需要连接 Franka follower arm。使用 `grav_comp_demo.yaml` 配置文件，适合测试 leader arm 的基本功能或演示重力补偿效果。

### 3. `collect_data.py`
**数据收集启动文件**。启动完整的数据收集系统，用于收集专家演示数据用于后续的策略训练。包含三个节点：`factr_teleop_franka_right`（右侧 Franka 遥操作节点）、`data_record`（数据记录节点，记录状态话题和图像话题）和 `realsense`（RealSense 相机节点），适用于收集模仿学习所需的训练数据。

### 4. `rollout.py`
**策略部署/回放启动文件**。启动策略回放系统，用于运行训练好的策略并保存回放数据。包含三个节点：`franka_bridge`（Franka 机械臂桥接节点，启用力矩反馈）、`policy_rollout`（策略回放节点，执行训练好的策略并保存数据）和 `realsense`（RealSense 相机节点），适用于部署和测试训练好的策略，进行策略评估和验证。

### 5. `check_joint_positions.py`
**关节角度检查工具脚本**。用于实时读取和显示 leader arm 的当前关节角度，包括角度（度）、弧度、目标角度和误差角度等信息。支持实时刷新显示，帮助用户将 leader arm 调整到校准位置，默认使用 USB 设备 `usb-FTDI_USB__-__Serial_Converter_FTA5H0VZ-if00-port0`，可通过命令行参数指定其他设备。

## 使用方法

### 运行遥操作系统
```bash
cd /home/cytoderm/projects/FACTR_Teleop
source .venv/bin/activate  # 激活虚拟环境（如果需要）
source install/local_setup.bash  # 加载 ROS 

# 完整遥操作（需要 Franka）
ros2 launch launch/factr_teleop.py

# 重力补偿演示（只需要 leader arm）
ros2 launch launch/factr_teleop_grav_comp_demo.py
```

### 数据收集
```bash
ros2 launch launch/collect_data.py
```

### 策略回放
```bash
ros2 launch launch/rollout.py
```

### 检查关节角度
```bash
# 使用默认 USB 设备
python3 launch/check_joint_positions.py

```

