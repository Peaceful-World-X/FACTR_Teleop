# Franka FCI 集成文档

本目录包含将 FACTR Teleop 系统与 Franka 机器人通过 FCI (Franka Control Interface) 模式连接的完整文档和工具。

## 📚 文档索引

### 1. [快速入门指南](FRANKA_FCI_QUICKSTART.md)
- **适用对象**: 希望快速建立基本通信的用户
- **内容**: 最简化的 3 步设置流程
- **特点**: 包含最小化示例代码和快速测试方法

### 2. [完整集成指南](FRANKA_FCI_INTEGRATION_GUIDE.md)
- **适用对象**: 需要深入了解系统架构和高级功能的用户
- **内容**: 
  - 详细的系统架构说明
  - 通信协议详解
  - 完整的 C++ 实现示例
  - 网络配置详解
  - 调试与故障排查
  - 高级功能（阻抗控制、笛卡尔空间控制等）
- **特点**: 深度技术文档，包含所有实现细节

## 🛠️ 工具

### 通信诊断工具
位置: `/home/cytoderm/projects/FACTR_Teleop/scripts/test_franka_comm.py`

**功能**:
- 测试网络连通性
- 测试 ZMQ 命令发布
- 测试 ZMQ 状态订阅
- 测试 ZMQ 力矩订阅

**使用方法**:
```bash
cd /home/cytoderm/projects/FACTR_Teleop

# 测试所有功能（右臂）
python3 scripts/test_franka_comm.py --side right

# 测试左臂
python3 scripts/test_franka_comm.py --side left

# 只测试特定功能
python3 scripts/test_franka_comm.py --test network    # 网络连通性
python3 scripts/test_franka_comm.py --test command    # 命令发布
python3 scripts/test_franka_comm.py --test state      # 状态接收
python3 scripts/test_franka_comm.py --test torque     # 力矩接收
```

## 🚀 快速开始

### 第一步：选择合适的指南

**如果您是第一次设置**:
- 先阅读 [快速入门指南](FRANKA_FCI_QUICKSTART.md)
- 完成基本设置并测试通信
- 确认系统能够正常工作

**如果您需要高级功能**:
- 阅读 [完整集成指南](FRANKA_FCI_INTEGRATION_GUIDE.md)
- 根据需求实现特定功能

### 第二步：配置网络

编辑 `src/python_utils/python_utils/global_configs.py`:

```python
# 您的 FACTR 工作站 IP
sim_desktop_ip_address = "192.168.40.200"

# Franka 控制端 IP（运行 FCI 程序的计算机）
franka_right_ip_address = "10.0.10.2"
franka_left_ip_address = "172.16.0.1"
```

### 第三步：测试连接

```bash
# 运行诊断工具
python3 scripts/test_franka_comm.py --side right
```

### 第四步：实现 Franka 控制端

按照快速入门指南中的示例代码在 Franka 控制端创建 C++ 程序。

### 第五步：启动系统

参考快速入门指南中的步骤启动完整系统。

## 📋 系统要求

### FACTR 工作站
- ✅ Ubuntu 20.04 / 22.04
- ✅ ROS 2 Foxy / Humble
- ✅ Python 3.8+
- ✅ PyZMQ (`pip install zmq`)

### Franka 控制端
- ✅ Ubuntu 20.04（推荐）
- ✅ libfranka 0.8.0+
- ✅ ZMQ C++ 库
- ✅ 实时内核（推荐但非必需）

### Franka 机器人
- ✅ Franka Emika Panda 或 FR3
- ✅ 系统版本 4.0+（支持 FCI）
- ✅ 机器人已解锁且处于用户模式

## 🔗 通信架构

```
┌──────────────────────┐                    ┌──────────────────────┐
│   FACTR 工作站       │                    │  Franka 控制端        │
│   (Python/ROS2)      │                    │  (C++/libfranka)     │
│                      │                    │                      │
│  ┌────────────────┐  │   ZMQ 命令流      │  ┌────────────────┐  │
│  │ factr_teleop_  │──┼──────────────────►│  │  ZMQ Subscriber│  │
│  │ franka_zmq     │  │  (位置命令)        │  │                │  │
│  │                │  │                    │  │  ┌──────────┐  │  │
│  │                │◄─┼────────────────────┼──┤  │ FCI      │  │  │
│  │                │  │  (状态/力矩)       │  │  │ Control  │  │  │
│  └────────────────┘  │   ZMQ 状态流      │  │  └────┬─────┘  │  │
│                      │                    │  │       │        │  │
└──────────────────────┘                    └──┴───────┼────────┴──┘
                                                       │ FCI
                                            ┌──────────▼──────────┐
                                            │  Franka Robot       │
                                            └─────────────────────┘
```

## 🔍 故障排查速查表

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 无法 ping 通 | 网络未配置 | 检查 IP 地址、子网掩码、网关 |
| ZMQ 连接超时 | 防火墙阻止 | 开放相应端口 |
| 机器人不响应 | 机器人未解锁 | 通过 Desk 解锁机器人 |
| 收不到状态 | 程序未运行 | 确认 Franka 控制端程序在运行 |
| 运动不平滑 | 插值参数不当 | 调整插值系数 |

## 📖 参考资源

### 官方文档
- [libfranka 文档](https://frankaemika.github.io/docs/)
- [Franka Control Interface (FCI) 指南](https://frankaemika.github.io/docs/libfranka.html)
- [ZMQ 指南](https://zguide.zeromq.org/)
- [ROS 2 文档](https://docs.ros.org/)

### FACTR 项目
- [FACTR 项目主页](https://jasonjzliu.com/factr/)
- [FACTR 论文](https://arxiv.org/abs/2502.17432)
- [FACTR 主仓库](https://github.com/RaindragonD/factr/)
- [FACTR 硬件](https://github.com/JasonJZLiu/FACTR_Hardware)

## ⚠️ 安全注意事项

1. **首次运行**: 保持急停按钮触手可及
2. **工作空间**: 确保机器人周围无障碍物和人员
3. **测试策略**: 从小幅度运动开始，逐步增加范围
4. **碰撞检测**: 使用合适的碰撞阈值
5. **监控**: 始终监控机器人行为是否正常

## 🤝 贡献

如果您发现文档中的错误或有改进建议，欢迎提交 Issue 或 Pull Request。

## 📝 许可证

本文档随 FACTR Teleop 项目一起发布，遵循 Apache 2.0 许可证。

---

**最后更新**: 2025年11月25日
