# Franka FCI 网络配置模板

## 配置说明

此文件提供了不同网络拓扑下的配置示例，帮助您快速配置系统。

## 场景 1: 单机器人配置（最常见）

```
网络拓扑:
┌─────────────────┐         ┌─────────────────┐         ┌──────────────┐
│  FACTR 工作站   │◄───────►│ Franka 控制端   │◄───────►│ Franka Robot │
│  192.168.1.100  │  以太网 │  192.168.1.101  │  专用网 │  172.16.0.2  │
└─────────────────┘         └─────────────────┘         └──────────────┘
```

### FACTR 工作站配置
编辑 `src/python_utils/python_utils/global_configs.py`:

```python
sim_desktop_ip_address = "192.168.1.100"
franka_right_ip_address = "192.168.1.101"
franka_left_ip_address = "192.168.1.101"  # 单臂时可忽略
```

### Franka 控制端配置
修改 `franka_zmq_bridge.cpp`:

```cpp
const std::string ROBOT_IP = "172.16.0.2";              // Franka 机器人 IP
const std::string CMD_ADDRESS = "tcp://192.168.1.100:2098";  // FACTR 工作站 IP
const std::string STATE_ADDRESS = "tcp://*:3099";
const std::string TORQUE_ADDRESS = "tcp://*:3087";
```

### 网络接口配置（Franka 控制端）

```bash
# 假设连接 FACTR 的网卡是 enp0s31f6
sudo ip addr add 192.168.1.101/24 dev enp0s31f6

# 假设连接 Franka 的网卡是 enp3s0
sudo ip addr add 172.16.0.1/16 dev enp3s0
```

---

## 场景 2: 双臂配置

```
网络拓扑:
                        ┌─────────────────┐
                        │  FACTR 工作站   │
                        │  192.168.1.100  │
                        └────────┬────────┘
                                 │
                  ┌──────────────┴──────────────┐
                  │                             │
         ┌────────▼────────┐           ┌───────▼────────┐
         │ Franka 控制端 1 │           │ Franka 控制端 2│
         │  192.168.1.101  │           │ 192.168.1.102  │
         └────────┬────────┘           └───────┬────────┘
                  │                             │
         ┌────────▼────────┐           ┌───────▼────────┐
         │ Franka Robot L  │           │ Franka Robot R │
         │   172.16.0.2    │           │   10.0.10.2    │
         └─────────────────┘           └────────────────┘
```

### FACTR 工作站配置

```python
sim_desktop_ip_address = "192.168.1.100"
franka_left_ip_address = "192.168.1.101"   # 左臂控制端
franka_right_ip_address = "192.168.1.102"  # 右臂控制端

franka_left_real_zmq_addresses = {
    "joint_state_sub":  "tcp://192.168.1.101:5099",
    "joint_torque_sub": "tcp://192.168.1.101:5087",
    "joint_pos_cmd_pub": "tcp://192.168.1.100:4098",
}

franka_right_real_zmq_addresses = {
    "joint_state_sub":  "tcp://192.168.1.102:3099",
    "joint_torque_sub": "tcp://192.168.1.102:3087",
    "joint_pos_cmd_pub": "tcp://192.168.1.100:2098",
}
```

### 左臂控制端配置

```cpp
const std::string ROBOT_IP = "172.16.0.2";
const std::string CMD_ADDRESS = "tcp://192.168.1.100:4098";  // 注意端口 4098
const std::string STATE_ADDRESS = "tcp://*:5099";            // 注意端口 5099
const std::string TORQUE_ADDRESS = "tcp://*:5087";           // 注意端口 5087
```

### 右臂控制端配置

```cpp
const std::string ROBOT_IP = "10.0.10.2";
const std::string CMD_ADDRESS = "tcp://192.168.1.100:2098";  // 注意端口 2098
const std::string STATE_ADDRESS = "tcp://*:3099";            // 注意端口 3099
const std::string TORQUE_ADDRESS = "tcp://*:3087";           // 注意端口 3087
```

---

## 场景 3: 一体化配置（FACTR 与 FCI 在同一台机器）

```
网络拓扑:
┌──────────────────────────────┐         ┌──────────────┐
│  统一工作站                   │         │              │
│  (FACTR + Franka 控制端)     │◄───────►│ Franka Robot │
│  IP1: 192.168.1.100 (对外)   │  专用网 │  172.16.0.2  │
│  IP2: 172.16.0.1 (Franka)    │         │              │
└──────────────────────────────┘         └──────────────┘
```

### 配置（使用 localhost）

```python
# global_configs.py
sim_desktop_ip_address = "127.0.0.1"      # 本地通信
franka_right_ip_address = "127.0.0.1"
```

```cpp
// franka_zmq_bridge.cpp
const std::string ROBOT_IP = "172.16.0.2";
const std::string CMD_ADDRESS = "tcp://127.0.0.1:2098";
const std::string STATE_ADDRESS = "tcp://*:3099";
const std::string TORQUE_ADDRESS = "tcp://*:3087";
```

**优点**: 
- 无需网络配置
- 通信延迟最低
- 成本最低

**缺点**:
- 需要较高的计算性能
- 难以实现实时性保证

---

## ZMQ 端口分配

| 机器人 | 用途 | 方向 | 地址示例 |
|--------|------|------|---------|
| 右臂 | 位置命令 | FACTR → Franka | tcp://FACTR_IP:2098 |
| 右臂 | 关节状态 | Franka → FACTR | tcp://Franka_IP:3099 |
| 右臂 | 关节力矩 | Franka → FACTR | tcp://Franka_IP:3087 |
| 左臂 | 位置命令 | FACTR → Franka | tcp://FACTR_IP:4098 |
| 左臂 | 关节状态 | Franka → FACTR | tcp://Franka_IP:5099 |
| 左臂 | 关节力矩 | Franka → FACTR | tcp://Franka_IP:5087 |

---

## 防火墙配置

### 在 FACTR 工作站

```bash
# 允许接收 Franka 状态（如果有防火墙）
sudo ufw allow from <Franka_Control_IP> to any port 2098
sudo ufw allow from <Franka_Control_IP> to any port 4098
```

### 在 Franka 控制端

```bash
# 允许 FACTR 工作站连接
sudo ufw allow from <FACTR_IP> to any port 3099
sudo ufw allow from <FACTR_IP> to any port 3087
sudo ufw allow from <FACTR_IP> to any port 5099
sudo ufw allow from <FACTR_IP> to any port 5087

# 或者简单禁用防火墙（仅在安全网络中）
sudo ufw disable
```

---

## 网络测试命令

### 测试连通性

```bash
# 在 FACTR 工作站测试到 Franka 控制端
ping <Franka_Control_IP>

# 在 Franka 控制端测试到 FACTR 工作站
ping <FACTR_IP>

# 在 Franka 控制端测试到机器人
ping <Robot_IP>
```

### 测试端口

```bash
# 测试端口是否开放（需要安装 netcat）
nc -zv <IP_Address> <Port>

# 例如
nc -zv 192.168.1.101 3099
```

### 监听网络流量

```bash
# 监听特定端口（需要 tcpdump）
sudo tcpdump -i <interface> port <port_number>

# 例如监听 ZMQ 状态发布
sudo tcpdump -i enp0s31f6 port 3099
```

---

## 配置验证清单

使用此清单确认配置正确：

- [ ] IP 地址已在 `global_configs.py` 中正确设置
- [ ] IP 地址已在 C++ 程序中正确设置
- [ ] 网络接口已配置正确的 IP
- [ ] 可以 ping 通所有设备
- [ ] 防火墙已正确配置或禁用
- [ ] ZMQ 端口没有被其他程序占用
- [ ] Franka 机器人已解锁
- [ ] 运行诊断工具测试通过

---

## 常见问题

### 问题：IP 地址配置后无法通信

**检查步骤**:
1. 确认 IP 地址在同一子网
2. 检查子网掩码（通常是 /24 或 255.255.255.0）
3. 使用 `ip addr` 确认网卡配置生效
4. 尝试重启网络服务：`sudo systemctl restart networking`

### 问题：配置正确但通信不稳定

**可能原因**:
1. 网络带宽不足
2. 网络延迟过高
3. 使用了 WiFi（应使用有线连接）

**解决方案**:
- 使用千兆以太网
- 使用交换机而非路由器
- 避免网络拥堵

### 问题：端口冲突

**检查端口占用**:
```bash
sudo lsof -i :<port_number>
```

**更换端口**: 如果端口被占用，修改配置文件使用其他端口。

---

## 推荐网络配置

### 专用网络（最佳）

```
FACTR 工作站 ←→ 独立交换机 ←→ Franka 控制端 ←→ Franka 机器人
```

**优点**:
- 无干扰
- 低延迟
- 高可靠性

### 使用 VLAN（备选）

如果必须在现有网络中部署，使用 VLAN 隔离机器人通信。

---

## 自动化配置脚本

### FACTR 工作站 (setup_network.sh)

```bash
#!/bin/bash
# 设置网络接口
INTERFACE="enp0s31f6"
IP_ADDRESS="192.168.1.100/24"

sudo ip addr add $IP_ADDRESS dev $INTERFACE
sudo ip link set $INTERFACE up

echo "网络接口 $INTERFACE 已配置为 $IP_ADDRESS"
```

### Franka 控制端 (setup_network.sh)

```bash
#!/bin/bash
# 设置网络接口
FACTR_INTERFACE="enp0s31f6"
FACTR_IP="192.168.1.101/24"

ROBOT_INTERFACE="enp3s0"
ROBOT_IP="172.16.0.1/16"

sudo ip addr add $FACTR_IP dev $FACTR_INTERFACE
sudo ip link set $FACTR_INTERFACE up

sudo ip addr add $ROBOT_IP dev $ROBOT_INTERFACE
sudo ip link set $ROBOT_INTERFACE up

echo "网络配置完成"
echo "  连接 FACTR: $FACTR_INTERFACE - $FACTR_IP"
echo "  连接 Robot: $ROBOT_INTERFACE - $ROBOT_IP"
```

使用方法:
```bash
chmod +x setup_network.sh
sudo ./setup_network.sh
```

---

**注意**: 网络配置会在重启后丢失。要使配置永久生效，请编辑 `/etc/netplan/` 中的配置文件（Ubuntu 18.04+）。
