# 个人搭建框架部署与实验

## 一、相关配置
### 网络配置
连接网线后以太网连接IPv4的地址需要设置为
```bash
10.0.10.1
```
连接https://10.0.10.2/desk/的图形化界面后对界面进行 *activate FCI*的操作
### 环境配置
**factr相关ros节点**,用于factr端的相关ros节点的启动
```bash
cd /home/cytoderm/projects/FACTR_Teleop
source .venv/bin/activate  # 激活虚拟环境（如果需要）
source install/local_setup.bash  # 加载 ROS 
source install/setup.bash
```
**基于Franky的franka_FCI_brige程序**，用于[franka_factr_bridge_franky.py](franka_factr_bridge_franky.py)的启动配置
```bash
conda activate franka
```
## 二、遥操数采

**先按照前面的环境配置进入环境**
- **分别打开**

打开两个终端，分别运行配置以上两个环境

```bash
#.venv环境下运行
ros2 launch factr_teleop teleop_system.launch.py

#conda franka环境下运行
python franka_factr_bridge_franky.py
```
- **一键启动式遥操**
```bash
chmod +x run.sh
./run.sh
```

### 程序启动
- 保持各关节尽可能保持初始点，程序启动前保持factr夹爪关闭；夹爪启动经过校准为零点。

### 程序关闭
- 对两个程序进行启动后需要关闭遥操程序，最好先ctrl+C掉factr_teleop teleop_system.launch.py程序，再ctrl+C掉franka_factr_bridge_franky.py程序，避免出现可能的错误

- 若factr leader arm出现剧烈不规则运动，即使拔除电源即可

## 三、回放示例

**回放程序为[action_playback.py](src/factr_teleop/factr_teleop/action_playback.py),提供了从采集的数据文件中回放Franka机械臂动作的功能。**

数据文件应为 pickle 格式，包含帧数据序列。每一帧应包含：

- **关节位置数据**：`/factr_teleop/right/cmd_franka_pos` topic，包含 `position` 字段（7个关节角度）
- **夹爪命令**：`leader_gripper` 字段（夹爪开度，0.0-1.0之间，0=完全关闭，1=完全张开）
- **时间控制**：基于固定 30Hz 帧率

### 使用方法

#### 基本用法

```bash
cd /home/cytoderm/projects/FACTR_Teleop
python src/factr_teleop/factr_teleop/action_playback.py --data_path /path/to/data.pkl
```

#### 参数说明

- `--data_path`, `-d`: **必需** data.pkl 文件路径
- `--speed`, `-s`: 回放速度倍数 (默认: 1.0)
  - `0.5`: 慢放一半速度
  - `2.0`: 快放两倍速度
- `--zmq_address`, `-z`: ZMQ 命令发布地址 (默认: `tcp://127.0.0.1:2098`)
- `--gripper_topic`, `-g`: ROS 夹爪命令话题 (默认: `/factr_teleop/franka/cmd_gripper_pos`)
- `--verbose`, `-v`: 启用详细输出
- `--interactive`, `-i`: 启用交互模式（支持键盘控制）

启用 `--interactive` 模式后，可以使用键盘控制回放：

- **p**: 暂停/恢复回放
- **q**: 退出回放

### 必需组件

1. **Franka 桥接程序运行中**:
   ```bash
   python franka_factr_bridge_franky.py
   ```

2. **夹爪桥接节点运行中** (如果需要控制夹爪):
   ```bash
   ros2 run factr_teleop gripper_bridge_node
   ```

## 四、编译和构建
对节点进行更改后可能需要对该包进行重新编译
```bash
colcon build --packages-select factr_teleop
```

## 五、相关节点和程序
### 主要程序
- **基于franky库的franka FCI和主机ZMQ的桥接程序** [franka_factr_bridge_franky.py](franka_factr_bridge_franky.py)
    1. ZMQpub：发布状态：14 个 float（7 个位置 + 7 个速度）|发布力矩：7 个 float | 发布位姿16 个 float64
    位姿信息包含：旋转和平移，左上 3x3 为末端相对基坐标系的旋转矩阵，右上 3x1（第 3 列的前 3 个元素）为平移向量（x, y, z，以米为单位）最后一行通常是 [0, 0, 0, 1]（齐次坐标）
    2. sub：订阅主臂7 个 double（8 字节/项）位置信息
- **夹爪的主动控制程序，接收factr端的控制信息与对franka发送夹爪控制（串口通信）**[gripper_bridge_node.py](src/factr_teleop/factr_teleop/gripper_bridge_node.py)
    1. pub：发送主臂和从臂的夹爪状态0～1（0表示完全闭合，1为完全打开）
    2. sub：订阅主臂夹爪的位置
- **factr端的ZMQ服务，用于与桥接程序通信**[factr_teleop_franka_zmq.py ](src/factr_teleop/factr_teleop/factr_teleop_franka_zmq.py)

- **ros2同时启用夹爪和factr的launch**[teleop_system.launch.py](src/factr_teleop/launch/teleop_system.launch.py)

- **遥操收集数据的launch**[collect_data.launch.py](src/factr_teleop/launch/collect_data.launch.py)
    1. factr_teleop_franka_right：右臂遥操节点
    2. gripper_node：夹爪处理节点
    3. data_record_node:数据记录节点，用于记录摄像头和机械臂的数据
    4. realsense_front_node（realsense_back_node）：摄像头节点
    5. zmq_ros_bridge_node：ZMQ服务数据转换为ROS2节点数据并发布的节点

### 相关配置
- **遥操程序IP地址配置**[global_configs.py ](src/python_utils/python_utils/global_configs.py)

- **遥操程序factr补偿参数等配置**[franka_example.yaml](src/factr_teleop/factr_teleop/configs/franka_example.yaml)

- **示例程序factr补偿参数等配置**[grav_comp_demo.yaml](src/factr_teleop/factr_teleop/configs/grav_comp_demo.yaml)



## 六、策略部署

ROS2 中提供了一个示例的部署脚本，用于将已有的策略部署到franka上。在我们的实现中，部署启动文件可以按以下方式调用： 
```bash
ros2 launch factr_teleop/launch/rollout.py
```


## 七、TODO
- 一键启动的稳定性，有时候退出后会存在factr乱动的情况
- 回放，franka控制接口


## 八、问题

Topic 名称: /franka/joint_states
消息类型: sensor_msgs.msg.JointState
发布的信息:

包含 Franka 机械臂的关节状态数据。
position: 7 维关节位置 (q, 从 ZMQ 的 STATE_SUB_ADDRESS 接收，float32 数组的前 7 个元素)。
velocity: 7 维关节速度 (dq, 从 ZMQ 的 STATE_SUB_ADDRESS 接收，float32 数组的后 7 个元素)。
effort: 7 维关节力矩 (tau, 从 ZMQ 的 TORQUE_SUB_ADDRESS 接收，7 个 float32；如果未收到，则补零)。
name: 关节名称列表 ["panda_joint1", "panda_joint2", ..., "panda_joint7"]。
header.stamp: 当前 ROS 时间戳。
发布频率：取决于 ZMQ 数据到达频率（约 100Hz），当 current_q 和 current_dq 都可用时发布。
Topic 名称: /franka/end_effector_pose
消息类型: std_msgs.msg.Float64MultiArray
发布的信息:

包含 Franka 末端执行器 (end-effector) 的位姿数据。
data: 16 个 float64 元素，表示 4x4 齐次变换矩阵的展平（col-major 顺序，从 ZMQ 的 POSE_SUB_ADDRESS 接收）。
发布频率：每当收到 ZMQ 位姿数据时发布（约 100Hz）。
Topic 名称: /franka/right/obs_franka_torque
消息类型: sensor_msgs.msg.JointState
发布的信息:

专门发布 Franka 的关节力矩数据（用于 ROS teleop 节点）。
effort: 7 维关节力矩 (tau, 从 ZMQ 的 TORQUE_SUB_ADDRESS 接收，7 个 float32)。
header.stamp: 与 /franka/joint_states 相同的 ROS 时间戳。
其他字段（如 position/velocity）为空。
发布频率：与 /franka/joint_states 同步，当力矩数据可用时发布。


Topic 名称: /bridge/obs_gripper_state
消息类型: sensor_msgs.msg.JointState
发布的信息:
包含 Leader 和 Follower 夹爪的当前状态数据。
name: 关节名称列表 ['leader_gripper_ratio', 'follower_gripper_ratio']。
position: 两个浮点数列表 [leader_ratio, follower_ratio]，表示归一化开度（0.0 = 完全闭合，1.0 = 完全张开）。
leader_ratio: Leader 夹爪的归一化开度（基于内部状态 self.leader_ratio，从命令回调中更新）。
follower_ratio: Follower 夹爪的归一化开度（基于物理夹爪读取的位置，线性映射到 0.0-1.0 范围）。
header.stamp: 当前 ROS 时间戳（使用 self.get_clock().now().to_msg()）。
发布频率：由 poll_rate 参数控制（默认 50Hz），在 control_loop 中定期发布。

以上是从bridge中发布的话题情况，我需要把它们全部接收，包括franka的关节位置、关节速度、关节力矩、末端位姿；这些信息是需要从zmq_ros_bridge中接收的