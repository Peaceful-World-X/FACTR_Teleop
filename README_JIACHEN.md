# 个人搭建框架部署与实验

## 相关配置
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
## 遥操示例

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

## 编译和构建
对节点进行更改后可能需要对该包进行重新编译
```bash
colcon build --packages-select factr_teleop
```
7分 40秒
## 相关节点和程序
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



## 策略部署

ROS2 中提供了一个示例的部署脚本，用于将已有的策略部署到franka上。在我们的实现中，部署启动文件可以按以下方式调用： 
```bash
ros2 launch factr_teleop/launch/rollout.py
```


## TODO
- **摄像头开发**
- **数采流程**
- 关节5关节飘的问题
- 一键启动的稳定性，有时候退出后会存在factr乱动的情况
- （已解决）程序关闭后主动关闭终端（或改成后台运行），结束后kill
- 夹爪速度过慢 无力反馈


## 问题

