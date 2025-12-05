# 个人搭建框架部署与实验

## 环境配置
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

## 编译和构建
对节点进行更改后可能需要对该包进行重新编译
```bash
colcon build --packages-select factr_teleop
```

## 相关节点和程序
### 主要程序
**基于franky库的franka FCI和主机ZMQ的桥接程序** [franka_factr_bridge_franky.py](franka_factr_bridge_franky.py)
**夹爪的主动控制程序，接收factr端的控制信息与对franka发送夹爪控制（串口通信**[gripper_bridge_node.py](src/factr_teleop/factr_teleop/gripper_bridge_node.py)
**factr端的ZMQ服务，用于与桥接程序通信**[factr_teleop_franka_zmq.py ](src/factr_teleop/factr_teleop/factr_teleop_franka_zmq.py)
**同时启用夹爪和factr的launch**[teleop_system.launch.py](src/factr_teleop/launch/teleop_system.launch.py)


### 相关配置
**遥操程序IP地址配置**[global_configs.py ](src/python_utils/python_utils/global_configs.py)
**遥操程序factr补偿参数等配置**[franka_example.yaml](src/factr_teleop/factr_teleop/configs/franka_example.yaml)
**示例程序factr补偿参数等配置**[grav_comp_demo.yaml](src/factr_teleop/factr_teleop/configs/grav_comp_demo.yaml)



## 策略部署

ROS2 中提供了一个示例的部署脚本，用于将已有的策略部署到franka上。在我们的实现中，部署启动文件可以按以下方式调用： 
```bash
ros2 launch factr_teleop/launch/rollout.py
```