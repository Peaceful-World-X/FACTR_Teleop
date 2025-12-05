
<h1> FACTR Teleop: 低成本力反馈遥操作系统</h1>



#### [Jason Jingzhou Liu](https://jasonjzliu.com)<sup>\*</sup>, [Yulong Li](https://yulongli42.github.io)<sup>\*</sup>, [Kenneth Shaw](https://kennyshaw.net), [Tony Tao](https://tony-tao.com), [Ruslan Salakhutdinov](https://www.cs.cmu.edu/~rsalakhu/), [Deepak Pathak](https://www.cs.cmu.edu/~dpathak/)
_卡内基梅隆大学_

[项目主页](https://jasonjzliu.com/factr/) | [arXiV](https://arxiv.org/abs/2502.17432) | [FACTR](https://github.com/RaindragonD/factr/) | [FACTR 硬件](https://github.com/JasonJZLiu/FACTR_Hardware)


<br>

## 目录
- [安装](#安装)
- [FACTR 遥操作](#factr-遥操作)
- [数据采集](#数据采集)
- [训练与部署](#训练与部署)
- [许可与致谢](#许可与致谢)
- [引用](#引用)


## 安装

本仓库需要 **ROS 2**。
如果您尚未安装 ROS 2，请按照官方的 [ROS 2 安装指南](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) 进行安装。

### 提供的 ROS 2 包

本仓库包含以下 ROS 2 包：

- **`factr_teleop`**: 核心遥操作包，实现了 FACTR 低成本力反馈遥操作系统。提供主控臂控制功能，包括重力补偿、零空间调节、摩擦补偿和力反馈功能。

- **`bc`**: 行为克隆包，用于数据采集和策略部署。包含用于记录遥操作数据、同步数据流、回放轨迹和部署训练好的策略的节点。

- **`cameras`**: 相机接口包，提供用于运行遥操作和数据采集过程中使用的相机（ZED 和 RealSense）的 ROS 2 节点。

- **`python_utils`**: Python 工具函数包，包含通用工具，如 ZMQ 消息传递、全局配置和在其他包中使用的辅助函数。

这些包位于：

```
<repo_root>/src
```

### ROS 2 工作空间设置

这些包必须位于 **ROS 2 工作空间** 内。如果您还没有工作空间，请按照 [ROS 2 工作空间教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) 创建一个。

然后：

1. 将提供的四个包复制到您工作空间的 `src/` 目录中。
2. 确保在终端中 source ROS2 设置脚本
   ```bash
   source /opt/ros/<ROS-Distribution>/setup.bash
   ```
   注意：每次打开新终端时都需要运行此命令。
3. 从工作空间根目录，通过以下命令构建工作空间：
   ```bash
   colcon build --symlink-install
   ```
   这应该会在您的工作空间根目录中创建以下文件夹
   ```bash
   build  install  log  src
   ```
4. 从工作空间根目录，通过以下命令 source 覆盖层：
   ```bash
   source install/local_setup.bash
   ```
   注意：每次打开新终端时也需要运行此命令。

> 更多指导，请参考 [ROS 2 教程](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)。

### 额外的 Python 依赖

安装 [ZMQ](https://zeromq.org/)：

```bash
pip install zmq
```
安装 [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)：
```bash
sudo apt install ros-<ROS-Distribution>-pinocchio
```
- 例如，
   ```bash
   sudo apt install ros-humble-pinocchio
   ```
或者，可以通过 pip 尝试以下方式：
```bash
python -m pip install pin
```

最后，导航到 Dynamixel 子模块并通过以下方式安装：
```bash
cd <repo_root>/src/factr_teleop/factr_teleop/dynamixel
pip install -e python
```


## FACTR 遥操作
有关设置 FACTR 主控臂和运行提供的示例演示的说明，请参见
[此处](src/factr_teleop/README.md)。



## 数据采集
我们提供 ROS2 中的说明和示例数据采集脚本。您可能需要自定义的机器人和传感器节点来运行系统。在我们的实现中，采集的数据按以下格式保存：
### 数据结构
每个轨迹保存为单独的 pickle 文件。每个 pickle 文件包含一个具有以下结构的字典：
```
trajectory.pkl
├── "data" : dict
│   ├── "topic_name_1" : list[data_points]
│   ├── "topic_name_2" : list[data_points]
│   └── ...
└── "timestamps" : dict
    ├── "topic_name_1" : list[timestamps]
    ├── "topic_name_2" : list[timestamps]
    └── ...
```
### 关键组件：

- **data**: 一个字典，其中：
  - 键是数据源名称（在我们的实现中是 ROS 话题名称）
  - 值是包含实际数据点的列表（低维状态或图像）

- **timestamps**: 一个字典，其中：
  - 键与 "data" 字典中的数据源名称相同
  - 值是包含每个对应数据点记录时间戳的列表

*注意*：不同的数据源可能以不同的频率记录，导致不同数据源的列表长度不同。时间戳对于正确对齐和后处理数据至关重要。
虽然 ROS 提供了同步 API，但我们选择记录原始时间戳并执行后处理，以便在数据分析和对齐方面具有更大的灵活性。
```python
# 轨迹结构示例
{
    "data": {
        "/camera/rgb/image_raw": [image1, image2, ...],
        "/joint_states": [state1, state2, ...],
        "/robot/end_effector_pose": [pose1, pose2, ...]
    },
    "timestamps": {
        "/camera/rgb/image_raw": [1615420323.45, 1615420323.55, ...],
        "/joint_states": [1615420323.40, 1615420323.50, ...],
        "/robot/end_effector_pose": [1615420323.42, 1615420323.52, ...]
    }
}
```



## 训练与部署

### 数据处理与训练
请查看我们的 [factr](https://github.com/RaindragonD/factr) 仓库以获取详细说明。

### 策略部署

我们在 ROS2 中提供了一个示例部署脚本。在我们的实现中，部署启动文件可以按以下方式调用： 
```bash
ros2 launch factr_teleop/launch/rollout.py
```
请查看 [rollout.py](launch/rollout.py) 了解配置详情。


## 许可与致谢
本源代码根据本仓库根目录中 LICENSE 文件中的 Apache 2.0 许可证进行许可。

本项目基于或使用了以下第三方依赖。
- [GELLO](https://wuphilipp.github.io/gello_site/): 本工作的灵感来源。
- [ZMQ](https://zeromq.org/): Python 进程之间的轻量级通信。
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/): 用于操作任务的快速运动学和动力学计算。


## 引用
如果您发现此代码库有用，欢迎引用我们的工作！
<div style="display:flex;">
<div>

```bibtex
@article{factr,
  title={FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning},
  author={Liu, Jason Jingzhou and Li, Yulong and Shaw, Kenneth and Tao, Tony and Salakhutdinov, Ruslan and Pathak, Deepak},
  journal={arXiv preprint arXiv:2502.17432},
  year={2025}
}
```
