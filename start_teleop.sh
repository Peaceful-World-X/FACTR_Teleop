#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 获取脚本所在目录的上一级目录作为项目根目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# --- 配置区域 ---
# 指定用于运行 franky 脚本的 Python 解释器路径
# 如果你在 conda 环境中，可以设置为: $HOME/miniconda3/envs/your_env/bin/python
# 或者默认尝试寻找 python3.11
PYTHON_FRANKY="/home/cytoderm/anaconda3/envs/franka/bin/python3.11"

# 检查是否安装了 python3.11
if ! command -v $PYTHON_FRANKY &> /dev/null; then
    echo -e "${RED}Error: $PYTHON_FRANKY not found.${NC}"
    echo "Please edit this script to set the correct PYTHON_FRANKY path."
    exit 1
fi

# ----------------

echo -e "${BLUE}=== Starting FACTR Teleop System ===${NC}"

# 定义清理函数，当脚本接收到 Ctrl+C 时执行
cleanup() {
    echo -e "\n${RED}Shutting down all processes...${NC}"
    kill 0 # 杀死当前进程组的所有进程
}

# 捕获 SIGINT (Ctrl+C)
trap cleanup SIGINT EXIT

# 1. 启动 Franka ZMQ Bridge (Python 3.11)
echo -e "${GREEN}[1/2] Starting Franka-FACTR Bridge ($PYTHON_FRANKY)...${NC}"
# 假设脚本在项目根目录下
BRIDGE_SCRIPT="$PROJECT_ROOT/franka_factr_bridge_franky.py"

if [ ! -f "$BRIDGE_SCRIPT" ]; then
    echo -e "${RED}Error: Bridge script not found at $BRIDGE_SCRIPT${NC}"
    exit 1
fi

$PYTHON_FRANKY "$BRIDGE_SCRIPT" &
BRIDGE_PID=$!

# 等待几秒钟让 ZMQ 服务建立
sleep 2

# 2. 启动 ROS 2 Nodes (Launch file)
echo -e "${GREEN}[2/2] Starting ROS 2 Teleop Nodes...${NC}"
# 假设当前环境已经 source 了 ROS 2 和 workspace 的 setup.bash
# 如果没有，可以在这里 source
# source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

ros2 launch factr_teleop teleop_system.launch.py &
ROS_PID=$!

# 等待子进程结束
wait $BRIDGE_PID $ROS_PID

