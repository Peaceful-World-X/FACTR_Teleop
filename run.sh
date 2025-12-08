#!/bin/bash

# 脚本状态信息
echo "🚀 启动 FACTR 遥操作系统..."
echo "   - Franky Bridge: Franka 机器人通信"
echo "   - ROS2 Launch: 遥操作和相机节点"
echo ""

# 全局变量存储进程ID
FRANKY_PID=""
ROS_PID=""


cleanup() {
    echo ""
    echo "🛑 收到中断信号，正在停止程序..."
    
    if [ "$STOP_STAGE" -eq 0 ]; then
        # 第一次 Ctrl+C：先停 ROS
        if [ -n "$ROS_PID" ] && kill -0 "$ROS_PID" 2>/dev/null; then
            echo "   停止 ROS2 Launch (PID: $ROS_PID)..."
            kill -SIGINT "$ROS_PID" 2>/dev/null || kill -9 "$ROS_PID" 2>/dev/null
        fi
        STOP_STAGE=1
        echo "  ROS 已请求停止，如需退出 Bridge，请再按一次 Ctrl+C"
        return
    fi

    # 第二次 Ctrl+C：再停 Franky
    if [ -n "$FRANKY_PID" ] && kill -0 "$FRANKY_PID" 2>/dev/null; then
        echo "   停止 Franky Bridge (PID: $FRANKY_PID)..."
        kill -SIGINT "$FRANKY_PID" 2>/dev/null || kill -9 "$FRANKY_PID" 2>/dev/null
    fi

    wait 2>/dev/null
    echo " 所有程序已停止"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# Franky Bridge (后台运行)
echo "🔧 启动 Franky Bridge..."
(
    echo "[Franky] 激活 Franka 环境..."
    # cd /home/cytoderm/projects/FACTR_Teleop
    source ~/anaconda3/etc/profile.d/conda.sh
    conda activate franka
    
    echo "[Franky] 启动 Franka-FACTR Bridge..."
    /home/cytoderm/anaconda3/envs/franka/bin/python3.11 franka_factr_bridge_franky.py
) &
FRANKY_PID=$!
echo "   Franky Bridge 已启动 (PID: $FRANKY_PID)"

# 等待 2 秒让 Franky 初始化
sleep 2

# ROS2 Launch (后台运行)
echo "🤖 启动 ROS2 系统..."
    enable: True  # 是否启用力矩反馈（True=启用，False=禁用）
    gain: 1.5  # 力矩反馈增益系数
    damping: 0.0  # 力矩反馈阻尼系数
(
    echo "[ROS2] 配置环境..."
    # cd /home/cytoderm/projects/FACTR_Teleop
    source .venv/bin/activate
    source install/local_setup.bash
    source install/setup.bash
    
    echo "[ROS2] 启动遥操作和相机节点..."
    #ros2 launch factr_teleop teleop_system.launch.py  #启动遥操作节点
    ros2 launch factr_teleop collect_data.launch.py  #启动数据采集节点
) &
ROS_PID=$!
echo "   ROS2 系统已启动 (PID: $ROS_PID)"

echo ""
echo "✅ 系统启动完成！"
echo "   Franky Bridge (PID: $FRANKY_PID)"
echo "   ROS2 Launch (PID: $ROS_PID)"
echo ""
echo "💡 按 Ctrl+C 停止所有程序"
echo "📊 查看运行状态: ps aux | grep -E '(franky|ros2)'"
echo ""

# 等待用户中断
wait