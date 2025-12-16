#!/bin/bash

# 脚本状态信息
echo "🚀 启动 FACTR 遥操作系统..."
echo "   - Franky Bridge: Franka 机器人通信"
echo "   - ROS2 Launch: 遥操作和相机节点"
echo ""

# 全局变量存储进程ID
FRANKY_PID=""
ROS_PID=""
STOP_STAGE=0


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

    # 第二次 Ctrl+C：再停 Franky 和执行最终清理
    if [ -n "$FRANKY_PID" ] && kill -0 "$FRANKY_PID" 2>/dev/null; then
        echo "   停止 Franky Bridge (PID: $FRANKY_PID)..."
        kill -SIGINT "$FRANKY_PID" 2>/dev/null || kill -9 "$FRANKY_PID" 2>/dev/null
    fi

    # 执行最终清理
    echo "   执行最终清理..."
    pkill -f "python.*franka_factr_bridge" 2>/dev/null || true
    pkill -f "ros2.*launch" 2>/dev/null || true
    pkill -f "factr_teleop_franka" 2>/dev/null || true
    pkill -f "zmq_ros_bridge" 2>/dev/null || true
    pkill -f "gripper_bridge_node" 2>/dev/null || true
    pkill -f "data_record" 2>/dev/null || true
    pkill -f "realsense" 2>/dev/null || true

    # 停止ROS daemon
    ros2 daemon stop 2>/dev/null || true

    wait 2>/dev/null

    # 最终检查
    remaining=$(ps aux | grep -E "(franka_factr_bridge|factr_teleop|zmq_ros_bridge|gripper_bridge|data_record|realsense)" | grep -v grep | wc -l)
    if [ $remaining -eq 0 ]; then
        echo "   ✅ 所有程序已停止并清理完成"
    else
        echo "   ⚠️  仍有 $remaining 个进程残留，已尽力清理"
    fi

    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 清理可能的残留进程和端口
echo "🧹 清理可能的残留进程..."

# 1. 清理Franka相关进程
echo "   清理Franka桥接进程..."
pkill -f "franka_factr_bridge_franky" 2>/dev/null || true
pkill -f "franky.*bridge" 2>/dev/null || true

# 2. 清理ROS2相关进程
echo "   清理ROS2进程..."
pkill -f "ros2.*launch.*collect_data" 2>/dev/null || true
pkill -f "ros2.*launch.*teleop_system" 2>/dev/null || true
pkill -f "factr_teleop_franka" 2>/dev/null || true
pkill -f "gripper_bridge_node" 2>/dev/null || true
pkill -f "zmq_ros_bridge" 2>/dev/null || true
pkill -f "data_record" 2>/dev/null || true

# 3. 清理相机相关进程
echo "   清理相机进程..."
pkill -f "realsense" 2>/dev/null || true
pkill -f "cameras.*realsense" 2>/dev/null || true

# 4. 清理可能的Python进程
echo "   清理Python残留进程..."
pkill -f "python.*franka_factr_bridge" 2>/dev/null || true
pkill -f "python.*factr_teleop" 2>/dev/null || true
pkill -f "python.*zmq_ros_bridge" 2>/dev/null || true

# 5. 强制清理可能残留的ROS节点
echo "   清理ROS节点..."
ros2 daemon stop 2>/dev/null || true
pkill -f "ros2.*daemon" 2>/dev/null || true

# 6. 清理可能的ZMQ端口占用
echo "   清理网络端口..."
# 查找并终止占用相关端口的进程
for port in 2098 3099 3087 3098 3100; do
    pid=$(lsof -ti :$port 2>/dev/null)
    if [ ! -z "$pid" ]; then
        echo "     终止占用端口 $port 的进程 (PID: $pid)"
        kill -9 $pid 2>/dev/null || true
    fi
done

# 7. 等待清理完成并验证
echo "   等待清理完成..."
sleep 3

# 8. 检查是否还有残留进程
echo "   检查清理结果..."
remaining_processes=$(ps aux | grep -E "(franka_factr_bridge|factr_teleop|zmq_ros_bridge|gripper_bridge|data_record|realsense)" | grep -v grep | wc -l)
if [ $remaining_processes -gt 0 ]; then
    echo "   ⚠️  发现 $remaining_processes 个残留进程，尝试强制清理..."
    ps aux | grep -E "(franka_factr_bridge|factr_teleop|zmq_ros_bridge|gripper_bridge|data_record|realsense)" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null || true
    sleep 2
fi

# 9. 最终验证
final_check=$(ps aux | grep -E "(franka_factr_bridge|factr_teleop|zmq_ros_bridge|gripper_bridge|data_record|realsense)" | grep -v grep | wc -l)
if [ $final_check -eq 0 ]; then
    echo "   ✅ 清理完成"
else
    echo "   ⚠️  仍有 $final_check 个进程残留，可能影响启动"
    echo "   残留进程详情:"
    ps aux | grep -E "(franka_factr_bridge|factr_teleop|zmq_ros_bridge|gripper_bridge|data_record|realsense)" | grep -v grep
fi

# 10. 清理可能的临时文件
echo "   清理临时文件..."
rm -rf /tmp/launch_params_* 2>/dev/null || true
rm -rf /tmp/params_file_* 2>/dev/null || true

# Franky Bridge (后台运行)
echo "🔧 启动 Franky Bridge..."
(
    echo "[Franky] 激活 Franka 环境..."
    source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || echo "[Franky] 警告: 无法加载conda环境"
    conda activate franka 2>/dev/null || echo "[Franky] 警告: 无法激活franka环境"

    echo "[Franky] 启动 Franka-FACTR Bridge..."
    if [ -f "/home/cytoderm/anaconda3/envs/franka/bin/python3.11" ]; then
        /home/cytoderm/anaconda3/envs/franka/bin/python3.11 franka_factr_bridge_franky.py
    else
        echo "[Franky] 错误: Python环境不存在，使用系统Python..."
        python3 franka_factr_bridge_franky.py
    fi
) &
FRANKY_PID=$!
echo "   Franky Bridge 已启动 (PID: $FRANKY_PID)"

# 等待并检查Franky是否成功启动
echo "   等待 Franky Bridge 初始化..."
sleep 3
if kill -0 $FRANKY_PID 2>/dev/null; then
    echo "   ✅ Franky Bridge 启动成功"
else
    echo "   ❌ Franky Bridge 启动失败，请检查日志"
    exit 1
fi

# ROS2 Launch (后台运行)
echo "🤖 启动 ROS2 系统..."
(
    echo "[ROS2] 配置环境..."
    # 激活Python虚拟环境
    if [ -f ".venv/bin/activate" ]; then
        source .venv/bin/activate
        echo "[ROS2] 已激活Python虚拟环境"
    else
        echo "[ROS2] 警告: 虚拟环境不存在，使用系统Python"
    fi

    # 设置ROS2环境
    if [ -f "install/local_setup.bash" ]; then
        source install/local_setup.bash
        source install/setup.bash
        echo "[ROS2] 已设置ROS2环境"
    else
        echo "[ROS2] 错误: ROS2工作空间未构建，请先运行 'colcon build'"
        exit 1
    fi

    echo "[ROS2] 启动遥操作和相机节点..."
    # 检查launch文件是否存在
    if [ -f "src/factr_teleop/launch/collect_data.launch.py" ]; then
        ros2 launch factr_teleop collect_data.launch.py
    else
        echo "[ROS2] 错误: launch文件不存在"
        exit 1
    fi
) &
ROS_PID=$!
echo "   ROS2 系统已启动 (PID: $ROS_PID)"

# 等待并检查ROS2是否成功启动
echo "   等待 ROS2 系统初始化..."
sleep 5
if kill -0 $ROS_PID 2>/dev/null; then
    echo "   ✅ ROS2 系统启动成功"
else
    echo "   ❌ ROS2 系统启动失败，请检查日志"
    exit 1
fi

# 启动后验证
echo ""
echo "🔍 验证系统启动状态..."

# 检查关键进程
echo "   检查进程状态:"
if kill -0 $FRANKY_PID 2>/dev/null; then
    echo "   ✅ Franky Bridge (PID: $FRANKY_PID) - 运行中"
else
    echo "   ❌ Franky Bridge - 未运行"
fi

if kill -0 $ROS_PID 2>/dev/null; then
    echo "   ✅ ROS2 Launch (PID: $ROS_PID) - 运行中"
else
    echo "   ❌ ROS2 Launch - 未运行"
fi

# 检查ROS节点
echo "   检查ROS节点:"
ros2_node_count=$(ros2 node list 2>/dev/null | wc -l)
if [ $ros2_node_count -gt 0 ]; then
    echo "   ✅ ROS节点数量: $ros2_node_count"
    echo "   关键节点列表:"
    ros2 node list 2>/dev/null | grep -E "(factr_teleop|franka|zmq|data_record)" | sed 's/^/     - /'
else
    echo "   ❌ 无ROS节点运行"
fi

# 检查关键话题
echo "   检查关键话题:"
key_topics=$(ros2 topic list 2>/dev/null | grep -E "(cmd_franka|obs_franka)" | wc -l)
if [ $key_topics -gt 0 ]; then
    echo "   ✅ 发现 $key_topics 个关键话题"
else
    echo "   ⚠️  未发现关键话题，可能遥操作未正常工作"
fi

echo ""
echo "✅ 系统启动完成！"
echo "   Franky Bridge (PID: $FRANKY_PID)"
echo "   ROS2 Launch (PID: $ROS_PID)"
echo ""
echo "💡 按 Ctrl+C 停止所有程序 (分两阶段清理)"
echo "📊 查看运行状态: ps aux | grep -E '(franky|ros2|factr)'"
echo "🔍 监控系统: ros2 node list && ros2 topic list"
echo ""

# 等待用户中断
wait