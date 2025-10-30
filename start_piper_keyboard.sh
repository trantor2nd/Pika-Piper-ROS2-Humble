#!/bin/bash
#set -e  # 一旦出错立即退出
trap cleanup INT TERM  # 捕获 Ctrl+C / kill 信号
# ========= 配置部分 =========
CONDA_ENV="py310"
PIPER_WS="/home/data/Project/piper_ros"
TELEOP_WS="/home/data/Project/Deploy"
CAN_PORT="can0"
CAN_ACTIVATE_SCRIPT="$PIPER_WS/can_activate.sh"
LOG_FILE="$TELEOP_WS/piper.log"


cleanup() {
    echo "[ACTION] 捕获退出信号，执行安全下电..."
    ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: false" >/dev/null 2>&1 || \
    ros2 topic pub /enable_flag std_msgs/msg/Bool "data: false" -1 >/dev/null 2>&1
    if [ -n "$PIPER_PID" ] && ps -p $PIPER_PID >/dev/null 2>&1; then
        kill $PIPER_PID 2>/dev/null || true
    fi
    pkill -9 -f "piper_single_ctrl" 2>/dev/null || true
    sudo ifconfig $CAN_PORT down 2>/dev/null || true
    echo "[INFO] 已安全退出 ✅"
    exit 0
}

# ========= 初始化 =========
echo "[INFO] 激活 conda 环境: $CONDA_ENV"
source ~/miniforge3/etc/profile.d/conda.sh
conda activate $CONDA_ENV

echo "[INFO] 加载 ROS2 Humble"
source /opt/ros/humble/setup.bash

echo "[INFO] 加载 Piper 工作空间"
source $PIPER_WS/install/setup.bash

# ========= 清理旧进程 =========
echo "[INFO] 清理旧的 Piper 节点..."
pkill -9 -f "piper_single_ctrl" 2>/dev/null
pkill -9 -f "ros2 run piper" 2>/dev/null
pkill -9 -f "ros2 launch piper" 2>/dev/null
sleep 1

# ========= 激活 CAN =========
echo "[INFO] 激活 CAN 总线: $CAN_PORT"
if [ -f "$CAN_ACTIVATE_SCRIPT" ]; then
    bash $CAN_ACTIVATE_SCRIPT $CAN_PORT 1000000
    sleep 2
else
    echo "[ERROR] 未找到 $CAN_ACTIVATE_SCRIPT，请确认路径是否正确"
    exit 1
fi

# ========= 启动 Piper 控制节点（自动上电） =========
echo "[INFO] 启动 Piper 控制节点 (auto_enable=true)..."
ros2 run piper piper_single_ctrl --ros-args \
    -p can_port:=$CAN_PORT \
    -p auto_enable:=true \
    -p gripper_exist:=true \
    -p gripper_val_mutiple:=2 \
    --log-level WARN >"$LOG_FILE" 2>&1 &
PIPER_PID=$!
sleep 2

# ========= 检查节点是否正常启动 =========
if ! ps -p $PIPER_PID >/dev/null 2>&1; then
    echo "[ERROR] Piper 控制节点启动失败，请检查 $LOG_FILE"
    exit 1
fi

# ========= 启动键盘控制 =========
echo "[INFO] 启动键盘控制 (按空格退出)"
python3 $TELEOP_WS/keyboard_piper_control.py

# ========= 安全退出 =========
cleanup