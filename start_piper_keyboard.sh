#!/bin/bash
# ===========================================
# 一键启动 Piper 控制节点 + 键盘控制 + 数据采集
# ===========================================

#set -m   # 启用作业控制
trap cleanup INT TERM

# ========= 配置 =========
CONDA_ENV="py310"
PIPER_WS="/home/data/Project/piper_ros"
TELEOP_WS="/home/data/Project/Deploy"
CAN_PORT="can0"
USB_PORT="/dev/ttyUSB1"
CAN_ACTIVATE_SCRIPT="$PIPER_WS/can_activate.sh"
LOG_FILE="$TELEOP_WS/piper.log"
CONTROL_SCRIPT="$TELEOP_WS/keyboard_piper_control.py"
RECORD_SCRIPT="$TELEOP_WS/data_recorder.py"
sudo chmod 666 $USB_PORT

# >>> Use Conda Python for ROS2 >>>
export PYTHON_EXECUTABLE=/home/hsb/miniforge3/envs/py310/bin/python
export PYTHONPATH=/home/hsb/miniforge3/envs/py310/lib/python3.10/site-packages:$PYTHONPATH
# <<< Use Conda Python for ROS2 <<<

# ========= 清理函数 =========
cleanup() {
    echo ""
    echo "[ACTION] 捕获退出信号，执行安全下电..."

    # 通知 ROS2 节点退出
    timeout 2 ros2 topic pub --once /record_cmd std_msgs/msg/String "data: 'stop'" >/dev/null 2>&1 || true
    sleep 0.3
    timeout 2 ros2 topic pub --once /record_cmd std_msgs/msg/String "data: 'exit'" >/dev/null 2>&1 || true
    timeout 2 ros2 topic pub --once /enable_flag std_msgs/msg/Bool "data: false" >/dev/null 2>&1 || true
    timeout 3 ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: false" >/dev/null 2>&1 || true

    for pid in $CONTROL_PID $RECORD_PID $PIPER_PID; do
        if [ -n "$pid" ] && ps -p "$pid" >/dev/null 2>&1; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    pkill -9 -f "piper_single_ctrl" 2>/dev/null || true
    sudo ifconfig $CAN_PORT down 2>/dev/null || true

    wait $CONTROL_PID $RECORD_PID $PIPER_PID 2>/dev/null || true

    echo "[INFO] ✅ 已安全退出"
    exit 0
}

# ========= 初始化环境 =========
echo "[INFO] 激活 conda 环境: $CONDA_ENV"
source ~/miniforge3/etc/profile.d/conda.sh
conda activate $CONDA_ENV

echo "[INFO] 加载 ROS2 Humble 与 Piper 环境"
source /opt/ros/humble/setup.bash
source $PIPER_WS/install/setup.bash

# ========= 清理旧节点 =========
echo "[INFO] 清理旧 Piper 节点..."
pkill -9 -f "piper_single_ctrl" 2>/dev/null || true
sleep 1

# ========= 激活 CAN 总线 =========
echo "[INFO] 激活 CAN 总线: $CAN_PORT"
if [ -f "$CAN_ACTIVATE_SCRIPT" ]; then
    bash $CAN_ACTIVATE_SCRIPT $CAN_PORT 1000000
else
    echo "[ERROR] 未找到 $CAN_ACTIVATE_SCRIPT"
    exit 1
fi
sleep 2

# ========= 启动 Piper 控制节点 =========
echo "[INFO] 启动 Piper 控制节点..."
ros2 run piper piper_single_ctrl --ros-args \
    -p can_port:=$CAN_PORT \
    -p auto_enable:=true \
    -p gripper_exist:=true \
    -p gripper_val_mutiple:=2 \
    --log-level WARN >"$LOG_FILE" 2>&1 &
PIPER_PID=$!
sleep 2

if ! ps -p $PIPER_PID >/dev/null 2>&1; then
    echo "[ERROR] Piper 控制节点启动失败，请检查 $LOG_FILE"
    exit 1
fi

# ========= 启动控制与采集 =========
echo "[INFO] 启动数据采集节点..."
python3 -u "$RECORD_SCRIPT" &
RECORD_PID=$!
sleep 1

echo "[INFO] 启动键盘控制节点..."
# 这里关键：确保 keyboard 控制脚本在交互式终端下运行
if [ -t 0 ]; then
    python3 "$CONTROL_SCRIPT" < /dev/tty &
else
    python3 "$CONTROL_SCRIPT" &
fi
CONTROL_PID=$!

echo "[INFO] ✅ 所有节点已启动"
echo "-------------------------------------"
echo "按 Ctrl+C 或 空格 键 退出控制"
echo "-------------------------------------"

wait $CONTROL_PID
cleanup
