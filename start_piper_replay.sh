#!/bin/bash
# ===========================================
# 一键启动 Piper 控制节点 + 轨迹复现
# ===========================================

trap cleanup INT TERM

# ========= 配置 =========
CONDA_ENV="py310"
PIPER_WS="/home/data/Project/piper_ros"
DEPLOY_WS="/home/data/Project/Deploy"
CAN_PORT="can0"
USB_PORT="/dev/ttyUSB1"
CAN_ACTIVATE_SCRIPT="$PIPER_WS/can_activate.sh"
LOG_FILE="$DEPLOY_WS/piper_replay.log"
REPLAY_SCRIPT="$DEPLOY_WS/sample_deploy.py"

DATASET_ROOT="/home/data/Dataset/oracle_dataset_raw"
EPISODE="${1:-episode_00000}"
RATE="${RATE:-10}"
WARMUP="${WARMUP:-3}"
LOOP_FLAG=""

# >>> Use Conda Python for ROS2 >>>
export PYTHON_EXECUTABLE=/home/hsb/miniforge3/envs/py310/bin/python
export PYTHONPATH=/home/hsb/miniforge3/envs/py310/lib/python3.10/site-packages:$PYTHONPATH
# <<< Use Conda Python for ROS2 <<<


if [ "${LOOP:-0}" -eq 1 ] || [ "${2:-}" = "--loop" ]; then
    LOOP_FLAG="--loop"
fi

sudo chmod 666 $USB_PORT

REPLAY_PID=""
PIPER_PID=""

# ========= 清理函数 =========
cleanup() {
    echo ""
    echo "[ACTION] 捕获退出信号，执行安全下电..."

    timeout 2 ros2 topic pub --once /enable_flag std_msgs/msg/Bool "data: false" >/dev/null 2>&1 || true
    timeout 3 ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: false" >/dev/null 2>&1 || true

    for pid in $REPLAY_PID $PIPER_PID; do
        if [ -n "$pid" ] && ps -p "$pid" >/dev/null 2>&1; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    pkill -9 -f "piper_single_ctrl" 2>/dev/null || true
    sudo ifconfig $CAN_PORT down 2>/dev/null || true

    wait $REPLAY_PID $PIPER_PID 2>/dev/null || true

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
    cleanup
fi

# ========= 启动轨迹复现 =========
echo "[INFO] 启动轨迹复现: $EPISODE (rate=${RATE}Hz)"
python3 -u "$REPLAY_SCRIPT" \
    --dataset-root "$DATASET_ROOT" \
    --episode "$EPISODE" \
    --rate "$RATE" \
    --warmup "$WARMUP" \
    $LOOP_FLAG &
REPLAY_PID=$!

wait $REPLAY_PID
cleanup
