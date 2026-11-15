#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd -- "$SCRIPT_DIR/../.." && pwd)
LOGS_DIR="$WS_DIR/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
GUI=${GUI:-true}
ENABLE_ARUCO=${ENABLE_ARUCO:-false}
STARTUP_TIMEOUT=${STARTUP_TIMEOUT:-60}
TERMINAL_APP=${TERMINAL_APP:-auto}
TERMINAL_APP_REQUESTED=$TERMINAL_APP

if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
    echo "找不到 $WS_DIR/devel/setup.bash，请先编译工作空间。" >&2
    exit 1
fi
# Use one session bus for the launch windows and the interactive command node.
if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && "$TERMINAL_APP" != xterm ]]; then
    exec dbus-launch --exit-with-session bash "$SCRIPT_DIR/single_water.sh" "$@"
fi
mkdir -p "$LOGS_DIR"
# ROS setup scripts may reference unset environment variables.
set +u
source "$WS_DIR/devel/setup.bash"
set -u

launch_window() {
    local title=$1
    shift
    local script='
        set -o pipefail
        source "$1"
        log=$2
        shift 2
        "$@" 2>&1 | tee "$log"
        status=${PIPESTATUS[0]}
        echo "roslaunch 已退出，状态码: $status；日志: $log"
        exec bash
    '
    if [[ "$TERMINAL_APP" == auto || "$TERMINAL_APP" == gnome-terminal ]]; then
        if gnome-terminal --title="$title" -- bash -c "$script" _ \
            "$WS_DIR/devel/setup.bash" "$LOGS_DIR/${title}_${TIMESTAMP}.log" "$@"; then
            TERMINAL_APP=gnome-terminal
            return
        fi
        if [[ "$TERMINAL_APP_REQUESTED" == gnome-terminal ]]; then
            echo "GNOME Terminal 启动失败；请用 TERMINAL_APP=xterm 重试。" >&2
            return 1
        fi
        echo "GNOME Terminal 无法连接，改用 xterm 启动窗口。" >&2
        TERMINAL_APP=xterm
    fi
    if [[ "$TERMINAL_APP" != xterm ]]; then
        echo "不支持的 TERMINAL_APP: $TERMINAL_APP (可用: auto, gnome-terminal, xterm)" >&2
        return 1
    fi
    if ! command -v xterm >/dev/null; then
        echo "找不到 xterm；请安装 xterm 或修复 GNOME Terminal 会话。" >&2
        return 1
    fi
    xterm -T "$title" -e bash -c "$script" _ \
        "$WS_DIR/devel/setup.bash" "$LOGS_DIR/${title}_${TIMESTAMP}.log" "$@" &
    local terminal_pid=$!
    sleep 0.3
    if ! kill -0 "$terminal_pid" 2>/dev/null; then
        echo "xterm 未能打开；请检查 DISPLAY 和图形会话。" >&2
        return 1
    fi
}

wait_ready() {
    if ! timeout "${STARTUP_TIMEOUT}s" python3 "$SCRIPT_DIR/wait_single_water.py" "$1" --timeout "$STARTUP_TIMEOUT"; then
        echo "停止启动后续节点；请检查 $LOGS_DIR 中本次启动的日志。" >&2
        exit 1
    fi
}

launch_window world roslaunch --disable-title gazebo_world simple_underwater.launch "gui:=$GUI"
wait_ready world

launch_window model roslaunch --disable-title new_gazebo_model new_gazebo_model.launch x:=0 y:=0 z:=-0.1 "enable_bottom_camera:=$ENABLE_ARUCO"
wait_ready model

launch_window controllers roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water
wait_ready controllers

if [[ "$TERMINAL_APP" == xterm ]]; then
    command_terminal_prefix='xterm -T command -e'
else
    command_terminal_prefix='gnome-terminal --wait --title=command --'
fi
launch_window robot_control roslaunch --disable-title asrobot asrobot.launch "enable_aruco:=$ENABLE_ARUCO" \
    "mancontrol_launch_prefix:=$command_terminal_prefix" \
    "joint_command_config:=$WS_DIR/src/new_gazebo_model/config/joint_command_calibration.yaml"
wait_ready feedback

cat <<EOF
========================================
  单机器人水下仿真已启动
========================================
world: Gazebo 世界；model: 模型加载
controllers: 关节控制器（不是命令输入窗口）
robot_control: 控制节点日志（不是命令输入窗口）
command: 键盘命令输入，出现 please input cmd! 后输入：
  fuzzypid on  # 所有手动 PID 使用模糊增益（默认开启）
  fuzzypid off # 所有手动 PID 使用经典固定增益
  xyz 1 1 -1    # 目标位置，单位 m
  yaw 90       # 航向控制
  dep -1       # 单环深度控制
  depyaw -1 90 # 深度与航向组合控制
  pz -1        # 深度双环控制
  vz           # 速度内环调试，期望速度 -0.15 m/s
  st           # 停止

默认使用限速、无阴影世界，关闭本次任务不需要的 ArUco 节点。
需要视觉功能：ENABLE_ARUCO=true bash "$SCRIPT_DIR/single_water.sh"
不需要 Gazebo GUI：GUI=false bash "$SCRIPT_DIR/single_water.sh"
终端无法连接 D-Bus：TERMINAL_APP=xterm bash "$SCRIPT_DIR/single_water.sh"
日志目录：$LOGS_DIR
EOF
