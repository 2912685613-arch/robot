#!/bin/bash
# 动态 ArUco 定位/接近演示：后台启动各节点，当前终端保留命令输入。
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd -- "$SCRIPT_DIR/../.." && pwd)
GUI=${GUI:-true}
IMAGE_VIEW=${IMAGE_VIEW:-false}
STARTUP_TIMEOUT=${STARTUP_TIMEOUT:-60}
WAIT_PAUSED=${WAIT_PAUSED:-true}
ROBOT_X=${ROBOT_X:-0}
ROBOT_Y=${ROBOT_Y:-0}
ROBOT_Z=${ROBOT_Z:--0.1}
MARKER_X=${MARKER_X:-0.10}
MARKER_Y=${MARKER_Y:-0}
MARKER_Z=${MARKER_Z:--1.0}
LOG_RATE=${LOG_RATE:-2}
PYTHON_BIN=${PYTHON_BIN:-/usr/bin/python3}

usage() {
    cat <<'HELP'
用法：bash src/gazebo_launch/aruco_docking.sh
世界、动态机器人、控制器、标记和 ArUco 节点在后台启动，不打开额外终端。
当前终端输入 dock 恢复物理并启用接近，st 停止并暂停，reset 重置初始位姿，quit 退出。
Ctrl+C 或输入结束时，清理本次脚本启动的进程；不会使用 killall/pkill。

可选环境变量：
  GUI=true|false           是否显示 Gazebo，默认 true
  IMAGE_VIEW=true|false    是否打开图像查看器，默认 false（可在其中选择 /aruco_det_image）
  STARTUP_TIMEOUT=60      每个启动阶段的墙钟超时，秒
  ROBOT_X=0 ROBOT_Y=0 ROBOT_Z=-0.1
  MARKER_X=0.10 MARKER_Y=0 MARKER_Z=-1.0
  LOG_RATE=2             每秒位姿日志采样次数
  WAIT_PAUSED=true|false   等待命令时暂停物理以避免漂移，默认 true

清理旧脚本遗留进程：bash src/gazebo_launch/aruco_docking.sh --stop [本次日志目录]
示例：IMAGE_VIEW=true bash src/gazebo_launch/aruco_docking.sh
注意：使用现有 dock 控制代码，仅验证视觉对准/接近，不包含机械锁定和完成判定。
HELP
}
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    usage
    exit 0
fi
if [[ ${1:-} == --stop ]]; then
    shift
    exec "$PYTHON_BIN" "$SCRIPT_DIR/cleanup_aruco_docking.py" "$@"
fi
if (( $# != 0 )); then
    usage >&2
    exit 2
fi
if [[ ! $STARTUP_TIMEOUT =~ ^[1-9][0-9]*$ ]]; then
    echo "STARTUP_TIMEOUT 必须是正整数。" >&2
    exit 2
fi
if [[ $GUI != true && $GUI != false ]] || [[ $IMAGE_VIEW != true && $IMAGE_VIEW != false ]] ||
   [[ $WAIT_PAUSED != true && $WAIT_PAUSED != false ]]; then
    echo "GUI、IMAGE_VIEW 和 WAIT_PAUSED 必须为 true 或 false。" >&2
    exit 2
fi
if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
    echo "找不到 devel/setup.bash，请先编译工作空间。" >&2
    exit 1
fi
set +u
source "$WS_DIR/devel/setup.bash"
set -u
for executable in roslaunch rosrun rostopic setsid timeout "$PYTHON_BIN"; do
    command -v "$executable" >/dev/null || { echo "找不到命令: $executable" >&2; exit 1; }
done

# 仅检查现有 Master，不启动仿真；防止和之前的世界/同名机器人节点冲突。
"$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" preflight
mkdir -p "$WS_DIR/logs"
RUN_DIR=$(mktemp -d "$WS_DIR/logs/aruco_docking_$(date +%Y%m%d_%H%M%S)_XXXXXX")
# 所有 ROS 节点自己的日志也集中到本次目录（另外保留位姿 CSV 和启动标准输出）。
export ROS_LOG_DIR="$RUN_DIR/ros"
export ARUCO_DOCKING_SESSION="$RUN_DIR"
mkdir -p "$ROS_LOG_DIR"
PIDS=()
LABELS=()
CLEANING=false

cleanup() {
    local status=$? pid deadline alive
    if [[ $CLEANING == true ]]; then return; fi
    CLEANING=true
    trap '' INT TERM
    if [[ -n ${monitor_pid:-} ]]; then
        kill "$monitor_pid" 2>/dev/null || true
        wait "$monitor_pid" 2>/dev/null || true
    fi
    echo
    echo "正在关闭本次启动的节点；日志保留在 $RUN_DIR"
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || true
    done
    # roslaunch 的子节点会创建自己的 session，单独杀外层进程组无法覆盖它们。
    # 按继承的运行标记/日志目录重扫，并核对 PID 的内核启动时间。
    "$PYTHON_BIN" "$SCRIPT_DIR/cleanup_aruco_docking.py" "$RUN_DIR" \
        >>"$RUN_DIR/cleanup.log" 2>&1 || true
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    sleep 0.3
    for pid in "${PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    return "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_background() {
    local label=$1
    shift
    echo "启动: $label"
    # 独立会话使清理作用于该 roslaunch 及其子进程，不影响用户的其他程序。
    setsid "$@" >"$RUN_DIR/$label.log" 2>&1 < /dev/null &
    PIDS+=("$!")
    LABELS+=("$label")
}

check_processes() {
    local i status
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
            status=0
            wait "${PIDS[$i]}" || status=$?
            echo "后台节点已退出: ${LABELS[$i]}（退出码 $status）" >&2
            tail -n 15 "$RUN_DIR/${LABELS[$i]}.log" >&2
            return 1
        fi
    done
}

wait_ready() {
    local stage=$1 helper=$SCRIPT_DIR/wait_single_water.py
    if [[ $stage == detection || $stage == command ]]; then
        helper=$SCRIPT_DIR/wait_aruco_docking.py
    fi
    echo "等待就绪: $stage"
    if ! timeout -k 2 "$((STARTUP_TIMEOUT + 5))s" "$PYTHON_BIN" "$helper" "$stage" \
        --timeout "$STARTUP_TIMEOUT" >"$RUN_DIR/ready_$stage.log" 2>&1; then
        cat "$RUN_DIR/ready_$stage.log" >&2
        echo "启动检查失败，详细日志: $RUN_DIR" >&2
        return 1
    fi
    check_processes
}

cat > "$RUN_DIR/startup.txt" <<CONFIG
robot_xyz=$ROBOT_X $ROBOT_Y $ROBOT_Z
marker_xyz=$MARKER_X $MARKER_Y $MARKER_Z
marker_outer_id=5 marker_outer_black_border_m=0.2
marker_inner_id=18 marker_inner_black_border_m=0.02
gui=$GUI image_view=$IMAGE_VIEW log_rate=$LOG_RATE wait_paused=$WAIT_PAUSED
ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
CONFIG

echo "本次日志: $RUN_DIR"
start_background world roslaunch --disable-title gazebo_world simple_underwater.launch "gui:=$GUI"
wait_ready world
start_background model roslaunch --disable-title new_gazebo_model new_gazebo_model.launch \
    "x:=$ROBOT_X" "y:=$ROBOT_Y" "z:=$ROBOT_Z" enable_bottom_camera:=true static_model:=false
wait_ready model
start_background controllers roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water
wait_ready controllers

# spawn_model 是一次性命令，完成后退出，不计入常驻节点健康检查。
echo "生成同心 ID 5/18 双码标记..."
setsid timeout -k 2 "${STARTUP_TIMEOUT}s" rosrun gazebo_ros spawn_model \
    -sdf -file "$WS_DIR/src/gazebo_world/models/aruco_demo_marker/model.sdf" \
    -model aruco_demo_marker -x "$MARKER_X" -y "$MARKER_Y" -z "$MARKER_Z" \
    >"$RUN_DIR/marker.log" 2>&1 < /dev/null &
marker_pid=$!
PIDS+=("$marker_pid")
LABELS+=(marker)
if ! wait "$marker_pid"; then
    cat "$RUN_DIR/marker.log" >&2
    exit 1
fi
unset "PIDS[$((${#PIDS[@]} - 1))]" "LABELS[$((${#LABELS[@]} - 1))]"

start_background robot_control roslaunch --disable-title asrobot asrobot.launch \
    enable_aruco:=false enable_mancontrol:=false \
    "joint_command_config:=$WS_DIR/src/new_gazebo_model/config/joint_command_calibration.yaml"
wait_ready feedback
start_background aruco roslaunch --disable-title asrobot aruco_det.launch \
    aruco_length_outer:=0.2 aruco_id_inner:=18 aruco_length_inner:=0.02 marker_model_name:=aruco_demo_marker \
    "log_dir:=$RUN_DIR" "log_rate:=$LOG_RATE"
wait_ready command
if [[ $WAIT_PAUSED == true ]]; then
    echo "重置启动期间的漂移，取得定位后暂停等待 dock..."
    if ! timeout -k 2 "$((STARTUP_TIMEOUT + 10))s" "$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" prepare \
        --timeout "$STARTUP_TIMEOUT" --x "$ROBOT_X" --y "$ROBOT_Y" --z "$ROBOT_Z" \
        >"$RUN_DIR/ready_prepare.log" 2>&1; then
        cat "$RUN_DIR/ready_prepare.log" >&2
        echo "准备阶段未取得有效定位，保持暂停；可输入 reset 重试，或查看图像。" >&2
    fi
    # 即使准备检查被外层 timeout 中止，也明确再执行暂停；暂停失败不继续运行。
    timeout -k 2 6s "$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" pause --timeout 3
elif ! wait_ready detection; then
    echo "未确认有效标记位姿；dock 前会重新检查。" >&2
fi
check_processes
if [[ $IMAGE_VIEW == true ]]; then
    # 图像窗口是可关闭的辅助工具，不计入核心节点健康检查。
    setsid rosrun rqt_image_view rqt_image_view >"$RUN_DIR/image_view.log" 2>&1 < /dev/null &
    viewer_pid=$!
fi

cat <<HELP

动态仿真已启动；当前终端就是命令窗口，不需要另开终端。
  dock      恢复物理，检查新鲜标记位姿后启用现有对准/接近控制
  st        停止运动；WAIT_PAUSED=true 时同时暂停世界
  reset     停止并重置初始位姿/速度，重新定位后暂停
  status    查看机器人、标记、相机和检测状态
  quit      退出并清理本次启动的进程（也可 Ctrl+C）
其他命令（如 xyz 0 0 -0.3、dep -0.3）原样发到 /robot1/cmd_message。
不会自动发送 dock。WAIT_PAUSED=$WAIT_PAUSED；暂停时机器人和相机画面都停在当前时刻。
当前控制代码没有对接成功/机械锁定判定，输入 dock 后没有新增悬停控制。
日志目录: $RUN_DIR
HELP
# 查看器可以随时关掉；退出脚本时也回收它，但不让其提前退出触发仿真停止。
if [[ -n ${viewer_pid:-} ]]; then
    PIDS+=("$viewer_pid")
    LABELS+=(image_view)
fi

# 核心节点健康检查排除可关闭的查看器。
check_core() {
    local i
    if [[ -n ${viewer_pid:-} ]]; then
        for i in "${!PIDS[@]}"; do
            if [[ ${PIDS[$i]} == "$viewer_pid" ]]; then
                # 不从数组移除，保留其进程组用于退出清理。
                continue
            fi
            if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
                echo "后台节点已退出: ${LABELS[$i]}，请查看 $RUN_DIR/${LABELS[$i]}.log" >&2
                return 1
            fi
        done
    else
        check_processes
    fi
}

# 后台监视器检测核心节点退出；命令输入使用完整行读取，不因轮询截断慢速输入。
(
    trap - EXIT INT TERM
    while sleep 1; do
        if ! check_core; then
            kill -TERM "$$"
            exit 1
        fi
    done
) &
monitor_pid=$!

physics() {
    timeout -k 2 6s "$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" "$1" --timeout 3
}

send_command() {
    timeout -k 2 6s rostopic pub -1 /robot1/cmd_message std_msgs/String -- "$1" \
        >>"$RUN_DIR/commands.log" 2>&1
}

printf 'aruco> '
while IFS= read -r command_text; do
    command_text="${command_text#"${command_text%%[![:space:]]*}"}"
    command_text="${command_text%"${command_text##*[![:space:]]}"}"
    check_core
    case "$command_text" in
        '') ;;
        quit|exit) break ;;
        help) usage ;;
        status)
            timeout -k 2 5s "$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" status || true
            ;;
        pause)
            # 用户主动暂停先取消对接；区别于渲染/仿真卡顿后的传输恢复。
            send_command st || { echo "停止命令未确认，请查看 commands.log" >&2; }
            physics pause
            ;;
        resume) physics resume ;;
        reset)
            physics resume
            send_command st || { physics pause; continue; }
            if ! timeout -k 2 "$((STARTUP_TIMEOUT + 10))s" "$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" prepare \
                --timeout "$STARTUP_TIMEOUT" --x "$ROBOT_X" --y "$ROBOT_Y" --z "$ROBOT_Z" \
                >>"$RUN_DIR/ready_prepare.log" 2>&1; then
                echo "重置后仍未取得有效定位，请查看 ready_prepare.log。" >&2
            fi
            physics pause
            ;;
        *)
            # 必须先恢复物理和 /clock，motion 的 spinOnce 才能处理新命令。
            physics resume
            if [[ $command_text == dock ]]; then
                if ! timeout -k 2 6s "$PYTHON_BIN" "$SCRIPT_DIR/wait_aruco_docking.py" detection --timeout 3; then
                    echo "当前没有新鲜有效的标记位姿，本次未发送 dock。可输入 reset 恢复初始位置。"
                    if [[ $WAIT_PAUSED == true ]]; then physics pause; fi
                    printf 'aruco> '
                    continue
                fi
            fi
            if ! send_command "$command_text"; then
                echo "命令发送失败，请查看 $RUN_DIR/commands.log" >&2
                if [[ $WAIT_PAUSED == true ]]; then physics pause; fi
            elif [[ $command_text == st && $WAIT_PAUSED == true ]]; then
                physics pause
            fi
            ;;
    esac
    printf 'aruco> '
done
