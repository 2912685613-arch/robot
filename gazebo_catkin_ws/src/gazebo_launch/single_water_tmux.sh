#!/bin/bash
# 单终端版本 - 适合WSL或不需要多窗口的环境
# 使用tmux分屏管理多个进程

source ~/robot/gazebo_catkin_ws/devel/setup.bash

echo "========================================"
echo "  启动单机器人水下仿真 (单终端版本)"
echo "========================================"
echo ""

# 检查tmux是否安装
if ! command -v tmux &> /dev/null; then
    echo "警告: tmux未安装，将顺序启动各组件"
    echo "建议安装tmux以获得更好的体验: sudo apt install tmux"
    echo ""

    # 不使用tmux的顺序启动
    echo "[1/3] 启动Gazebo世界..."
    roslaunch gazebo_world underwater_world_noshadow.launch &
    WORLD_PID=$!
    sleep 5

    echo "[2/3] 加载机器人模型..."
    roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1 &
    MODEL_PID=$!
    sleep 3

    echo "[3/3] 启动控制器..."
    roslaunch gazebo_control gazebo_control.launch mode:=water &
    CONTROL_PID=$!

    echo ""
    echo "========================================"
    echo "  所有组件已启动"
    echo "========================================"
    echo "进程ID:"
    echo "  - Gazebo世界: $WORLD_PID"
    echo "  - 机器人模型: $MODEL_PID"
    echo "  - 控制器: $CONTROL_PID"
    echo ""
    echo "按Ctrl+C停止所有进程"

    # 等待所有后台进程
    wait
else
    # 使用tmux
    SESSION_NAME="gazebo_sim"

    # 如果会话已存在，先关闭
    tmux has-session -t $SESSION_NAME 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "检测到已有会话，正在关闭..."
        tmux kill-session -t $SESSION_NAME
        sleep 1
    fi

    echo "使用tmux启动仿真环境..."
    echo "会话名称: $SESSION_NAME"
    echo ""

    # 创建新的tmux会话并启动第一个窗口（world）
    tmux new-session -d -s $SESSION_NAME -n "world" "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world underwater_world_noshadow.launch"

    # 等待Gazebo启动
    sleep 5

    # 创建第二个窗口（model）
    tmux new-window -t $SESSION_NAME -n "model" "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1"

    # 等待模型加载
    sleep 3

    # 创建第三个窗口（control）
    tmux new-window -t $SESSION_NAME -n "control" "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water"

    echo ""
    echo "========================================"
    echo "  仿真环境已在tmux中启动"
    echo "========================================"
    echo ""
    echo "使用以下命令管理tmux会话:"
    echo "  - 进入会话: tmux attach -t $SESSION_NAME"
    echo "  - 切换窗口: Ctrl+b 然后按数字 0/1/2"
    echo "  - 退出会话: Ctrl+b 然后按 d"
    echo "  - 关闭会话: tmux kill-session -t $SESSION_NAME"
    echo ""
    echo "窗口列表:"
    echo "  0: world   - Gazebo世界"
    echo "  1: model   - 机器人模型"
    echo "  2: control - 控制器"
    echo ""

    # 进入tmux会话
    tmux attach -t $SESSION_NAME
fi
