#!/bin/bash

# 启动Gazebo世界
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world simple_underwater.launch; exec bash" &

sleep 5s

# 加载机器人模型
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1; exec bash" &

sleep 3s

# 启动控制器
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water; exec bash" &

sleep 3s

# 启动机器人控制节点（包含mancontrol命令输入）
dbus-launch gnome-terminal --title="robot_control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch asrobot asrobot.launch; exec bash" &

echo "========================================"
echo "  仿真环境启动中..."
echo "========================================"
echo "已打开4个dbus-launch gnome-terminal窗口："
echo "  - world: Gazebo世界"
echo "  - model: 机器人模型"
echo "  - control: 控制器"
echo "  - robot_control: 机器人控制（命令输入窗口）"
echo ""
echo "在 robot_control 窗口输入命令控制机器人："
echo "  xyz 1 1 -1    # 全向运动"
echo "  yaw 90        # 航向控制"
echo "  pz -1         # 深度控制"
echo ""
