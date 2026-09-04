#!/bin/bash

# 启动Gazebo世界
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world simple_underwater.launch; exec bash" &

sleep 5s

# 加载机器人模型
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1; exec bash" &

sleep 3s

# 启动控制器
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water; exec bash" &

echo "========================================"
echo "  仿真环境启动中..."
echo "========================================"
echo "已打开3个gnome-terminal窗口："
echo "  - world: Gazebo世界"
echo "  - model: 机器人模型"
echo "  - control: 控制器"
echo ""
