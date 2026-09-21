
###
 # @Author: ljl
 # @Date: 2025-03-17 19:47:45
 # @LastEditors: ljl
 # @LastEditTime: 2025-03-17 19:50:24
 # @FilePath: /gazebo_catkin_ws/src/gazebo_launch/single_water_uuv.sh
 # @Description: 
 # 
 # Copyright (c) 2025 by HEU_LJL, All Rights Reserved. 
### 
#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash

{
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world underwater_world_noshadow1.launch;exec bash"
exit
}&

sleep 3s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1;exec bash"
}&
 

sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water;exec bash"
}&
 

