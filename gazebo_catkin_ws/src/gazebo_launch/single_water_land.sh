
###
 # @Author: ljl
 # @Date: 2025-03-18 10:06:34
 # @LastEditors: ljl
 # @LastEditTime: 2025-03-18 10:56:28
 # @FilePath: /gazebo_catkin_ws/src/gazebo_launch/single_water_land.sh
 # @Description: 
 # 
 # Copyright (c) 2025 by HEU_LJL, All Rights Reserved. 
### 
#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash

{
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world water_land_world_noshadow.launch;exec bash"
exit
}&

sleep 3s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch mode:=water x:=2 y:=2 z:=-0.1;exec bash"
}&
 

sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water;exec bash"
}&
 

