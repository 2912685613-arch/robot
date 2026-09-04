#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world underwater_world_noshadow.launch;exec bash"
}&
 
sleep 3s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_eight1.launch mode:=water x:=0 y:=0 z:=-0.1;exec bash"
}&
 
sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_eight_camera1;exec bash"
}&
 

