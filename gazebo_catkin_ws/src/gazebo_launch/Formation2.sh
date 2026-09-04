#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world underwater_world_noshadow.launch;exec bash"
}&
 
sleep 5s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_eight1.launch mode:=water x:=1 y:=-1 z:=-0.1;exec bash"
}&
 
sleep 5s
{
gnome-terminal --title="model2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_eight2.launch mode:=water x:=1 y:=1 z:=-0.1;exec bash"
}&
 
sleep 5s
{
gnome-terminal --title="model3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model1.launch mode:=water x:=0 y:=0 z:=-0.1;exec bash"
}&

sleep 5s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_eight_camera1;exec bash"
}&

sleep 5s
{
gnome-terminal --title="control2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_eight_camera2;exec bash"

}&

sleep 5s
{
gnome-terminal --title="control3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_robot1;exec bash"
}&


