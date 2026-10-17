#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_world underwater_world_noshadow.launch;exec bash"
}&
 
sleep 5s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_eight1.launch mode:=water x:=1 y:=-1 z:=-0.1;exec bash"
}&
 
sleep 5s
{
dbus-launch gnome-terminal --title="model2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_eight2.launch mode:=water x:=1 y:=1 z:=-0.1;exec bash"
}&
 
sleep 5s
{
dbus-launch gnome-terminal --title="model3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_model.launch mode:=water x:=0 y:=0 z:=-0.1;exec bash"
}&

sleep 5s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_eight_camera1 config:=$(rospack find gazebo_control)/config/gazebo_control_eight_water.yaml;exec bash"
}&

sleep 5s
{
dbus-launch gnome-terminal --title="control2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_eight_camera2 config:=$(rospack find gazebo_control)/config/gazebo_control_eight2_water.yaml;exec bash"

}&

sleep 5s
{
dbus-launch gnome-terminal --title="control3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_robot1;exec bash"
}&


