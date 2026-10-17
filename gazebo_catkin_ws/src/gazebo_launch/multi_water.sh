#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_world water_land_world_noshadow.launch;exec bash"
}&
#position of sand is 50 -20 -18
sleep 5s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title quad_gazebo quad_joystick_gps_empty_world.launch;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_model.launch x:=6 y:=8 z:=-0.16;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_model.launch color:=Red namespace:=gazebo_robot2 x:=7.5 y:=-7 z:=-0.16;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_model.launch color:=RedBright namespace:=gazebo_robot3 x:=6 y:=-6 z:=-0.16;exec bash"
}&

sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water;exec bash"
}&

sleep 3s
{
dbus-launch gnome-terminal --title="control2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_robot2;exec bash"

}&

sleep 3s
{
dbus-launch gnome-terminal --title="control3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water namespace:=gazebo_robot3;exec bash"
}&

