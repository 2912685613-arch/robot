#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world gazebo_world.launch;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model2.launch;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model3.launch;exec bash"
}&

sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch;exec bash"
}&

sleep 3s
{
dbus-launch gnome-terminal --title="control2" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch namespace:=gazebo_robot2;exec bash"

}&

sleep 3s
{
dbus-launch gnome-terminal --title="control3" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch namespace:=gazebo_robot3;exec bash"
}&

