#!/bin/bash

source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
dbus-launch gnome-terminal --title="multiComm" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title asrobot  multi_cast.launch;exec bash"
}&

sleep 3s
{
dbus-launch gnome-terminal --title="quadrotor" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title quad2robot quad2robot.launch;exec bash"
}&
 
