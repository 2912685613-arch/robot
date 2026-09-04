#!/bin/bash

source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
gnome-terminal --title="multiComm" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch asrobot  multi_cast.launch;exec bash"
}&

sleep 3s
{
gnome-terminal --title="quadrotor" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch quad2robot quad2robot.launch;exec bash"
}&
 
