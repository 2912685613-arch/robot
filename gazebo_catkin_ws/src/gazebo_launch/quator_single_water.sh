
#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash

#{
#dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world underwater_world_noshadow.launch;exec bash"
#exit
#}&


#sleep 5s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch quad_gazebo quad_joystick_gps_empty_world.launch;exec bash"
}&



sleep 3s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch x:=6 y:=3 z:=-0.1;exec bash"
}&
 

sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch quad2robot quad2robot.launch ;exec bash"
}&
