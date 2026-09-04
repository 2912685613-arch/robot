
#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash

{
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world water_land_world_noshadow.launch;exec bash"
exit
}&
#position of sand is 50 -20 -18

sleep 5s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch quad_gazebo quad_joystick_gps_empty_world.launch;exec bash"
}&



sleep 3s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch x:=0 y:=0 z:=-0.1;exec bash"
}&
 

sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch mode:=water;exec bash"
}&
 
sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch quad2robot quad2robot.launch ;exec bash"
}&
