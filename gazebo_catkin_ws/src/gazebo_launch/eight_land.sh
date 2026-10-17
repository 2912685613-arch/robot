
#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_world gazebo_world.launch;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_eight2.launch;exec bash"
}&
 
sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch namespace:=gazebo_eight_camera2 config:=$(rospack find gazebo_control)/config/gazebo_control_eight_land.yaml;exec bash"
}&
 

