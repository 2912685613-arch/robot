
#!/bin/bash
source ~/robot/gazebo_catkin_ws/devel/setup.bash
{
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world gazebo_world.launch;exec bash"
}&
 
sleep 3s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_eight2.launch;exec bash"
}&
 
sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control.launch namespace:=gazebo_eight_camera2;exec bash"
}&
 

