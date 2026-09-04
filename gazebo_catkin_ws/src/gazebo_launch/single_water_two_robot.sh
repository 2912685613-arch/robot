

source ~/robot/gazebo_catkin_ws/devel/setup.bash

{
gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_world underwater_l.launch;exec bash"
exit
}&

sleep 3s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model.launch mode:=water x:=0 y:=0 z:=-0.1;exec bash"
}&
 

sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control_robot1.launch mode:=water;exec bash"
}&
 
sleep 3s
{
gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_model gazebo_model2.launch mode:=water x:=0 y:=0 z:=-1.0;exec bash"
}&
 

sleep 3s
{
gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch gazebo_control gazebo_control_robot2.launch mode:=water;exec bash"
}&
 

