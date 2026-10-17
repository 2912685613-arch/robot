

source ~/robot/gazebo_catkin_ws/devel/setup.bash

{
dbus-launch gnome-terminal --title="world" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_world underwater_l.launch;exec bash"
exit
}&

sleep 3s
{
dbus-launch gnome-terminal --title="model" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_model gazebo_model.launch mode:=water x:=0 y:=0 z:=-0.1;exec bash"
}&


sleep 3s
{
dbus-launch gnome-terminal --title="control" -- bash -c "source ~/robot/gazebo_catkin_ws/devel/setup.bash && roslaunch --disable-title gazebo_control gazebo_control.launch mode:=water;exec bash"
}&
 

