#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <uuv_gazebo_ros_plugins_msgs/GetThrusterState.h>
#include <uuv_gazebo_ros_plugins_msgs/SetThrusterState.h>
#include "asrobot/pwm.h"
#include "math.h"

#include <iostream>
#include <stdio.h>

using namespace std;//hcy

uuv_gazebo_ros_plugins_msgs::FloatStamped rotor;
uuv_gazebo_ros_plugins_msgs::GetThrusterState getsrv;
uuv_gazebo_ros_plugins_msgs::SetThrusterState setsrv;

ros::Publisher Propeller1_pub,Propeller2_pub,Propeller3_pub,Propeller4_pub;
ros::ServiceClient GetThrustClient1,GetThrustClient2,GetThrustClient3,GetThrustClient4;
ros::ServiceClient SetThrustClient1,SetThrustClient2,SetThrustClient3,SetThrustClient4;

ros::ServiceClient GetThrustClient[4]={GetThrustClient1,GetThrustClient2,GetThrustClient3,GetThrustClient4};
ros::ServiceClient SetThrustClient[4]={SetThrustClient1,SetThrustClient2,SetThrustClient3,SetThrustClient4};
 /********************pwmcallback**********************/
void pwmcallback(const asrobot::pwm::ConstPtr& msg)
{
    rotor.data = msg->rotor_speed1;
    Propeller1_pub.publish(rotor);
    rotor.data = msg->rotor_speed2;
    Propeller2_pub.publish(rotor);
    rotor.data = msg->rotor_speed3;
    Propeller3_pub.publish(rotor);
    rotor.data = msg->rotor_speed4;
    Propeller4_pub.publish(rotor);

    cout<<"speed1:"<<msg->rotor_speed1<<"  speed2:"<<msg->rotor_speed2<<"  speed3:"<<msg->rotor_speed3<<"  speed4:"<<msg->rotor_speed4<<endl;//hcy
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "thruster_control");
 
    ros::NodeHandle n;

    ros::Rate loop_rate = ros::Rate(1000);

    ros::Subscriber pwm_sub = n.subscribe("pwm_message", 1000, pwmcallback);
    Propeller1_pub = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("thrusters/0/input", 1000);
    Propeller2_pub = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("thrusters/1/input", 1000);
    Propeller3_pub = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("thrusters/2/input", 1000);
    Propeller4_pub = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("thrusters/3/input", 1000);

    GetThrustClient1 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::GetThrusterState>("thrusters/0/get_thruster_state");
    GetThrustClient2 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::GetThrusterState>("thrusters/1/get_thruster_state");
    GetThrustClient3 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::GetThrusterState>("thrusters/2/get_thruster_state");
    GetThrustClient4 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::GetThrusterState>("thrusters/3/get_thruster_state");

    SetThrustClient1 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::SetThrusterState>("thrusters/0/set_thruster_state");
    SetThrustClient2 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::SetThrusterState>("thrusters/1/set_thruster_state");
    SetThrustClient3 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::SetThrusterState>("thrusters/2/set_thruster_state");
    SetThrustClient4 = n.serviceClient<uuv_gazebo_ros_plugins_msgs::SetThrusterState>("thrusters/3/set_thruster_state");

    for(int id = 0; id < 4; id++)
    {
        if (!GetThrustClient[id].call(getsrv))
        {
            setsrv.request.on = true;
            SetThrustClient[id].call(setsrv);
        }
    }

    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();

    }

 return 0;  
}

