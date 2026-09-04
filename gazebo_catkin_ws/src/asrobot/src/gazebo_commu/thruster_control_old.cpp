#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include "asrobot/pwm.h"
#include "math.h"

#include <iostream>
#include <stdio.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

using namespace Eigen;


Eigen::Vector3f forceInRobot, forceInWorld;
double torque_z;
double attitude_roll;
double attitude_yaw;
double attitude_pitch;

gazebo_msgs::ApplyBodyWrench objstate;

ros::ServiceClient client;
Eigen::Matrix3f rotMatrix2Robot;
 /********************pwmcallback**********************/
void pwmcallback(const asrobot::pwm::ConstPtr& msg)
{
    forceInRobot(0) = msg->force_x;//scale factor k
    forceInRobot(1) = msg->force_y;
    forceInRobot(2) = msg->force_z;

   std::cout<<"robot:"<<forceInRobot(0)<<std::endl;
    torque_z = msg->torque_z;
    attitude_yaw = msg->attitude_yaw;
    attitude_roll = msg->attitude_roll;
    attitude_pitch = msg->attitude_pitch;
   std::cout<<"robot_yaw:"<<attitude_yaw<<std::endl;
    Eigen::Vector3f eulerAngle1(M_PI * attitude_yaw / 180, M_PI * attitude_pitch/180, M_PI * attitude_roll / 180);
//    ROS_INFO("roll=%f, yaw=%f, pitch=%f", quadState_.roll, quadState_.yaw, quadState_.pitch);
    Eigen::AngleAxisf rollAngle1(AngleAxisf(eulerAngle1[2], Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle1(AngleAxisf(eulerAngle1[1], Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle1(AngleAxisf(eulerAngle1[0], Vector3f::UnitZ()));
    rotMatrix2Robot = yawAngle1 * pitchAngle1 * rollAngle1;
//    cout << "rotMat2VitualPlane" << rotMat2VitualPlane << endl;

    forceInWorld = rotMatrix2Robot * forceInRobot;


 /****************gazebo_robot xx link*****************/
/****************control the motor based on ROS client-servicer*****************/

    objstate.request.body_name = "gazebo_robot1::gazebo_robot1/base_link";
//    objstate.request.wrench.force.x = forceInRobot(0);//forceInWorld(0); //force_sim*cos(alpha/180*3.14159);
//    objstate.request.wrench.force.y = forceInRobot(1);//forceInWorld(1); //force_sim*sin(alpha/180*3.14159);
//    objstate.request.wrench.force.z = forceInRobot(2);//forceInWorld(2); //force_sim_z;
    objstate.request.wrench.force.x = forceInWorld(0); //force_sim*cos(alpha/180*3.14159);
    objstate.request.wrench.force.y = forceInWorld(1); //force_sim*sin(alpha/180*3.14159);
    objstate.request.wrench.force.z = forceInWorld(2); //force_sim_z;
    objstate.request.wrench.torque.z = torque_z;
    std::cout<<"world:"<<forceInWorld(0)<<std::endl;
    objstate.request.duration = ros::Duration(-1);
    client.call(objstate);
 //   printf("force_x:%.2f\tforce_z:%.2f\ntorque:%.2f\n", force_sim,force_sim_z,torque_sim);
    printf("angle:%.2f\n",attitude_yaw);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "thruster_control_old");
 
    ros::NodeHandle n;

    ros::Rate loop_rate = ros::Rate(1000);

    ros::Subscriber pwm_sub = n.subscribe("pwm_message", 1000, pwmcallback);
    client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

 



    ros::spin();

 return 0;  
}

