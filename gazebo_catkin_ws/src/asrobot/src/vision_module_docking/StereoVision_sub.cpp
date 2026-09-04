#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asrobot/camera_data.h"
void chatterCallback(const asrobot::camera_data::ConstPtr& cmd)
{
	ROS_INFO("I heard: [%f]", cmd->yaw_angle);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "StereoVision");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cameradata", 1000, chatterCallback);
	ros::spin();

	return 0;
}