#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mancontrol");

  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("cmd_message", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;


    cout << "please input cmd!"<<endl;

    getline(cin,msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

