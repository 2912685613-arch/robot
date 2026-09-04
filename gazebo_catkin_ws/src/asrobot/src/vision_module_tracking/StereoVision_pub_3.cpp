
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asrobot/camera_data.h"

#include <sstream>
#include "../../include/asrobot/stereovisionpub.h"
#include "../../include/asrobot/kcftracker.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>


#include <iostream>
#include <stdio.h>
#include "opencv2/imgcodecs.hpp"

#include<string>
#include<opencv2/opencv.hpp>

// Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
// Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


using namespace cv;
using namespace std;




//读取左眼图像
void imageCallback_left(const sensor_msgs::ImageConstPtr& gazebo_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(gazebo_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    camera_pic_L = cv_ptr->image;
    // imshow("origin_pic_L", camera_pic_L);
}


//读取右眼图像
void imageCallback_right(const sensor_msgs::ImageConstPtr& gazebo_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(gazebo_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    camera_pic_R = cv_ptr->image;
    // imshow("origin_pic_R", camera_pic_R);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "StereoVision");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<asrobot::camera_data>("cameradata", 1000);
    ros::Rate loop_rate(10);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub_L = it.subscribe("camera/left/image_raw", 1, imageCallback_left);
    image_transport::Subscriber sub_R = it.subscribe("camera/right/image_raw", 1, imageCallback_right);

    for(int i = 0; i < 10; i++)//hcy:等待订阅到图像
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if(camera_pic_L.empty())
    {
        cout << "Camera L Open Failed!" << endl;
    }
    if(camera_pic_R.empty())
    {
        cout << "Camera R Open Failed!" << endl;
    }

    Mat frame_L, frame_R;
    char key;
    char image_left[200];
    char image_right[200];
    int count1 = 0;
    int count2 = 0;

    while (ros::ok())
    {
        key = waitKey(50);

        imshow("origin_pic_L", camera_pic_L);
        imshow("origin_pic_R", camera_pic_R);

        frame_L = camera_pic_L;
        frame_R = camera_pic_R;

        if (key == 27) //ESC退出
            break;
        if (key == 32) //空格拍照
        {

            sprintf(image_left, "/home/tortoise/gazebo_catkin_ws/src/gazebo_ros_robot/asrobot/src/vision_module_tracking/L%d.jpg", ++count1);
            imwrite(image_left, frame_L);
            imshow("left", frame_L);

            sprintf(image_right, "/home/tortoise/gazebo_catkin_ws/src/gazebo_ros_robot/asrobot/src/vision_module_tracking/R%d.jpg", ++count2);
            imwrite(image_right, frame_R);
            imwrite("R1.jpg", frame_R);
            imshow("right", frame_R);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



