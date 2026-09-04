#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asrobot/camera_data.h"

#include <sstream>
#include "../../include/asrobot/stereovisionpub.h"
#include "../../include/asrobot/kcftracker.h"
#include "../../include/asrobot/F_FourLedsSolvePnP.h"
// #include "../../include/asrobot/C_v4l2cap.h"
#include "../../include/asrobot/c_filter_group.h"

//hcy
// #include "../../include/asrobot/lime.h"   

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sys/types.h>

///hcy
// Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
// Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
///

/****************************  保存数据 打印输出 发布消息开关   ********************************/
#define SAVE_DATA_SWITCH 0
#define PRINT_SWITCH 1
#define SAVE_VIDEO_SWITCH 0

#define PUBLISH_SWITCH 1               // 发布开关
#define SUBSCRIBE_SWITCH 1             // 接收开关

/**********************************************************************************/
using namespace FilterGroup;

const int FOUR_LED_STATE = 1;
const int MARKER_STATE = 2;


bool save_data_flag = false;
bool save_video_flag = false;


// 引入外部变量
extern Mat origin_pic_clone; // 四灯连线图


// extern Mat camera_pic_L;//hcy





#if PUBLISH_SWITCH
// 发布器以及对应的消息
ros::Publisher chatter_pub;
asrobot::camera_data msg;//define a camera message
#endif



#if SUBSCRIBE_SWITCH
ros::Subscriber sub_cmd;                        // 订阅手动调试指令
#endif



void cmdcallback(const std_msgs::String::ConstPtr& msg)
{
    string cmdstring;
    vector<string> cmd;
    // getline(cin,cmdstring);
    cmdstring = msg->data;
    stringstream cmdstream(cmdstring);
    string buf;
    while(cmdstream >> buf) {
        cmd.push_back(buf);
    }

    if(cmd[0] == "xyz")//set pid parameter,for example:"pid 3 2 1"
    {
        save_data_flag = true;
        save_video_flag = true;
    }

    if(cmd[0] == "st")//set pid parameter,for example:"pid 3 2 1"
    {
        save_data_flag = false;
        save_video_flag = false;
    }

    if(cmd[0] == "docking")//hcy
    {
        save_data_flag = true;
        save_video_flag = true;
    }

    cmd.clear();
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */



//hy:读取左眼图像
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
    // imshow("origin_pic", camera_pic_L);

}


//hy:读取右眼图像
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
    // imshow("origin_pic2", camera_pic_R);

    cv::waitKey(3);
}



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "StereoVision_Docking");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    // ros::Rate loop_rate(5);


    /****************************  x, y, z, yaw滤波器   ********************************/
    C_KalmanFilter_WhiteNoise pos_x_filter;
    C_KalmanFilter_WhiteNoise pos_y_filter;
    C_KalmanFilter_WhiteNoise pos_z_filter;
    C_KalmanFilter_WhiteNoise yaw_filter;
    pos_x_filter.setNoiseRatio(10);
    pos_y_filter.setNoiseRatio(10);
    pos_z_filter.setNoiseRatio(10);
    yaw_filter.setNoiseRatio(10);
    /**********************************************************************************/

    /****************************  二维码 灯 定位状态切换标志位 ***************************/
    int switch_state = FOUR_LED_STATE;
    // int switch_state = MARKER_STATE;

    /**********************************************************************************/

    /****************************  摄像头内参 ***************************/
    Mat cam_mat = (Mat_<float>(3,3) << 381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0);
    Mat dist_mat = (Mat_<float>(5,1) <<0.0, 0.0, 0.0, 0.0, 0.0);
    /**********************************************************************************/


    /****************************  摄像头外参 ***************************/
    Mat rot_vec(3,1, CV_64FC1);
    Vec3f angles_vec;//hcy
    Mat trans_vec(3,1,CV_64FC1);
    /**********************************************************************************/


    /****************************  机器人需要知道的状态量 ***************************/
    double position_x, position_y, position_z = -80.0;
    double cam_pitch, cam_roll, cam_yaw;
    /**********************************************************************************/


    /****************************  四个灯的空间位置  ****************************/
    vector<Point3f> obj_points;
    /**********************************************************************************/

    /****************************  摄像头用的矩阵  ****************************/
    // 左右摄像头
    // Mat camera_pic_L;//hcy
    Mat camera_pic_L_RSZ;

    // V4L2用到的开关
    //V4L2cap vcap;//hcy
    /**********************************************************************************/


    /*****************************  图像增强算法  ****************************/   //hcy
    Mat	camera_pic_L_lime; 
    Mat imageL_0; 
    /**********************************************************************************/

#if PUBLISH_SWITCH
    chatter_pub = n.advertise<asrobot::camera_data>("cameradata", 1000);
#endif



#if SUBSCRIBE_SWITCH

    sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);

    //hcy
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub_L = it.subscribe("camera/left/image_raw", 1, imageCallback_left);
    image_transport::Subscriber sub_R = it.subscribe("camera/right/image_raw", 1, imageCallback_right);

#endif



    // 四个灯的空间位置
    // obj_points.push_back(Point3f(-28,38.2,59.4));// 单位是cm //hcy//new
    // obj_points.push_back(Point3f(28,38.2,59.4));
    // obj_points.push_back(Point3f(-28,-16.5,59.4));
    // obj_points.push_back(Point3f(28,-16.5,59.4));
    // obj_points.push_back(Point3f(-28,33.2,59.4));// 单位是cm //hcy//new
    // obj_points.push_back(Point3f(28,33.2,59.4));
    // obj_points.push_back(Point3f(-28,-21.5,59.4));
    // obj_points.push_back(Point3f(28,-21.5,59.4));
    obj_points.push_back(Point3f(-28,23.4,59.4));// 单位是cm //hcy//new
    obj_points.push_back(Point3f(28,23.4,59.4));
    obj_points.push_back(Point3f(-28,-31.3,59.4));
    obj_points.push_back(Point3f(28,-31.3,59.4));
    // 显示图像的窗口
    //namedWindow("origin_pic", CV_WINDOW_NORMAL);
    namedWindow("origin_pic", WINDOW_AUTOSIZE);//hcy

#if SAVE_DATA_SWITCH
    // 保存实时数据
    ofstream fout("/home/tortoise/gazebo_catkin_ws/src/gazebo_ros_robot/asrobot/src/vision_module_docking/data/attitude_and_position_raw3.txt");

    if(!fout)
    {
        printf("cant open this file\n");
    }
    else
    {
        // 写标题
        fout<<"forward_angle     "<<"position_x      "<<"position_y      "<<"position_z      "<<endl;
    }
#endif



#if SAVE_VIDEO_SWITCH
    // init
//    VideoWriter writer0("/home/ubuntu/video_raw.avi", CV_FOURCC('X','V','I','D'), 8, Size(320, 240) );
//    VideoWriter writer1("/home/ubuntu/video_line.avi", CV_FOURCC('X','V','I','D'), 8, Size(320, 240) );
    cv::VideoWriter writer0, writer1;
    writer0.open( "/home/ubuntu/video_raw.avi", CV_FOURCC('M','J','P','G'), 25.0, Size(640, 360),true);
    writer1.open( "/home/ubuntu/video_line.avi", CV_FOURCC('M','J','P','G'), 25.0, Size(640, 360),true);

    if(!writer0.isOpened())
    {
        cout << " writer0 open failed !!!! " <<endl;
        return -1;
    }

#endif

    // 等待程序稳定的倒计时
    int time_count = 30;
    Mat camera_pic_temp;
    Mat camera_line_temp;


    //hcy:如果使用640x480
    // VideoCapture cap1;
    // cap1.open(1);
    // //cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640); //hcy
    // //cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 
    // cap1.set(CAP_PROP_FRAME_WIDTH, 640); 
    // cap1.set(CAP_PROP_FRAME_HEIGHT, 480); 

    // if (!cap1.isOpened() )          
    // {
    //     cout << "cap1 open failed!!!!!!!!!!" << endl;
    //     return -1;
    // }
        // waitKey(5000);

    for(int i = 0; i < 10; i++)//hcy:等待订阅到图像
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {

        //vcap.V4L2open();//hy:如果使用1280x720
        //vcap.imageL.copyTo(camera_pic_L);
	

        //cap1 >> imageL_0; //hcy:如果使用640x480
        // cap1 >> camera_pic_L_lime; //hcy:如果使用640x480

        //vcap.imageL.copyTo(imageL_0);//hy:如果使用1280x720

        //rotate(imageL_0,camera_pic_L_lime,ROTATE_180);//hcy //将图像翻转180度
	    //flip(imageL_0, camera_pic_L_lime,-1);//hcy

        // feature::lime* l;//hzy //图像增强
        // l = new feature::lime(camera_pic_L_lime);
        // camera_pic_L = l->lime_enhance(camera_pic_L_lime);
    

// cout<<camera_pic_L.size()<<endl;
        // camera_pic_L=camera_pic_L_0;//hcy

        if(!camera_pic_L.empty())
        {
            imshow("origin_pic", camera_pic_L);
           
#if SAVE_VIDEO_SWITCH
    // save  camera_pic_L
            if(save_video_flag == true)
            {
                resize(camera_pic_L, camera_pic_temp, Size(640, 360), 0, 0, INTER_LINEAR);
                writer0.write(camera_pic_temp);

                cout << "====   saving raw video =====" << endl;

            }
            else
            {
                cout << "*****   exit raw video   *****" << endl;
                ;
            }
#endif

            if( switch_state == FOUR_LED_STATE )
            {
                if(FourLedLocation(camera_pic_L, obj_points, cam_mat, dist_mat, &rot_vec, &trans_vec) == 1)
                //if(FourLedLocation(camera_pic_L, obj_points, cam_mat, dist_mat, &angles_vec, &trans_vec) == 1)//hcy
                {
                    //提取原始值
                    cam_roll = rot_vec.at<double>(0,0);
                    cam_pitch = rot_vec.at<double>(1,0);
                    cam_yaw = rot_vec.at<double>(2,0);

                    position_x = trans_vec.at<double>(0,0);
                    position_y = trans_vec.at<double>(1,0);
                    position_z = trans_vec.at<double>(2,0);



                    // 处理一下
                    cam_pitch = cam_pitch * 57.3;//transfer to degrees
                    cam_roll = cam_roll * 57.3;
                    cam_yaw = cam_yaw * 57.3;
                    position_x = position_x/100.0;
                    position_y = position_y/100.0;
                    position_z = position_z/100.0;

                    // 给一段时间，让它稳定下来
                    if(time_count > 0)
                    {
                        time_count -= 1;
                        cout<< "**************** waiting for camera stablization ****************** " <<endl;

                        continue;
                    }
                    else
                    {

                    if((position_x <= 500) && (position_x >= -500) && (position_y <= 500) && (position_y >= -500) && (position_z <= 500) && (position_z >= -500))
                        {

                            // kalman white noise filter
                            position_x = pos_x_filter.CalFilterOutput(position_x);
                            position_y = pos_y_filter.CalFilterOutput(position_y);
                            position_z = pos_z_filter.CalFilterOutput(position_z);
                            cam_yaw = yaw_filter.CalFilterOutput(cam_yaw);

#if PUBLISH_SWITCH
                            // 统一发布 将变换后的数据发布出去
                            msg.cam_pitch = cam_pitch;
                            msg.cam_roll = cam_roll;
                            // msg.cam_yaw = -cam_yaw;
                            msg.cam_yaw = cam_yaw;//hcy

                            msg.position_x = -position_z;
                            msg.position_y = -position_x;
                            msg.position_z = position_y;
                            chatter_pub.publish(msg);

#endif

#if SAVE_DATA_SWITCH
                            if(save_data_flag == true)
                            {
                                // 统一保存数据
                                fout<<"   "<<cam_yaw <<"                   "<<position_x
                                   <<"                   "<<position_y<<"                   "<<position_z<<endl;
                                cout<<"write_successfully"<<endl;
                            }
                            else
                            {
                                ;
                            }

#endif

#if PRINT_SWITCH
                            // 统一输出
                            cout<< "rot_vec: " <<endl;
                            cout<< "cam_roll: " << cam_roll << endl;
                            cout<< "cam_pitch: "  << cam_pitch<< endl;
                            cout<< "cam_yaw: " << cam_yaw<< endl;

                            cout<< "trans_vec: " <<endl;
                            cout<< "pos_x: " << position_x << endl;
                            cout<< "pos_y: " << position_y << endl;
                            cout<< "pos_z: " << position_z << endl;

                            // 变换到机器人的坐标系
                            cout << "after transform:  " << endl;
                            // cout << "yaw: " << -cam_yaw << endl;
                            cout << "yaw: " << cam_yaw << endl;//hcy
                            cout << "pos_x: " << -position_z << endl;
                            cout << "pos_y: " << -position_x << endl;
                            cout << "pos_z: " << position_y << endl;

#endif

                            // 距离太近了 切换成marker识别
                            //if(position_z <= 0.6)// less than 60cm
                            if(position_z <= 1.6)// hcy
                            {
                                switch_state = MARKER_STATE;
                            }
                        }
                    }
                    
                }
                else
                {
                    cout<<"aquired points are not valid"<<endl;
                }
                cout<<" running"<<endl;


            }

            else if(switch_state == MARKER_STATE)
            {
                //camera_pic_L_RSZ = camera_pic_L(Rect(320,130,640,480 ));
                camera_pic_L_RSZ = camera_pic_L;//hcy:640x480
                if(MarkerLocation(camera_pic_L_RSZ, 0.202, &rot_vec, &trans_vec) == 0)
                {
                    

                    cout<<"detect marker successfully"<<endl;

                    cam_roll = rot_vec.at<float>(0,0);
                    cam_pitch = rot_vec.at<float>(1,0);
                    cam_yaw = rot_vec.at<float>(2,0);

                    position_x = trans_vec.at<float>(0,0);
                    position_y = trans_vec.at<float>(1,0);
                    position_z = trans_vec.at<float>(2,0);




                    //undertake these static
                    cam_pitch = cam_pitch * 57.3;//transfer to degrees
                    cam_roll = cam_roll * 57.3;
                    cam_yaw = cam_yaw * 57.3;
                    position_x =  position_x;
                    position_y = position_y;
                    position_z = position_z;


                    if((position_x <= 500) && (position_x >= -500) && (position_y <= 500) && (position_y >= -500) && (position_z <= 500) && (position_z >= -500))
                    {
#if PUBLISH_SWITCH
                            // 统一发布 将变换后的数据发布出去
                            msg.cam_pitch = cam_pitch;
                            msg.cam_roll = cam_roll;
                            // msg.cam_yaw = -cam_yaw;
                            msg.cam_yaw = cam_yaw;//hcy

                            msg.position_x = -position_z;
                            msg.position_y = -position_x;
                            // msg.position_z = position_y;
                            msg.position_z = position_y - 0.1;//hcy:将坐标系向上移动10cm，避免机器人卡住
                            chatter_pub.publish(msg);

#endif

#if SAVE_DATA_SWITCH
                            if(save_data_flag == true)
                            {
                                // 统一保存数据
                                fout<<"   "<<cam_yaw <<"                   "<<position_x
                                   <<"                   "<<position_y<<"                   "<<position_z<<endl;
                                cout<<"write_successfully"<<endl;
                            }
                            else
                            {
                                 ;
                            }
#endif

#if PRINT_SWITCH
                            // 统一输出
                            cout<< "rot_vec: " <<endl;
                            cout<< "cam_roll: " << cam_roll << endl;
                            cout<< "cam_pitch: "  << cam_pitch<< endl;
                            cout<< "cam_yaw: " << cam_yaw<< endl;

                            cout<< "trans_vec: " <<endl;
                            cout<< "pos_x: " << position_x << endl;
                            // cout<< "pos_y: " << position_y << endl;
                            cout<< "pos_y: " << position_y - 0.1<< endl;//hcy
                            cout<< "pos_z: " << position_z << endl;

                            // 变换到机器人的坐标系
                            cout << "after transform:  " << endl;
                            // cout << "yaw: " << -cam_yaw << endl;
                            cout << "yaw: " << cam_yaw << endl;//hcy
                            cout << "pos_x: " << -position_z << endl;
                            cout << "pos_y: " << -position_x << endl;
                            // cout << "pos_z: " << position_y << endl;
                            cout << "pos_z: " << position_y - 0.1<< endl;//hcy

#endif
         
                        // 距离太远了， 再切换成灯
                        // if(position_z > 0.6)// more than 0.6
                        if(position_z > 1.6)//hcy
                        {
                             switch_state = FOUR_LED_STATE;
                        }
                    }
              
                }
                else
                {
                    cout<< "detect marker failed, switching to FOUR_LED_STATE"<< endl;
                    switch_state = FOUR_LED_STATE;   // 二维码检测不到，那就切换成灯
                }

            }




#if SAVE_VIDEO_SWITCH
            // save  origin_pic_clone
            if( !origin_pic_clone.empty() )
            {
                if(save_video_flag == true)
                {
                    resize(origin_pic_clone, camera_line_temp, Size(640, 360), 0, 0, INTER_LINEAR);

                    writer1.write(camera_line_temp);
                    cout << "=====    saving line video    ======" << endl;
                }
                else
                {
                    cout << "*****    exit line video    *****" << endl;
                    ;

                }
            }
#endif



        }
        else
        {
            ROS_ERROR("video is empty\n");
            break;
        }

        ROS_INFO("the camera is running\n");

        waitKey(30);
        ros::spinOnce();
        loop_rate.sleep();
     
    }

#if SAVE_VIDEO_SWITCH
    writer0.release();
    writer1.release();
#endif

#if SAVE_DATA_SWITCH
    fout.close();
#endif

    destroyAllWindows();



    return 0;
}



