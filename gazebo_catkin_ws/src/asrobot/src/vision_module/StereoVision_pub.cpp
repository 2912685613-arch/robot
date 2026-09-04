// Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
// Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asrobot/camera_data.h"

#include <sstream>
#include "../../include/asrobot/stereovisionpub.h"
#include "../../include/asrobot/kcftracker.h"

using namespace cv;
using namespace std;


Rect box;
bool drawingBox = false;
bool gotBB ;
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
#define pi 3.1415926;


bool firstframeflag = true;

/*************find a special color, and get it's position*********************/
bool find_color(cv::Mat img, InputArray lowerb, InputArray upperb, float area_threshold, vector<AreaContour>& output)
{
    cv::Mat img_hsv, out, out_gauss, out_threshold;
    bool flag=false;
    if(img.empty())
    {
        return false;
    }
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    /*find specific color in image*/
    inRange(img_hsv, lowerb, upperb, out);
    GaussianBlur(out, out_gauss, Size(5,5),0,0);//reduce noise in image
    threshold(out_gauss, out_threshold, 127, 255, CV_THRESH_BINARY);

    /*find contours of a specific color*/
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(out_threshold, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    output.clear();
    /***caculate the property of every contours****/
    if(contours.size()>0)
    {
        flag=true;
        for( size_t i = 0; i < contours.size(); i++ )
        {
            AreaContour output_temp;
            Moments mu = moments(contours[i]);
            output_temp.contours = contours[i];
            output_temp.area = mu.m00;
            output_temp.center = Point2f((mu.m10 / (mu.m00 + 1e-5)), (mu.m01 / (mu.m00 + 1e-5)) );
            //cout << "area" << output_temp.area << endl;
            if(output_temp.area > area_threshold)
            {
                output.push_back(output_temp);
                //drawContours(img, contours, i, Scalar(0,255,0), 2, 2, hierarchy);
            }
        }
    }
    else
    {
        printf("could not find contourd!\n");
    }

    return flag;
}

/************************************/

void recongnize(cv::Mat img, RobotPosition& position)
{
    vector<AreaContour> orange, yellow, red, gold, redbright;

    /*find all orange area in image*/
    int LowH=11,   LowS=185,  LowV=38;//orange
    int HighH=20, HighS=255, HighV=255;
    find_color(img, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), 25, orange);

    /*find all yellow area in image*/
    LowH=29,   LowS=170,  LowV=50;//yellow
    HighH=36, HighS=255, HighV=255;
    find_color(img, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), 25, yellow);

    /*find all red area in image*/
    LowH=0,   LowS=230,  LowV=45;//red
    HighH=4, HighS=255, HighV=255;
    find_color(img, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), 25, red);

    /*find all gold area in image*/
    LowH=22,   LowS=90,  LowV=50;//gold
    HighH=25, HighS=225, HighV=255;
    find_color(img, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), 25, gold);

    /*find all redbright area in image*/
    LowH=5,   LowS=120,  LowV=66;//redbright
    HighH=8, HighS=248, HighV=194;
    find_color(img, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), 25, redbright);

    int robot1=orange.size();
    int robot2=red.size();
    int robot3=redbright.size();
    int robot81=yellow.size();
    int robot82=gold.size();


    if(robot1)
    {
        position.isrobot1 = true;
        position.robot1p.x = orange[0].center.x;
        position.robot1p.y = orange[0].center.y;
        circle( img, Point2f(position.robot1p.x, position.robot1p.y), 4, Scalar(0,255,0), -1 );


        printf("robot1_orange \n");

    }
    if(robot2)
    {
        position.isrobot2 = true;
        position.robot2p.x = red[0].center.x;
        position.robot2p.y = red[0].center.y;
        circle( img, Point2f(position.robot2p.x, position.robot2p.y), 4, Scalar(0,255,0), -1 );
//        printf("%.2f  %.2f\n", position.robot2p.x, position.robot2p.y);
        printf("robot2_red \n");

    }
    if(robot3)
    {
        position.isrobot3 = true;
        position.robot3p.x = redbright[0].center.x;
        position.robot3p.y = redbright[0].center.y;
        circle( img, Point2f(position.robot3p.x, position.robot3p.y), 4, Scalar(0,255,0), -1 );
        printf("robot3_redbright \n");

    }
    if(robot81)
    {
        position.isrobot81 = true;
        position.robot81p.x = yellow[0].center.x;
        position.robot81p.y = yellow[0].center.y;
        circle( img, Point2f(position.robot81p.x, position.robot81p.y), 4, Scalar(0,255,0), -1 );
        printf("robot81_yellow \n");

    }
    if(robot82)
    {
        position.isrobot82 = true;
        position.robot82p.x = gold[0].center.x;
        position.robot82p.y = gold[0].center.y;
        circle( img, Point2f(position.robot82p.x, position.robot82p.y), 4, Scalar(0,255,0), -1 );
        printf("robot82_gold \n");

    }
}

void StereoRecongnize(cv::Mat imageL, cv::Mat imageR, RobotPosition& position)
{
    if(imageL.empty() || imageR.empty())
    {
        return;
    }
    recongnize(imageL, position);
    if(position.isrobot1 || position.isrobot2 || position.isrobot3 || position.isrobot81 || position.isrobot82)
    {
        sc.stereoMatch(imageL,imageR);
    }
    if(position.isrobot1)
    {
        Point point[9];
        point[0] = Point(position.robot1p.x, position.robot1p.y);
        point[1] = Point(position.robot1p.x-2, position.robot1p.y-2);
        point[2] = Point(position.robot1p.x, position.robot1p.y-2);
        point[3] = Point(position.robot1p.x+2, position.robot1p.y-2);
        point[4] = Point(position.robot1p.x-2, position.robot1p.y);
        point[5] = Point(position.robot1p.x+2, position.robot1p.y);
        point[6] = Point(position.robot1p.x-2, position.robot1p.y+2);
        point[7] = Point(position.robot1p.x, position.robot1p.y+2);
        point[8] = Point(position.robot1p.x+2, position.robot1p.y+2);
        cv::Mat xyz_m=sc.p();
        double a=0, b=0, c=0;
        for(int i=0; i<9; i++)
        {
            a+=xyz_m.at<Vec3f>(point[i])[0];
            b+=xyz_m.at<Vec3f>(point[i])[1];
            c+=xyz_m.at<Vec3f>(point[i])[2];
        }
        position.robot1p.z = c/9/2;
    }
    if(position.isrobot2)
    {
        Point point[9];
        point[0] = Point(position.robot2p.x, position.robot2p.y);
        point[1] = Point(position.robot2p.x-2, position.robot2p.y-2);
        point[2] = Point(position.robot2p.x, position.robot2p.y-2);
        point[3] = Point(position.robot2p.x+2, position.robot2p.y-2);
        point[4] = Point(position.robot2p.x-2, position.robot2p.y);
        point[5] = Point(position.robot2p.x+2, position.robot2p.y);
        point[6] = Point(position.robot2p.x-2, position.robot2p.y+2);
        point[7] = Point(position.robot2p.x, position.robot2p.y+2);
        point[8] = Point(position.robot2p.x+2, position.robot2p.y+2);
        cv::Mat xyz_m=sc.p();
        double a=0, b=0, c=0;
        for(int i=0; i<9; i++)
        {
            a+=xyz_m.at<Vec3f>(point[i])[0];
            b+=xyz_m.at<Vec3f>(point[i])[1];
            c+=xyz_m.at<Vec3f>(point[i])[2];
        }
        position.robot2p.z = c/9/2;
//        printf("%.2f\n%.2f\n%.2f\n",a,b,c);
    }
    if(position.isrobot3)
    {
        Point point[9];
        point[0] = Point(position.robot3p.x, position.robot3p.y);
        point[1] = Point(position.robot3p.x-2, position.robot3p.y-2);
        point[2] = Point(position.robot3p.x, position.robot3p.y-2);
        point[3] = Point(position.robot3p.x+2, position.robot3p.y-2);
        point[4] = Point(position.robot3p.x-2, position.robot3p.y);
        point[5] = Point(position.robot3p.x+2, position.robot3p.y);
        point[6] = Point(position.robot3p.x-2, position.robot3p.y+2);
        point[7] = Point(position.robot3p.x, position.robot3p.y+2);
        point[8] = Point(position.robot3p.x+2, position.robot3p.y+2);
        cv::Mat xyz_m=sc.p();
        double a=0, b=0, c=0;
        for(int i=0; i<9; i++)
        {
            a+=xyz_m.at<Vec3f>(point[i])[0];
            b+=xyz_m.at<Vec3f>(point[i])[1];
            c+=xyz_m.at<Vec3f>(point[i])[2];
        }
        position.robot3p.z = c/9/2;
    }
    if(position.isrobot81)
    {
        Point point[9];
        point[0] = Point(position.robot81p.x, position.robot81p.y);
        point[1] = Point(position.robot81p.x-2, position.robot81p.y-2);
        point[2] = Point(position.robot81p.x, position.robot81p.y-2);
        point[3] = Point(position.robot81p.x+2, position.robot81p.y-2);
        point[4] = Point(position.robot81p.x-2, position.robot81p.y);
        point[5] = Point(position.robot81p.x+2, position.robot81p.y);
        point[6] = Point(position.robot81p.x-2, position.robot81p.y+2);
        point[7] = Point(position.robot81p.x, position.robot81p.y+2);
        point[8] = Point(position.robot81p.x+2, position.robot81p.y+2);
        cv::Mat xyz_m=sc.p();
        double a=0, b=0, c=0;
        for(int i=0; i<9; i++)
        {
            a+=xyz_m.at<Vec3f>(point[i])[0];
            b+=xyz_m.at<Vec3f>(point[i])[1];
            c+=xyz_m.at<Vec3f>(point[i])[2];
        }
        position.robot81p.z = c/9/2;
    }
    if(position.isrobot82)
    {
        Point point[9];
        point[0] = Point(position.robot82p.x, position.robot82p.y);
        point[1] = Point(position.robot82p.x-2, position.robot82p.y-2);
        point[2] = Point(position.robot82p.x, position.robot82p.y-2);
        point[3] = Point(position.robot82p.x+2, position.robot82p.y-2);
        point[4] = Point(position.robot82p.x-2, position.robot82p.y);
        point[5] = Point(position.robot82p.x+2, position.robot82p.y);
        point[6] = Point(position.robot82p.x-2, position.robot82p.y+2);
        point[7] = Point(position.robot82p.x, position.robot82p.y+2);
        point[8] = Point(position.robot82p.x+2, position.robot82p.y+2);
        cv::Mat xyz_m=sc.p();
        double a=0, b=0, c=0;
        for(int i=0; i<9; i++)
        {
            a+=xyz_m.at<Vec3f>(point[i])[0];
            b+=xyz_m.at<Vec3f>(point[i])[1];
            c+=xyz_m.at<Vec3f>(point[i])[2];
        }
        position.robot82p.z = c/9/2;
    }
}


void showImage()
{
    if(showImageFlag)
    {
        if(!image2Show.empty())
        {
            imshow("result",image2Show);
            waitKey(30);
        }
        // m_videoRec << image2Show;
    }
}


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
    imageL_m = cv_ptr->image;

}

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
    imageR_m = cv_ptr->image;

    cv::waitKey(3);
}

// void savevideo()
// {
//     m_videoRec.open("video.avi", CV_FOURCC('M','J','P','G'), 14 , cv::Size(image2Show.cols,image2Show.rows));
//     if(!m_videoRec.isOpened())
//     {
//         cout << "fail to open video writer" << endl;
//     }
//     assert(m_videoRec.isOpened());
// }

// void stopsavevideo()
// {
//     cout << "stop save" << endl;
//     m_videoRec.release();
// }
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "StereoVision");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<asrobot::camera_data>("cameradata", 1000);
    ros::Rate loop_rate(30);
    image_transport::ImageTransport it(n);

    image_transport::Subscriber sub_L = it.subscribe("camera/left/image_raw", 1, imageCallback_left);
    image_transport::Subscriber sub_R = it.subscribe("camera/right/image_raw", 1, imageCallback_right);

    for(int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (firstframeflag)
    {
        if(imageL_m.empty())
        {
            cout << "Camera L Open Failed!" << endl;
        }
        if(imageR_m.empty())
        {
            cout << "Camera R Open Failed!" << endl;
        }

        // sc.imagePyrDown(imageL_m, imageR_m);
        // cv::Mat lm=sc.l();
        // cv::Mat rm=sc.r();
        cv::Mat lm=imageL_m;//hcy
        cv::Mat rm=imageR_m;

    sc.undistortRectifyMap(cameraMatrixLm, distCoeffLm, cameraMatrixRm, distCoeffRm, Tm, Rm);
   
//    cvNamedWindow("KCF", CV_WINDOW_AUTOSIZE);
   namedWindow("KCF", WINDOW_AUTOSIZE);//hcy
//    cvSetMouseCallback("KCF", mouseHandler, NULL);
   setMouseCallback("KCF", mouseHandler, NULL);//hcy

   if(manual)	//mouse operation
   {
       while (!gotBB)
       {
           sc.drawBox(lm, box, (0, 0, 255), 2);
           if (!lm.empty())
           {
               imshow("KCF", lm);
               waitKey(30);
           }
       }
    //    cvSetMouseCallback("KCF", NULL, NULL);
       setMouseCallback("KCF", NULL, NULL);
        
  }

   printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n", box.x, box.y, box.width, box.height);
   tracker.init(box, lm);
   
    firstframeflag = false;
    }

    while(ros::ok())
    {
        asrobot::camera_data cmd;
        ros::spinOnce();
        loop_rate.sleep();

        if (imageL_m.empty())
        {
            cout << "camera_left failed\n";
        }

        if(imageR_m.empty())
        {
            cout << "camera_right failed\n";
        } 
        
        // sc.imagePyrDown(imageL_m, imageR_m);
        // cv::Mat lm=sc.l();
        // cv::Mat rm=sc.r();
        cv::Mat lm=imageL_m;//hcy
        cv::Mat rm=imageR_m;

        timespec start12, end12, diff12;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start12);

        RobotPosition position;
        //StereoRecongnize(imageL_m, imageR_m, position);
        StereoRecongnize(lm, rm, position);//x y dots z mm

        if(position.isrobot1)
        {
                cout << "robot1:" << position.robot1p<<endl;
        }
        else
        {
                cout << "can not find robot1"<<endl;
        }
        if(position.isrobot2)
        {
                cout << "robot2:" << position.robot2p<<endl;
        }
        else
        {
                cout << "can not find robot2"<<endl;
        }
        if(position.isrobot3)
        {
                cout << "robot3:" << position.robot3p<<endl;
        }
        else
        {
                cout << "can not find robot3"<<endl<<endl;
        }
        if(position.isrobot81)
        {
                cout << "robot81:" << position.robot81p<<endl;
        }
        else
        {
                cout << "can not find robot81"<<endl;
        }
        if(position.isrobot82)
        {
                cout << "robot82:" << position.robot82p<<endl;
        }
        else
        {
                cout << "can not find robot82"<<endl;
        }
//        result = tracker.update(lm);
//        sc.stereoMatch(lm,rm);
//        rectangle(lm, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);
//        position=Point(result.x + result.width/2, result.y + result.height/2);
//        cv::Mat xyz_m=sc.p();
        
//        a=xyz_m.at<Vec3f>(position)[0];
//        b=xyz_m.at<Vec3f>(position)[1];
//        c=xyz_m.at<Vec3f>(position)[2];
//        xinimage = result.x + result.width/2;
//        yinimage = result.y + result.height/2;
        
        // cout << position <<"in world coordinate is: " << xyz_m.at<Vec3f>(position) << endl;
        
        c = judgedepthvalue(c);
        
        // yaw_angle =((xinimage-80)/80)*100;
        yaw_angle = atan(a/c);
        pitch_angle = atan(b/c);
        // cout<<"yaw_angle value is "<<yaw_angle<<endl;
        // cout<<"c value is "<<c<<endl;
        // savevisionvalue("xyz13.xls", a, b, c,xinimage,yinimage);

        //calculate the Fps:
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end12);
        diff12.tv_sec = ( end12.tv_sec - start12.tv_sec );
        diff12.tv_nsec = ( end12.tv_nsec - start12.tv_nsec );
        if (diff12.tv_nsec < 0)
        {
            diff12.tv_sec--;
            diff12.tv_nsec += 1000000000;
        }
        int usec12 = diff12.tv_nsec + diff12.tv_sec * 1000000000;
        double resultTime12 = (double)usec12 / 1000000000;
        //printf("%f\n",1/resultTime12);
        
        lm.copyTo(image2Show);
        showImageFlag = true;
        showImage();
        //imshow ("right", rm);
        //waitKey(3);
        
        cmd.obj_x = a;
        cmd.obj_y = b;
        cmd.obj_z = c;
        //ROS_INFO("send the position of the target %f", cmd.obj_x);
        chatter_pub.publish(cmd);

    }
    return 0;
}

void mouseHandler(int event, int x, int y, int flags, void *param)
{
    switch (event)
    {
    // case CV_EVENT_MOUSEMOVE:
    case EVENT_MOUSEMOVE://hcy
        if(drawingBox)
        {
            box.width = x - box.x;
            box.height = y - box.y;
        }
        break;
    // case CV_EVENT_LBUTTONDOWN:
    case EVENT_LBUTTONDOWN://hcy
        drawingBox = true;
        box = Rect(x,y,0,0);
        break;
    // case CV_EVENT_LBUTTONUP:
    case EVENT_LBUTTONUP://hcy
        drawingBox = false;
        if(box.width < 0)
        {
            box.x += box.width;
            box.width *= -1;
        }
        if(box.height < 0)
        {
            box.y += box.height;
            box.height *= -1;
        }
        gotBB=true;
        break;     
    }
}


