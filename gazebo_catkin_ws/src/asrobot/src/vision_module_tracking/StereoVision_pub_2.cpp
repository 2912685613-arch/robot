
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asrobot/camera_data.h"

#include <sstream>
#include "../../include/asrobot/stereovisionpub.h"
// #include "../../include/asrobot/kcftracker.h"

//hcy
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>



//hcy
#include <stdio.h>
#include "opencv2/imgcodecs.hpp"


///hcy
// Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
// Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
///

//hcy
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cstring>

using namespace cv;
using namespace std;


// Rect box;
// bool drawingBox = false;
// bool gotBB ;
// KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
Ptr<Tracker> tracker = TrackerKCF::create();//hcy:create a tracker object
#define pi 3.1415926;
//MySVM SvM;

Rect2d roi;//hcy


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

    // cv::waitKey(3);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "StereoVision");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<asrobot::camera_data>("cameradata", 1000);
    ros::Rate loop_rate(10);

    //hcy
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub_L = it.subscribe("camera/left/image_raw", 1, imageCallback_left);
    image_transport::Subscriber sub_R = it.subscribe("camera/right/image_raw", 1, imageCallback_right);

    for(int i = 0; i < 10; i++)//hcy:等待订阅到图像
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if(camera_pic_L.empty())//hcy
    {
        cout << "Camera L Open Failed!" << endl;
    }
    if(camera_pic_R.empty())
    {
        cout << "Camera R Open Failed!" << endl;
    }

    // sc.imagePyrDown(imageL_m, imageR_m);//hy:降采样	
    // cv::Mat lm=sc.l();//hy:取出降采样后返回的图像
    // cv::Mat rm=sc.r();
    cv::Mat lm=camera_pic_L;//hcy
    cv::Mat rm=camera_pic_R;


    sc.undistortRectifyMap(cameraMatrixLm, distCoeffLm, cameraMatrixRm, distCoeffRm, Tm, Rm);//hy:映射变换计算

    // namedWindow("KCF", WINDOW_AUTOSIZE);//hcy:创建KCF窗口

    // setMouseCallback("KCF", mouseHandler, NULL);//hcy:鼠标回调操作

/*
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

        setMouseCallback("KCF", NULL, NULL);
    }

    else
    {
//        SvM.load("SVM_Model.xml");
//        descriptorDim = SvM.get_var_count();     // the dimension of the eigenvector
//        supportVectorNum = SvM.get_support_vector_count();     // the number of the supportvector
//        cout << "support vector num: " << supportVectorNum << endl;
//        alphaMat = Mat::zeros(1, supportVectorNum, CV_32FC1);
//        supportVectorMat = Mat::zeros(supportVectorNum, descriptorDim, CV_32FC1);
//        resultMat = Mat::zeros(1, descriptorDim, CV_32FC1);
        
        //将支持向量的数据复制到supportVectorMat矩阵中
//        for (int i = 0; i < supportVectorNum; i++)
//        {
//            const float * pSupportVectorData = SvM.get_support_vector(i);   //返回第i个支持向量的数据指针
//            for(int j = 0 ;j < descriptorDim; j++)
//            {
//                supportVectorMat.at<float>(i,j) = pSupportVectorData[j];
//            }
//        }
        
        //将alpha向量的数据复制到alphaMat中, 决策公式Y= wx+b
//        double *pAlphaData = SvM.get_alpha_data();   //返回SVM的决策函数中的alpha向量
//        for (int i = 0; i < supportVectorNum; i++)
//        {
//            alphaMat.at<float>(0, i) = pAlphaData[i];
//        }
        
//        //计算-(alphaMat * supportVectorMat),结果放到resultMat中
//        resultMat = -1 * alphaMat * supportVectorMat;  //alphaMat就是权重向量
        
//        cout <<"the number of the supportvector: " << descriptorDim << endl;
        
//        //得到最终的setSVMDetector(const vector<float>& detector)参数中可用的检测子
        
//        //将resultMat中的数据复制到数组myDetector中
//        for (int i = 0 ;i < descriptorDim; i++)
//        {
//            myDetector.push_back(resultMat.at<float>(0, i));
//        }
        
//        //最后添加偏移量rho，得到检测子
//        rho = SvM.get_rho_data();
//        myDetector.push_back(rho);
//        cout << "the dimension of the detector: " << myDetector.size() << endl;
//        HOGDescriptor myHOG (Size(128, 128), Size(16, 16), Size(8, 8), Size(8, 8), 9);
//        myHOG.setSVMDetector(myDetector);   //设置检测子
        
        //保存检测子
        // int minusNum = 0;
        // int posNum = 0;
        
//        ofstream foutDetector("HogDetectorForCarFace.txt");
//        for (int i = 0 ;i < myDetector.size(); i++)
//        {
//            foutDetector << myDetector[i] << " ";
//        }
//        foutDetector.close();
        
//        while(training)
//        {
//            Mat testImg = lm;
//            Size dsize = Size(320, 240);
//            Mat testImgNorm (dsize, CV_32S);
//            resize(testImg, testImgNorm, dsize);
            
//            cout << "MultiScale detect " << endl;
//            myHOG.detectMultiScale(testImgNorm, found, 0, Size(8,8), Size(0,0), 1.05, 2);
//            cout << "Detected Rect Num" << found.size() << endl;
            
//            //查看是否有嵌套的矩形框
//            for (int i = 0; i < found.size(); i++)
//            {
//                r = found[i];
//                int j = 0;
//                for (; j < found.size(); j++)
//                {
//                    if ( i != j && (r & found[j]) == r)
//                    {
//                        break;
//                    }
//                }
//                if(j == found.size())
//                    foundFiltered.push_back(r);
//            }
            
//            for( int i = 0; i < foundFiltered.size(); i++)   //画出矩形框
//            {
//                r = foundFiltered[i];
//                rectangle(testImgNorm, r.tl(), r.br(), Scalar(0,255,0), 1);
//            }
//            box = r;
//            if(!testImgNorm.empty())
//            {
//                imshow("test", testImgNorm);
//            }
//            // waitKey(1);
//            // cout << "foundFiltered" << foundFiltered.size() << endl;
//            if (foundFiltered.size()>0)
//            {
//                training = 0;
//            }
//            else
//            {
//                // cap1 >> imageL_m;
//                cap0 >> imageL_m;
//                cap0 >> imageR_m;
//                sc.imagePyrDown(imageL_m, imageR_m);
//            }
//        }
    }
*/
    // printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n", box.x, box.y, box.width, box.height);
// cv::Mat lm,rm;
// Rect2d roi;//hcy

    // std::string video = "/home/tortoise/code_opencv/KCF_test/src/test2.mp4";
    // VideoCapture cap(video);
    // cap >> lm;

    roi = selectROI("tracker", lm);//hcy:选择目标roi以GUI的形式

    if (roi.width == 0 || roi.height == 0)//hcy:quit if ROI was not selected
        return 0;

    printf("Initial Bounding Box = x:%f y:%f h:%f w:%f\n", roi.x, roi.y, roi.width, roi.height);//hcy

    // Ptr<Tracker> tracker = TrackerKCF::create();//hcy:create a tracker object

    // tracker.init(box, lm);//hy:用左眼追踪
    tracker->init(lm, roi);//hcy:initialize the tracker

    printf("Start the tracking process\n");//hcy:perform the tracking process
    


    while(1)
    {
        // for (;; ) {
        // cap >> lm;

        // sc.imagePyrDown(imageL_m, imageR_m);//hy:降采样
        // cv::Mat lm=sc.l();//hy:取出降采样后返回的图像
        // cv::Mat rm=sc.r();
        cv::Mat lm=camera_pic_L;//hcy
        cv::Mat rm=camera_pic_R;

        timespec start12, end12, diff12;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start12);
        
        if (lm.rows == 0 || lm.cols == 0)//hcy:stop the program if no more images
            break;

        // result = tracker.update(lm);
        tracker->update(lm, roi);//hcy:update the tracking result

        sc.stereoMatch(lm,rm);//hy:立体匹配
        // sc.stereoMatch_elas(lm,rm);//hy:立体匹配

        // rectangle(lm, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);//hy:绘制矩形框
        rectangle(lm, roi, Scalar(255, 0, 0), 2, 1);//hcy:draw the tracked object
       
        imshow("tracker", lm);
        if (waitKey(1) == 27)
            break;

        // position=Point(result.x + result.width/2, result.y + result.height/2);//hy:矩形框中心点
        position=Point(roi.x + roi.width/2, roi.y + roi.height/2);//hy:矩形框中心点

        cv::Mat xyz_m = sc.p();

        a=xyz_m.at<Vec3f>(position)[0];
        b=xyz_m.at<Vec3f>(position)[1];
        c=xyz_m.at<Vec3f>(position)[2];
        xinimage = result.x + result.width/2;
        yinimage = result.y + result.height/2;

        cout << position <<"in world coordinate is: " << xyz_m.at<Vec3f>(position) << endl;
        
        c = judgedepthvalue(c);


        //yaw_angle =((xinimage-80)/80)*100;
        //pitch_angle = atan(position_y/position_z);

        yaw_angle = atan(a/c);//hcy
        pitch_angle = atan(b/c);

        cout<<"yaw_angle value is "<<yaw_angle<<endl;
        cout<<"c value is "<<c<<endl;

    //    savevisionvalue("xyz13.xls", a, b, c,xinimage,yinimage);

/*
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
       printf("%f\n",1/resultTime12);
*/       

    //    lm.copyTo(image2Show);
    //    showImageFlag = true;
    //    showImage();
        
        
       ros::init(argc, argv, "StereoVision");
       ros::NodeHandle n;
       ros::Publisher chatter_pub = n.advertise<asrobot::camera_data>("cameradata", 1000);
       ros::Rate loop_rate(10);
       int count = 0;
       while (ros::ok())
       {
           asrobot::camera_data cmd;
        //    cmd.cam_yaw = yaw_angle;
        //    cmd.position_x = a;//hcy
        //    cmd.position_y = b;
        //    cmd.position_z = c;
            cmd.cam_yaw = -yaw_angle*57.3;
            cmd.position_x = c/1000;//hcy
            cmd.position_y = -a/1000;
            cmd.position_z = -b/1000;
            ROS_INFO("send the position of the target %f %f %f %f", cmd.cam_yaw, cmd.position_x, cmd.position_y, cmd.position_z);
            chatter_pub.publish(cmd);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
            if (count == 1)
            {
                break;
            }
       }
       
                  ros::spinOnce();
            loop_rate.sleep();
    }

    return 0;
}
/*
void mouseHandler(int event, int x, int y, int flags, void *param)
{
    switch (event)
    {
    case EVENT_MOUSEMOVE:
        if(drawingBox)
        {
            box.width = x - box.x;
            box.height = y - box.y;
        }
        break;
    case EVENT_LBUTTONDOWN:
        drawingBox = true;
        box = Rect(x,y,0,0);
        break;
    case EVENT_LBUTTONUP:
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
*/