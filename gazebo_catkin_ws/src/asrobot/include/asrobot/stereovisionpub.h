#ifndef STEREOVISIONPUB_H
#define STEREOVISIONPUB_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>

#include <iostream>
#include <math.h>

#include "mysvm.h"//hcy
#include "stereocamera.h"


// tracking type
enum TrackingType{ DrawingBox, TrainingbySVM};
void mouseHandler(int event, int x, int y, int flags, void *param);

//class StereoVision_talker
//{
    
//public:
//    ~StereoVision_talker();

    void stop();
    StereoCamera sc;

    cv::Mat image2Show;
    // MySVM SvM;
//    static void savevisionvalue(const char* filename, const float& a_m , const float& b_m, const float& c_m,const float&xinimage_m,const float&yinimage_m)
//    {
//        //const double max_z = 16.0e4;
//            FILE* fp = fopen(filename, "aw+");
//           // printf("%d %d \n", mat.rows, mat.cols);
//           // for (int y = 0; y < 3; y++)
//           // {
//                for (int x = 0; x < 1; x++)
//               {
//               // cout<<"pointm[0] = "<<a<<endl;
//               // cout<<"pointm[1] = "<<b<<endl;
//                   // cout<<"pointm[2] = "<<c<<endl;

//            //cout<<"xinimage = "<<xinimage<<endl;
//            //cout<<"yinimage = "<<yinimage<<endl;
//                    //if (fabs(pointm[2] - max_z) < FLT_EPSILON || fabs(pointm[2] > max_z)) continue;
//                /*cout<<"pointm[0] = "<<a<<endl;
//                cout<<"pointm[1] = "<<b<<endl;
//                    cout<<"pointm[2] = "<<c<<endl;*/
//                    fprintf(fp, "%f %f %f %f %f\n", a_m, b_m, c_m,xinimage_m,yinimage_m);

//                }
//           // }
//            fclose(fp);
//    }



//private:

    bool mStopFlag;
    bool showImageFlag;

    TrackingType trackingType;
    //bool drawingBox = false;

    // Rect box;   // box for KCF
    Rect result;
//    cv::VideoCapture cap1,cap0;
    cv::VideoCapture cap0;
    cv::VideoWriter m_videoRec;
    int i=1;
    int j=1;
    cv::Mat imageL_m;
    cv::Mat imageR_m;
    cv::Mat imageL_m1;
    cv::Mat imageR_m2;
    cv::Mat camera_pic_L;//hcy
    cv::Mat camera_pic_R;//hcy

    //cv::Mat cameraMatrixLm, distCoeffLm, cameraMatrixRm, distCoeffRm;
    Mat cameraMatrixLm = (Mat_<double>(3, 3) << 381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0);
    Mat distCoeffLm = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    Mat cameraMatrixRm = (Mat_<double>(3, 3) << 381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0);
    Mat distCoeffRm = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    Mat Tm = (Mat_<double>(3, 1) << -62, 0, 0);//T平移向量

    Mat Rm = (Mat_<double>(3, 3) <<1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0) ;//R 旋转矩阵
    // Mat cameraMatrixLm = (Mat_<double>(3, 3) << 381.361150247924, 0.0, 320, 0.0, 381.361150247924, 240, 0.0, 0.0, 1.0);
    // Mat distCoeffLm = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // Mat cameraMatrixRm = (Mat_<double>(3, 3) << 381.361150247924, 0.0, 320, 0.0, 381.361150247924, 240, 0.0, 0.0, 1.0);
    // Mat distCoeffRm = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // Mat Tm = (Mat_<double>(3, 1) << -62, 0, 0);//T平移向量

    // Mat Rm = (Mat_<double>(3, 3) <<1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0) ;//R 旋转矩阵
    // Mat cameraMatrixLm = (Mat_<double>(3, 3) << 381.091150247924, 0.0, 320, 0.0, 381.091150247924, 240, 0.0, 0.0, 1.0);
    // Mat distCoeffLm = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // Mat cameraMatrixRm = (Mat_<double>(3, 3) << 381.091150247924, 0.0, 320, 0.0, 381.091150247924, 240, 0.0, 0.0, 1.0);
    // Mat distCoeffRm = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // Mat Tm = (Mat_<double>(3, 1) << -62, 0, 0);//T平移向量

    // Mat Rm = (Mat_<double>(3, 3) <<1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0) ;//R 旋转矩阵
    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool SILENT = false;
    bool LAB = false;
   // bool gotBB=false;
    cv::Mat frame;
    int manual=1;
    int training=0;

    //svm
    int descriptorDim;
    int supportVectorNum;
    Mat alphaMat;
    Mat supportVectorMat;
    Mat resultMat;
    Rect r;
    double rho;
    vector<float> myDetector;
    vector<Rect> found, foundFiltered;

    Point position;
    double a, b, c;      //object's position coordination(x,y,z)
    double clast=0;

    double xinimage, yinimage;
    double yaw_angle, pitch_angle;

   double judgedepthvalue(double ccurrent)
    {
        if (ccurrent==160000)
        {ccurrent = clast;}
        else
        {clast = ccurrent;}
        return ccurrent;

    }

//protected:


//};

#endif // STEREOVISIONTHREAD_H
