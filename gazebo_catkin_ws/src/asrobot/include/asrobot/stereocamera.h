#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>


#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

//cv::StereoBM bm;

class AreaContour
{
public:
	vector<Point> contours;
	Point2f center;
	float area;
};

class RobotPosition
{
public:
        Point3f robot1p, robot2p, robot3p, robot81p, robot82p;
        bool isrobot1=false, isrobot2=false, isrobot3=false, isrobot81=false, isrobot82=false;
};

class StereoCamera
{

public:
    StereoCamera();
    StereoCamera(Mat cML,Mat dCL,Mat cMR,Mat,Mat dCR,Mat R,Mat T);
    ~StereoCamera();

    void loadParameters();
    void loadParameters(string file);
    void drawBox(Mat image, Rect box, Scalar color, int thick);
    void imagePyrDown(Mat imageL, Mat imageR);
	
    //void undistortRectifyMap(Mat &mapLx, Mat &mapLy, Mat &mapRx,Mat &mapRy);
    void undistortRectifyMap(Mat cameraMatrixL, Mat distCoeffL, Mat cameraMatrixR, Mat distCoeffR, Mat T, Mat R);
    void stereoMatch(Mat &rgbImageL,Mat &rgbImageR);
    cv::Mat get3DPosition();
    cv::Mat l() {
        return imageL_down;
    }
    cv::Mat r() {
        return imageR_down;
    }
    cv::Mat p() {
        return xyz;
    }

private:
    //cv::Mat imageL, imageR;
    cv::Mat imageL_down, imageR_down,imageL_d,imageR_d;
    cv::Mat rgbImageL, rgbImageR, grayImageL, grayImageR;
    cv::Mat rectifyImageL, rectifyImageR;
    cv::Size imageSize;

    // Define region of interest
    cv::Rect validROIL, validROIR;

    cv::Mat cameraMatrixL, cameraMatrixR;
    cv::Mat distCoeffL, distCoeffR;
    cv::Mat R, T;

    cv::Mat mapLx, mapLy, mapRx, mapRy;
    cv::Mat Rl, Rr, Pl, Pr, Q;
    cv::Mat xyz;

    int imageScale;

    // int blockSize = 0, uniquenessRatio = 15, numDisparities = 0;
    int blockSize = 8, uniquenessRatio = 15, numDisparities = 3;//hcy



};

#endif // STEREOCAMERA_H
