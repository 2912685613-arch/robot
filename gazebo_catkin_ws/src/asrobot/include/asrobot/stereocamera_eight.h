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


class ASRobot
{
public:
    int robotID;
    bool isrobot = false;
    Point2i pos_2D;
    Point3f pos_3D;
    AreaContour imgContour;
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
    void inputImage(Mat imageL, Mat imageR);
    void imagePyrDown(Mat imageL, Mat imageR);

    //void undistortRectifyMap(Mat &mapLx, Mat &mapLy, Mat &mapRx,Mat &mapRy);
    void undistortRectifyMap(Mat cameraMatrixL, Mat distCoeffL, Mat cameraMatrixR, Mat distCoeffR, Mat T, Mat R);
    void stereoMatch(Mat &rgbImageL, Mat &rgbImageR);
    cv::Mat get3DPosition();
    cv::Mat l() {
        return imageL;
    }
    cv::Mat r() {
        return imageR;
    }
    cv::Mat l_down() {
        return imageL_down;
    }
    cv::Mat r_down() {
        return imageR_down;
    }
    cv::Mat p() {
        return xyz;
    }

    cv::Mat disparityImage;

private:
    cv::Mat imageL, imageR;
    cv::Mat imageL_down, imageR_down,imageL_d,imageR_d;
    cv::Mat rgbImageL, rgbImageR, grayImageL, grayImageR;
    cv::Mat rectifyImageL, rectifyImageR;

    cv::Size imageSize;
    int imageScale;

    // Define region of interest
    cv::Rect validROIL, validROIR;

    cv::Mat cameraMatrixL, cameraMatrixR;
    cv::Mat distCoeffL, distCoeffR;
    cv::Mat R, T;

    cv::Mat mapLx, mapLy, mapRx, mapRy;
    cv::Mat Rl, Rr, Pl, Pr, Q;
    cv::Mat xyz;

    int blockSize, uniquenessRatio, numDisparities;

};

#endif // STEREOCAMERA_H
