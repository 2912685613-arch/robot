
// #include "stereocamera.h"
#include "../../include/asrobot/stereocamera.h"

StereoCamera::StereoCamera()
{

}

StereoCamera::StereoCamera(Mat cML, Mat dCL, Mat cMR, Mat, Mat dCR, Mat R, Mat T)
{
    this->cameraMatrixL = cML;
    this->cameraMatrixR = cMR;
    this->distCoeffL = dCL;
    this->distCoeffR = dCR;
    this->R = R;
    this->T = T;
}

StereoCamera::~StereoCamera()
{

}

void StereoCamera::loadParameters()
{
//    this->cameraMatrixL = cML;
//    this->cameraMatrixR = cMR;
//    this->distCoeffL = dCL;
//    this->distCoeffR = dCR;
//    this->R = R;
//    this->T = T;
}

void StereoCamera::loadParameters(string file)
{

}

void StereoCamera::drawBox(Mat image, Rect box, Scalar color, int thick)
{
    rectangle(image,Point(box.x,box.y),Point(box.x+box.width, box.y+box.height),color,thick);

}

void StereoCamera::imagePyrDown(Mat imageL, Mat imageR)
{
     cout<<imageL.size()<<endl;
    pyrDown(imageL, imageL_down, Size(imageL.cols/2,imageL.rows/2));
//    pyrDown(imageL_d, imageL_down, Size(imageL_d.cols/2,imageL_d.rows/2));
    pyrDown(imageR, imageR_down, Size(imageR.cols/2,imageR.rows/2));
//    pyrDown(imageR_d, imageR_down, Size(imageL_d.cols/2,imageL_d.rows/2));
//    imshow("imageL_down", imageL_down);
//    imshow("imageR_down", imageR_down);
}

void StereoCamera::undistortRectifyMap(Mat cameraMatrixL, Mat distCoeffL, Mat cameraMatrixR, Mat distCoeffR ,Mat T, Mat R)
{
    imageSize = imageL_down.size();
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
                  CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
}

//hy:新bm算法
void StereoCamera::stereoMatch(Mat &rgbImageL, Mat &rgbImageR)
{
    //hcy
    int SADWindowSize =  (2*blockSize+5);
    int numberOfDisparities = (numDisparities*16 + 16);
    
    Ptr<StereoBM> bm = StereoBM::create(16,9);

    cvtColor(rgbImageL, grayImageL, COLOR_BGR2GRAY);
    cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);
    //hy:重映射
    //      输出：rectifyImageL，rectifyImageR
    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    Mat disp, disp8;
    bm->compute(rectifyImageL, rectifyImageR, disp);

    disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities * 16.));
    reprojectImageTo3D(disp, xyz, Q, true);
    xyz = xyz * 16;
    imshow("disparity", disp8);	// show disparity
}

/*
void StereoCamera::stereoMatch(Mat &rgbImageL, Mat &rgbImageR)
{
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
    // cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
    // cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
    cvtColor(rgbImageL, grayImageL, COLOR_BGR2GRAY);//hcy
    cvtColor(rgbImageR, grayImageR, COLOR_BGR2GRAY);

    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    bm->setBlockSize(2*blockSize+5);
    bm->setROI1 (validROIL);
    bm->setROI2 (validROIR);
    bm->setPreFilterCap(31);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numDisparities*16 + 16);
    bm->setTextureThreshold(10); 
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100); 
    bm->setSpeckleRange(32); 
    bm->setDisp12MaxDiff(1);     
    Mat img1border, img2border;

    // copyMakeBorder(rectifyImageL, img1border, 0, 0, (numDisparities*16 + 16), 0, IPL_BORDER_REPLICATE);
    // copyMakeBorder(rectifyImageR, img2border, 0, 0, (numDisparities*16 + 16), 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(rectifyImageL, img1border, 0, 0, (numDisparities*16 + 16), 0, CV_HAL_BORDER_REPLICATE);//hcy
    copyMakeBorder(rectifyImageR, img2border, 0, 0, (numDisparities*16 + 16), 0, CV_HAL_BORDER_REPLICATE);

    Mat disp, disp8;
    bm->compute(rectifyImageL, rectifyImageR, disp);

    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));
    reprojectImageTo3D(disp, xyz, Q, true);
    xyz = xyz * 16;

   imshow("disparity", disp8);	// show disparity
}
*/

Mat StereoCamera::get3DPosition()
{

}

