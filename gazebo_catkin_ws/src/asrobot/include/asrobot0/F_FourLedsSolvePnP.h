#ifndef __FOURLEDSOLVEPNP__H
#define __FOURLEDSOLVEPNP__H

#include "stdio.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "highgui.h"
#include <iostream>
#include <string>

#include <aruco.h>
#include <cvdrawingutils.h>


#include <opencv2/imgcodecs.hpp>//hcy



using namespace std;
using namespace cv;
using namespace aruco;



static Vec3f rotationMatrixToEulerAngles(Mat &R);//hcy
Point FindCenterPoint(vector<vector<Point>> contourss, int index);
vector<Point> FindFourPoints(vector<Point> many_points);
int FourLedLocation(Mat origin_pic,vector<Point3f> obj_pnts, Mat cam_mat, Mat dist_mat, Mat *posture, Mat *position);
int MarkerLocation(Mat input_image, float marker_size, Mat* rot_vec, Mat* trans_vec);

#endif
