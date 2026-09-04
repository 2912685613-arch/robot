# ifndef __C_DOCKING_H
# define __C_DOCKING_H

#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

using namespace std;
using namespace cv;
using namespace Eigen;


//
namespace docking {

const float docking_yaw_limit1 = 120;
const float docking_yaw_limit2 = 170;

// 最终无直线缓冲段
class C_Docking
{
public:
    float robot_yaw;        // 机器人航向角
    float k_0;              // 原始直线斜率
    float k_1;              // 新直线斜率
    float robot_x;          // 机器人全局坐标x
    float robot_y;          // 机器人全局坐标y
    float robot_z;          // 机器人全局坐标z
    float circle_center_x;  // 运动的圆心x
    float circle_center_y;  // 运动的圆心y
    float circle_radius;    // 圆的半径
    float medium_x;         // x方向的缓冲距离，用于最后的对接
    float start_yaw;        // 圆形轨迹起始角
    float theta;            // 参数方程中间变量

    int num_point;          // 路径点数

    float pi;

    MatrixXf exp_path_3d;   // 期望的三维路径

public:
    C_Docking();
    float myabs(float _x);
    void setMediumX(float _medium_x);
    void setPathPointsNum(int _num);
    void planCirclePathLeft(float _robot_x, float _robot_y, float _robot_z);
    void planCirclePathRight(float _robot_x, float _robot_y, float _robot_z);
    void planLinePath(float _robot_x, float _robot_y, float _robot_z);

    MatrixXf outputExpPath(void);


};


// 最终有直线缓冲段//hcy
class C_Docking_v2
{
public:
    float robot_yaw;        // 机器人航向角
    float k_0;              // 原始直线斜率
    float k_1;              // 新直线斜率
    float robot_x;          // 机器人全局坐标x
    float robot_y;          // 机器人全局坐标y
    float robot_z;          // 机器人全局坐标z
    float circle_center_x;  // 运动的圆心x
    float circle_center_y;  // 运动的圆心y
    float circle_radius;    // 圆的半径
    float medium_x;         // x方向的缓冲距离，用于最后的对接,
                            // 可认为是直线轨迹的起始位置
    float end_x;            // 直线轨迹沿着x方向的最终位置
    float start_yaw;        // 圆形轨迹起始角
    float theta;            // 参数方程中间变量

    int heli_num_point;     // 螺旋轨迹点数
    int line_num_point;     // 直线轨迹点数
    int num_point;          // 总的路径点数

    float pi;

    MatrixXf heli_exp_path_3d;  // 期望的螺旋轨迹
    MatrixXf line_exp_path_3d;  // 期望的直线轨迹
    MatrixXf exp_path_3d;   // 期望的三维轨迹


public:
    C_Docking_v2();
    float myabs(float _x);
    void setMediumXandEndX(float _medium_x, float _end_x);
    void setPathPointsNum(int _num1, int _num2); // 螺旋轨迹点数,  直线轨迹点数
    void planCirclePathLeft(float _robot_x, float _robot_y, float _robot_z);
    void planCirclePathRight(float _robot_x, float _robot_y, float _robot_z);
    void planLinePath(float _robot_x, float _robot_y, float _robot_z);
    void planEndLinePath();
    MatrixXf outputExpPath(void);


};

} // namespace 括号


#endif
