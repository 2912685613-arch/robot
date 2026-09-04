#include "../../include/asrobot/c_docking.h"

#define PRINT_SWITCH 1

namespace docking{
C_Docking::C_Docking()
{
    robot_yaw = 0;
    k_0 = 0;
    k_1 = 0;
    robot_x = 0;
    robot_y = 0;
    robot_z = 0;

    circle_center_x = 0;
    circle_center_y = 0;
    circle_radius = 0;

    // medium_x = 0.3; // 单位是m
    medium_x = 0.3; // 单位是m

    // num_point = 3;  // 默认三个点
    num_point = 5;  // hcy

    theta = 0;

    pi = 3.1415926535;

    exp_path_3d = MatrixXf(3, num_point);

#if PRINT_SWITCH
    cout <<  "the medium_x is:  " << medium_x <<endl;
    cout <<  "the number of expect points is: " << num_point << endl;
#endif

}


float C_Docking::myabs(float _x)
{
    if(_x > 0.000001)
        _x = _x;
    else if(_x < -0.000001)
        _x = -_x;
    else
        _x = 0;
    return _x;
}

//hy:尚未使用
void C_Docking::setMediumX(float _medium_x)
{
    medium_x = _medium_x;
}

//hy:尚未使用
void C_Docking::setPathPointsNum(int _num)
{
    num_point = _num;
}


void C_Docking::planCirclePathLeft(float _robot_x, float _robot_y, float _robot_z)
{
    robot_x = _robot_x;
    robot_y = _robot_y;
    robot_z = _robot_z;
    start_yaw = -(180 - atan2(robot_y, robot_x + medium_x)/pi*180 )*2; //

    //if(start_yaw >= 0)
     //   start_yaw = -start_yaw; // 向左转弯

#if PRINT_SWITCH
    cout << "start_yaw: " << start_yaw <<endl;
#endif

    // 直线斜率
    k_0 = tan(start_yaw/180*pi);
    k_1 = -(1/k_0);


    // 圆心和半径
    circle_center_x = -medium_x;
    circle_center_y = k_1*( -medium_x - robot_x ) + robot_y;
    circle_radius = myabs( circle_center_y );

#if PRINT_SWITCH
    cout << "circle_center_x: " << circle_center_x <<endl;
    cout << "circle_center_y: " << circle_center_y <<endl;
    cout << "circle_radius: " << circle_radius <<endl;
    cout << "myabs(5): " << myabs(5) <<endl;
    cout << "myabs(-5): " << myabs(-5) <<endl;
#endif


    // 用参数方程截取点 从近到远
    for(int i = 0; i < num_point ; i++)
    {
        theta = -90 + start_yaw + myabs(start_yaw)/num_point +
                ( myabs(start_yaw)/num_point*i);
#if PRINT_SWITCH
    cout << "theta: " << theta <<endl;
#endif
        // 转换成弧度，再写进去
        exp_path_3d(0, i) = circle_center_x + circle_radius*cos(theta/180*pi);
        exp_path_3d(1, i) = circle_center_y + circle_radius*sin(theta/180*pi);
    }

    // 截取z
    for(int i = 0; i<num_point; i++)
    {
        // 存进去
        exp_path_3d(2, i) = robot_z - robot_z/num_point - robot_z/num_point*i;
    }

#if PRINT_SWITCH
     cout << "exp_path_3d: " << endl << exp_path_3d <<endl;
#endif
}


void C_Docking::planCirclePathRight(float _robot_x, float _robot_y, float _robot_z)
{
    robot_x = _robot_x;
    robot_y = _robot_y;
    robot_z = _robot_z;
    start_yaw = myabs(-180 - atan2(robot_y, robot_x + medium_x)/pi*180 )*2; //

    //if(start_yaw >= 0)
     //   start_yaw = -start_yaw; // 向左转弯

#if PRINT_SWITCH
    cout << "start_yaw: " << start_yaw <<endl;
#endif

    // 直线斜率
    k_0 = tan(start_yaw/180*pi);
    k_1 = -(1/k_0);


    // 圆心和半径
    circle_center_x = -medium_x;
    circle_center_y = k_1*( -medium_x - robot_x ) + robot_y;
    circle_radius = myabs( circle_center_y );

#if PRINT_SWITCH
    cout << "circle_center_x: " << circle_center_x <<endl;
    cout << "circle_center_y: " << circle_center_y <<endl;
    cout << "circle_radius: " << circle_radius <<endl;
    cout << "myabs(5): " << myabs(5) <<endl;
    cout << "myabs(-5): " << myabs(-5) <<endl;
#endif


    // 用参数方程截取点 从近到远
    for(int i = 0; i < num_point ; i++)
    {
        theta = 90 + start_yaw - myabs(start_yaw)/num_point -
                ( myabs(start_yaw)/num_point*i);
#if PRINT_SWITCH
    cout << "theta: " << theta <<endl;
#endif
        // 转换成弧度，再写进去
        exp_path_3d(0, i) = circle_center_x + circle_radius*cos(theta/180*pi);
        exp_path_3d(1, i) = circle_center_y + circle_radius*sin(theta/180*pi);
    }

    // 截取z
    for(int i = 0; i<num_point; i++)
    {
        // 存进去
        exp_path_3d(2, i) = robot_z - robot_z/num_point - robot_z/num_point*i;
    }

#if PRINT_SWITCH
     cout << "exp_path_3d: " << endl << exp_path_3d <<endl;
#endif
}


void C_Docking::planLinePath(float _robot_x, float _robot_y, float _robot_z)
{
    robot_x = _robot_x;
    robot_y = _robot_y;
    robot_z = _robot_z;

    // 分别存x， y， z
    for(int i = 0; i<num_point ;i++)
    {
        exp_path_3d(0, i) = robot_x - (robot_x + medium_x)/num_point - (robot_x + medium_x)/num_point*i;
        exp_path_3d(1, i) = robot_y - (robot_y - 0)/num_point - (robot_y - 0)/num_point*i;
        exp_path_3d(2, i) = robot_z - robot_z/num_point - robot_z/num_point*i;
    }
#if PRINT_SWITCH
     cout << "exp_path_3d: " << endl << exp_path_3d <<endl;
#endif
}


MatrixXf C_Docking::outputExpPath()
{
    return exp_path_3d;
}







// hcy:新版本 docking 函数实现

C_Docking_v2::C_Docking_v2(){
    robot_yaw = 0;
    k_0 = 0;
    k_1 = 0;
    robot_x = 0;
    robot_y = 0;
    robot_z = 0;

    circle_center_x = 0;
    circle_center_y = 0;
    circle_radius = 0;

    medium_x = 0.5; // 单位是m
    end_x = 0.2;    // 单位是m

    heli_num_point = 400;
    line_num_point = 100;
    num_point = heli_num_point + line_num_point;  // 默认1000个点
    theta = 0;

    pi = 3.1415926535;
    heli_exp_path_3d = MatrixXf(3, heli_num_point);
    line_exp_path_3d = MatrixXf(3, line_num_point);
    exp_path_3d = MatrixXf(3, num_point);

#if PRINT_SWITCH
    cout <<  "the medium_x is:  " << medium_x <<endl;
    cout <<  "the number of expect points is: " << num_point << endl;
#endif

}

float C_Docking_v2::myabs(float _x){
    if(_x > 0.000001)
        _x = _x;
    else if(_x < -0.000001)
        _x = -_x;
    else
        _x = 0;
    return _x;
}

void C_Docking_v2::setMediumXandEndX(float _medium_x, float _end_x){
    medium_x = _medium_x;
    end_x = _end_x;
}

void C_Docking_v2::setPathPointsNum(int _num1, int _num2){
    heli_num_point = _num1;
    line_num_point = _num2;
}

void C_Docking_v2::planCirclePathLeft(float _robot_x, float _robot_y, float _robot_z){
    robot_x = _robot_x;
    robot_y = _robot_y;
    robot_z = _robot_z;
    start_yaw = -(180 - atan2(robot_y, robot_x + medium_x)/pi*180 )*2; //

    //if(start_yaw >= 0)
     //   start_yaw = -start_yaw; // 向左转弯

#if PRINT_SWITCH
    cout << "start_yaw: " << start_yaw <<endl;
#endif

    // 直线斜率
    k_0 = tan(start_yaw/180*pi);
    k_1 = -(1/k_0);


    // 圆心和半径
    circle_center_x = -medium_x;
    circle_center_y = k_1*( -medium_x - robot_x ) + robot_y;
    circle_radius = myabs( circle_center_y );

#if PRINT_SWITCH
    cout << "circle_center_x: " << circle_center_x <<endl;
    cout << "circle_center_y: " << circle_center_y <<endl;
    cout << "circle_radius: " << circle_radius <<endl;
    cout << "myabs(5): " << myabs(5) <<endl;
    cout << "myabs(-5): " << myabs(-5) <<endl;
#endif


    // 用参数方程截取点 从近到远
    for(int i = 0; i < heli_num_point ; i++){
        theta = -90 + start_yaw + myabs(start_yaw)/heli_num_point +
                ( myabs(start_yaw)/heli_num_point*i);
#if PRINT_SWITCH
    cout << "theta: " << theta <<endl;
#endif
        // 转换成弧度，再写进去
        heli_exp_path_3d(0, i) = circle_center_x + circle_radius*cos(theta/180*pi);
        heli_exp_path_3d(1, i) = circle_center_y + circle_radius*sin(theta/180*pi);
    }

    // 截取z
    for(int i = 0; i<heli_num_point; i++){
        // 存进去
        //heli_exp_path_3d(2, i) = robot_z - robot_z/heli_num_point - robot_z/heli_num_point*i;
	heli_exp_path_3d(2, i) = -0.15;
    }

#if PRINT_SWITCH
     cout << "heli_exp_path_3d: " << endl << heli_exp_path_3d <<endl;
#endif
}


void C_Docking_v2::planCirclePathRight(float _robot_x, float _robot_y, float _robot_z){
    robot_x = _robot_x;
    robot_y = _robot_y;
    robot_z = _robot_z;
    start_yaw = myabs(-180 - atan2(robot_y, robot_x + medium_x)/pi*180 )*2; //

    //if(start_yaw >= 0)
     //   start_yaw = -start_yaw; // 向左转弯

#if PRINT_SWITCH
    cout << "start_yaw: " << start_yaw <<endl;
#endif

    // 直线斜率
    k_0 = tan(start_yaw/180*pi);
    k_1 = -(1/k_0);


    // 圆心和半径
    circle_center_x = -medium_x;
    circle_center_y = k_1*( -medium_x - robot_x ) + robot_y;
    circle_radius = myabs( circle_center_y );

#if PRINT_SWITCH
    cout << "circle_center_x: " << circle_center_x <<endl;
    cout << "circle_center_y: " << circle_center_y <<endl;
    cout << "circle_radius: " << circle_radius <<endl;
    cout << "myabs(5): " << myabs(5) <<endl;
    cout << "myabs(-5): " << myabs(-5) <<endl;
#endif


    // 用参数方程截取点 从近到远
    for(int i = 0; i < heli_num_point ; i++)
    {
        theta = 90 + start_yaw - myabs(start_yaw)/heli_num_point -
                ( myabs(start_yaw)/heli_num_point*i);
#if PRINT_SWITCH
    cout << "theta: " << theta <<endl;
#endif
        // 转换成弧度，再写进去
        heli_exp_path_3d(0, i) = circle_center_x + circle_radius*cos(theta/180*pi);
        heli_exp_path_3d(1, i) = circle_center_y + circle_radius*sin(theta/180*pi);
    }

    // 截取z
    for(int i = 0; i<heli_num_point; i++)
    {
        // 存进去
        //heli_exp_path_3d(2, i) = robot_z - robot_z/heli_num_point - robot_z/heli_num_point*i;
	heli_exp_path_3d(2, i) = -0.15;
    }

#if PRINT_SWITCH
     cout << "heli_exp_path_3d: " << endl << heli_exp_path_3d <<endl;
#endif
}


void C_Docking_v2::planLinePath(float _robot_x, float _robot_y, float _robot_z)
{
    robot_x = _robot_x;
    robot_y = _robot_y;
    robot_z = _robot_z;

    // 分别存x， y， z
    for(int i = 0; i<heli_num_point ;i++)
    {
        heli_exp_path_3d(0, i) = robot_x - (robot_x + medium_x)/heli_num_point - (robot_x + medium_x)/heli_num_point*i;
        heli_exp_path_3d(1, i) = robot_y - (robot_y - 0)/heli_num_point - (robot_y - 0)/heli_num_point*i;
        // heli_exp_path_3d(2, i) = robot_z - (robot_z - 0)/heli_num_point - (robot_z - 0)/heli_num_point*i;
        heli_exp_path_3d(2, i) = -0.15;   // 暂时让z恒定
    }
#if PRINT_SWITCH
     cout << "heli_exp_path_3d: " << endl << heli_exp_path_3d <<endl;
#endif
}


void C_Docking_v2::planEndLinePath()
{
    // 分别存x， y， z
    for(int i = 0; i < line_num_point ;i++)
    {
        line_exp_path_3d(0, i) = -medium_x + (medium_x - end_x)/line_num_point + (medium_x - end_x)/line_num_point*i;
        line_exp_path_3d(1, i) = 0;
        line_exp_path_3d(2, i) = -0.15;
    }
#if PRINT_SWITCH
     cout << "line_exp_path_3d: " << endl << line_exp_path_3d <<endl;
#endif
}

// 将螺旋段和末端直线这两段轨迹拼接起来
MatrixXf C_Docking_v2::outputExpPath(){
    //cout << heli_exp_path_3d.size() << line_exp_path_3d.size() <<  exp_path_3d.size() <<endl;
    exp_path_3d.block(0,0,3,heli_num_point) = heli_exp_path_3d;
    exp_path_3d.block(0,heli_num_point,3,line_num_point) = line_exp_path_3d;
    return exp_path_3d;
}



}
