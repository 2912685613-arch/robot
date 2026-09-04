#ifndef C_VECTOREDTHRUSTCONTROL_H
#define C_VECTOREDTHRUSTCONTROL_H

#include "ros/ros.h"
#include "asrobot/pwm.h"

#include <stdio.h>
#include <stdlib.h>
#include "stdio.h"
#include "math.h"
#include <cmath>

#include "c_gaitcontrol.h"
#include "c_controller_group.h"
#include "c_filter_group.h"
#include "generate_trajectory.h"


//leg 1
#define     FL_CJ_ID        16      // H
#define     FL_FJ_ID        15    // M
#define     FL_TJ_ID        14      // V
#define     FL_WJP_ID       7       // P
//leg 2
#define     HL_CJ_ID        4
#define     HL_FJ_ID        1
#define     HL_TJ_ID        20
#define     HL_WJP_ID       9
//leg3
#define     HR_CJ_ID        18
#define     HR_FJ_ID        19
#define     HR_TJ_ID        17
#define     HR_WJP_ID       6
//leg4
#define     FR_CJ_ID        13
#define     FR_FJ_ID        12
#define     FR_TJ_ID        11
#define     FR_WJP_ID       8

// Locomoiton mode
#define     ONLAND_LOCOMOTION_TRIPOD_GAITS      0
#define     ONLAND_LOCOMOTION_TROT_GAITS        1
#define     ONLAND_LOCOMOTION_GAITS_Reserved1   2
#define     ONLAND_LOCOMOTION_GAITS_Reserved2   3

#define     UNDERWATER_LOCOMOTION_YAW           4
#define     UNDERWATER_LOCOMOTION_DEPTH         5
#define     UNDERWATER_LOCOMOTION_CRUISING      6
#define     UNDERWATER_LOCOMOTION_Reserved2     7
#define     UNDERWATER_LOCOMOTION_VISION        8

#define     NONE2                                10

// Joint Limitation
#define     FL_CJ_LEFT      1900    //-45deg
#define     FL_CJ_MIDDLE    1430    //0  deg
#define     FL_CJ_RIGHT     930    //45 deg
#define     FL_FJ_UP        2490    //
#define     FL_FJ_MIDDLE    2210
#define     FL_FJ_DOWN      1180
#define     FL_TJ_UP        690
#define     FL_TJ_MIDDLE    1210
#define     FL_TJ_DOWN      2190

#define     HL_CJ_LEFT      1920    //
#define     HL_CJ_MIDDLE    1400
#define     HL_CJ_RIGHT     910
#define     HL_FJ_UP        2490    //
#define     HL_FJ_MIDDLE    2270
#define     HL_FJ_DOWN      910
#define     HL_TJ_UP        630
#define     HL_TJ_MIDDLE    1150
#define     HL_TJ_DOWN      2120

#define     HR_CJ_LEFT      1960
#define     HR_CJ_MIDDLE    1500
#define     HR_CJ_RIGHT     1020
#define     HR_FJ_UP        2500
#define     HR_FJ_MIDDLE    2330    //1930 ?
#define     HR_FJ_DOWN      1280
#define     HR_TJ_UP        590
#define     HR_TJ_MIDDLE    1090
#define     HR_TJ_DOWN      2050

#define     FR_CJ_LEFT      1990    //
#define     FR_CJ_MIDDLE    1520
#define     FR_CJ_RIGHT     990
#define     FR_FJ_UP        2500    //
#define     FR_FJ_MIDDLE    2190
#define     FR_FJ_DOWN      1160
#define     FR_TJ_UP        650     //
#define     FR_TJ_MIDDLE    1200
#define     FR_TJ_DOWN      2110

#define     JOINT_ROTARY_LOW_SPEED          0x32
#define     JOINT_ROTARY_MIDDLE_SPEED       0x31
#define     JOINT_ROTARY_HIGH_SPEED         0x30

#define     BUFFER_SIZE     5
#define     pi     3.14159265358979

using namespace ControllerGroup;
using namespace GaitControl;
using namespace FilterGroup;


namespace VectoredThrustControl {

// 位姿结构体
typedef struct ASR_POSELOCATION
{
    float id;
    float x;
    float y;
    float z;
    float yaw;
    float roll;
    float pitch;

} ASR_PoseLoct;



// 腿形状枚举
enum LegStatus
{
    FB, LR, X
};



// 速度类
class Speed
{
public:
    float x;
    float y;
    float z;
    float yaw;

    Speed();

};

// 姿态类
class Attitude
{
public:
    float pitch;
    float roll;
    float yaw;
    
    Attitude();
};

// 位置类
class Position
{
public:
    float x;
    float y;
    float z;
    Position();
};


// 喷水电机推进类
class C_VectoredThrustControl
{

    
public: 
    // =======================================================
    // ===           单环控制 (Single Loop) PID            ===
    // =======================================================
    PID mcDepthPID, mcYawPID, mcPitchPID, mcSpeedPID, mcXPID, mcYPID; 
    
    double fmcYawpidError;
    double fmcYawpidValue;
    double fmcDepthpidError;
    double fmcDepthpidValue;
    double fmcSpeedpidError;
    double fmcSpeedpidValue;

    // =======================================================
    // ===           双环控制 (Dual Loop) PID            ===
    // =======================================================
    DualLoopPID mcDualLoop_Heave;
    double fmcDualLoop_Heave_OuterError;
    double fmcDualLoop_Heave_OuterOutput;
    double fmcDualLoop_Heave_InnerError;
    double fmcDualLoop_Heave_InnerOutput;

    LegStatus legstatus;

    C_KalmanFilter_WhiteNoise leg1_angle_filter, leg2_angle_filter,
                              leg3_angle_filter, leg4_angle_filter;
    float thrust1;
    float thrust2;
    float thrust3;
    float thrust4;

    float thrust_force_1,thrust_force_2, thrust_force_3,thrust_force_4;//hcy
    float leg_angle_1, leg_angle_2, leg_angle_3, leg_angle_4;//hcy

public:  // 常用函数声明
    C_VectoredThrustControl();
    void motorforce(float _force_x, float _force_y, float _force_z, float _torque_z, float _roll, float _yaw, float _pitch);
    void yawLocomMode_FB();
    void yawLocomMode_LR();
    void yawLocomMode_X();
    
    bool judgeDirection(double _expectValue, double _realValue);
    double keepPositiveValue(double _data);
    void attitudecontrol(Attitude _e_attitude, Speed _e_speed, float _e_depth, Attitude _c_attitude, Speed _c_speed, float _c_depth);
    
//驱动方式新版本

    void thrusterforce2speed(float _force1, float _force2, float _force3, float _force4);
    void yawLocomMode_FB(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle);
    void yawLocomMode_LR(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle);
    void yawLocomMode_X(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle);
    void yawLocomMode_G(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle);
    void shiftMode_G();
    void calcSprayForces_Shape_X(float _X_force, float _Y_force, float _Z_force,
                                      float *_force1, float *_force2, float *_force3, float *_force4,
                                      float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    void calcSprayForces_Shape_H(float _X_force, float _Z_force, float _yaw_force,
                                      float *_force1, float *_force2, float *_force3, float *_force4,
                                      float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    void calcSprayForces_Shape_H_T(float _Y_force, float _Z_force, float _yaw_force,
                                        float *_force1, float *_force2, float *_force3, float *_force4,
                                        float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    // 新解算策略 目前正在用该策略
    float myabs(float _a);

    float mapForce(float _src_left, float _src_right, float _dst_left, float _dst_right, float _input_value);

    void calcSprayForces_Shape_X_2(   float _X_force, float _Y_force, float _Z_force,
                                      float *_force1, float *_force2, float *_force3, float *_force4,
                                      float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    void calcSprayForces_Shape_H_2(float _X_force, float _Z_force, float _yaw_force,
                                      float *_force1, float *_force2, float *_force3, float *_force4,
                                      float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    void calcSprayForces_Shape_H_T_2(float _Y_force, float _Z_force, float _yaw_force,
                                        float *_force1, float *_force2, float *_force3, float *_force4,
                                        float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    void calcSprayForces_Shape_G(   float _X_force, float _Y_force, float _Z_force,
                                      float *_force1, float *_force2, float *_force3, float *_force4,
                                      float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle);
    void calcSprayForcesGrasp_Shape_G(float _controller_forward, float _controller_horizontal, float _controller_vertical, float _controller_torque,
                                                               float *_force1, float *_force2, float *_force3, float *_force4,
                                                               float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle,
                                                               float _force1_direction, float _object_direction);
    void PxControlWithPxForce(float _Px_force);
    void VxControlWithVxForce(float _Vx_force);
    void PyControlWithPyForce(float _Py_force);
    void VyControlWithVyForce(float _Vy_force);
    void yawControlWithYawForce(float _yaw_force);
    void depthControlWithPzForce(float _Pz_force);

    void doubleClosedLoopControlPx(float _Vx_force);
    void doubleClosedLoopControlPy(float _Vy_force);

    void space3DControl(int _leg_shape, float _X_force, float _Y_force, float _Z_force, float _yaw_force);
    void space3DGraspControl(int _leg_shape, float _forward_force, float _horizontal_force, float _vertical_force, float torque, float _force1_direction, float _object_direction);

    void get_thrust_force(float force1, float force2, float force3,float force4);


    void thrusterforce2speed_H(float _force1,float  _force2, float _force3,float _force4);//hcy
    void thrusterforce2speed_X(float _force1,float  _force2, float _force3,float _force4);//hcy
    void thrusterforce2speed_G(float _force1, float _force2, float _force3, float _force4);
};

}



#endif // VECTOREDTHRUSTCONTROL_H
