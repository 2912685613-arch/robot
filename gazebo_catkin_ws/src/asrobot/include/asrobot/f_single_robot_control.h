#ifndef F_SINGLE_ROBOT_CONTROL_H
#define F_SINGLE_ROBOT_CONTROL_H
#define SINGLE_ROBOT 1 // 单机模式
#define MULTI_ROBOT 2  // 编队模式

/*********************     ROS组件和消息头文件          ***********************/
#include "ros/ros.h"
#include "asrobot/pwm.h"
#include "asrobot/mc.h"
#include "asrobot/path_based_threeSpine.h"
#include "asrobot/mul_ref_trajectory.h"
#include "asrobot/quadrotor2robot.h"
#include "asrobot/status.h"
#include "std_msgs/String.h"

/*********************     -----------------          ***********************/

/*********************         C++标准库头文件           ***********************/
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <time.h>
/*********************     -----------------          ***********************/

/*********************         自定义库头文件             ***********************/
#include "c_controller_group.h"
#include "c_sim_3d_ctrl.h"
#include "c_vectoredthrustcontrol.h"
#include "c_gaitcontrol.h"
#include "MPCController.h"
#include "c_save_data.h"

/*********************        ---------------          ***********************/

#define FORWARD 0
#define FORWARD2 3
#define LEFT 1
#define RIGHT 2
#define BACK 4

using namespace std;

using namespace ControllerGroup;
using namespace CombineCtrl;
using namespace VectoredThrustControl;
using namespace GaitControl;

/*######################################################################################*/
/*@ 总目录
 *  按" ctrl + F " 对应查找
 *
 *    1. 单机控制变量目录
 *    2. 设置控制器参数，控制模式函数目录
 *    3. 机器人基本运动控制函数目录
 *    4. 三维控制函数目录
 *
 *
/*######################################################################################*/

extern ofstream fout;

/*######################################################################################*/
/*@ 获取从desicion订阅到的信息
 */
extern float vs_reference_PositionX;
extern float vs_reference_PositionY;
extern int last_ref_num;
extern int reference_Point_Num;
extern Eigen::MatrixXd robot1_reference_trajectory;
extern Eigen::MatrixXd robot2_reference_trajectory;
extern Eigen::MatrixXd robot3_reference_trajectory;
extern Eigen::MatrixXd virtureRobot_reference_trajectory;
extern VectoredThrustControl::Position Robot1_c_position;
extern VectoredThrustControl::Position Robot2_c_position;
extern VectoredThrustControl::Position Robot3_c_position;

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  单机控制变量目录
 *   按" ctrl + F " 对应查找
 *
 *     1. 基本变量
 *     2. 早期PID调试用到的变量
 *     3. 近期ADRC调试用到的变量
 *
 */

/******************************   基本变量   **********************************/
extern const int iMotionLoopRate; // 控制输出5Hz
/******************************   数据存储  **********************************/
extern float force_x, force_y, force_z, force_yaw;
extern float force_1, force_2, force_3, force_4;
extern float control_cmd_x, control_cmd_yaw;
extern float four_thrust1, four_thrust2, four_thrust3, four_thrust4; //%用于存储经过结算后实际给到机器人四个推进器上的力
extern float force_range_min, force_range_max;                       // 力的映射区间的范围
extern float force_for_surge, force_for_yaw, force_for_depth;        // 使用控制器求解的施加到不同自由度上的力
extern SaveData::C_SaveData save_debugdata;
extern SaveData::C_SaveData save_debugdata1;
extern SaveData::C_SaveData identi_data;

/******************************   早期PID调试用到的变量   **********************************/
extern int mode;
extern int dir;
extern float e_depthvalue;
extern float e_navyaw;
extern float c_depthvalue;
extern float c_navyaw;
extern float e_x;
extern float e_y;
extern float c_x;
extern float c_y;
extern float gait_R;
extern VectoredThrustControl::Attitude e_attitude, c_attitude;
extern VectoredThrustControl::Position c_position;
extern Speed e_speed, c_speed;

/******************************   MPC 轨迹跟踪用到的变量   **********************************/
extern float q11;
extern float q22;
extern float q33;
extern float q44;
extern float q55;
extern float Init_q33;
extern float Init_q22;
extern float Init_q44;
extern VectorXd MPC_real_force;
extern bool delta_mode;
extern int Linear_mode;

extern MPC mcYawMPC;
extern GenerateTrajectory trajectory;
extern MPC_trajectoryTracking tracking;
extern MPC_trajectoryTrackingyaw tracking3;
/*-------------------------------------------------------*/
extern int iter_num;
extern DyModel ASR_simulation_model;
extern MPCController MPC_tracking;
extern Eigen::Matrix2d MassMatrix;
extern Eigen::Matrix2d HydroNonLinear;
extern Eigen::Matrix2d HydroLinear;
/******************************   -----------------  **********************************/

/******************************   近期调试用到的变量   **********************************/
extern C_Combine3DControl comb3Dctrl;           // 该类的内部有变量vec_ctrl 不要与外部的vec_ctrl混淆
extern C_Combine3DSpeedControl comb3DSpeedCtrl; // 该类的内部有变量speed_vec_ctrl 不要与外部的vec_ctrl混淆
extern C_VectoredThrustControl vec_ctrl;
extern C_GaitControl gait_ctrl;

extern C_Combine3DPositionControl comb3DPositionCtrl;       // hcy:v1版本调试位置用这个
extern C_Combine3DPositionControl_v2 comb3DPositionCtrl_v2; // hcy:v2版本调试位置用这个

extern CombineCtrl::C_Combine3DPidControl combine3DPidCtrl;
/****************************** 矢量控制实例化************************************************/
extern C_VectoredThrustControl MPC_vec_ctrl;

// 期望变量
extern float exp_p_x, exp_v_x;
extern float exp_p_y, exp_v_y;
extern float exp_p_z, exp_v_z;
extern float exp_yaw, exp_roll, exp_pitch;
extern float exp_v_yaw, exp_v_roll, exp_v_pitch;
extern float exp_force;

// 反馈(实时采集)量
extern float cur_p_x, cur_v_x;
extern float cur_p_y, cur_v_y;
extern float cur_p_z, cur_v_z;
extern float cur_cla_vz;



extern float cur_yaw, cur_roll, cur_pitch;
extern float cur_v_yaw, cur_v_roll, cur_v_pitch;

// hcy:接受双目相机的期望和实际数据
extern float cam_exp_p_x;
extern float cam_exp_p_y;
extern float cam_exp_p_z;
extern float cam_exp_yaw, cam_exp_roll, cam_exp_pitch;
extern float cam_cur_p_x;
extern float cam_cur_p_y;
extern float cam_cur_p_z;
extern float cam_cur_yaw, cam_cur_roll, cam_cur_pitch;

// ljl：新定义的aruco码到机体位姿
extern float aruco_pos_x, aruco_pos_y, aruco_pos_z;
extern float aruco_roll, aruco_pitch, aruco_yaw;
// hcy:深度计数据
extern float depth_sensor_data;
// hcy:螺旋下降轨迹半径
extern float heli_traj_radius;
/******************************   -----------------  **********************************/

/******************************   发布器，消息对外扩展   **********************************/
extern ros::Publisher pub_pwm;
extern ros::Publisher pub_status;

extern ros::NodeHandle *extern_n;
extern ros::Publisher *extern_pub_pwm;
extern ros::Publisher *extern_pub_status;

extern asrobot::pwm pwm_msg; // 输出pwm信息
extern asrobot::status key;  // 输出舵机开关
/******************************   data save **********************************/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void save_identi_data(float px, float py, float speed_x, float acc_x, float yawnagle,
                      float speed_yaw, float fx, float fyaw);
void save_identi_data(float px, float py, float speed_x, float speed_y, float yawnagle, float speed_yaw,
                      float f1, float f2, float f3, float f4);

/* * * * * * * * * * * * * * * * * * * * * hcy* * * * * * * * * * * * * * * * * * * * * * * * */
// void save_this_robot_data_H_mode(float px, float py, float speed_x, float speed_y, float yawnagle, float speed_yaw,
//                       float fx, float fyaw);
// void save_this_robot_data_H_mode_global_cam(float px, float py, float speed_x, float speed_y, float yawnagle, float speed_yaw,
//                       float fx, float fyaw);
// void save_this_robot_data_X_mode(float px, float py, float speed_x, float speed_y, float yawnagle, float speed_yaw,
//                       float f1, float f2, float f3, float f4);
// void save_this_robot_data_X_mode_2( float _pos_x, float _pos_y, float _pos_z,
//                                     float _ref_pos_x, float _ref_pos_y, float _ref_pos_z,
//                                     float _force_x, float _force_y, float _force_z,
//                                     float _thrust_force_1, float _thrust_force_2, float _thrust_force_3, float _thrust_force_4);
// void save_thrust_data(int _iterm1,int _iterm, float f1, float f2, float f3, float f4);
// void save_identi_data(float px,float py, float speed_x,float speed_y, float yawnagle, float speed_yaw,
//                    float f1,float f2, float f3, float f4);
void save_this_robot_docking_data(float _robot_pos_x, float _robot_pos_y, float _robot_pos_z,
                                  float _robot_roll, float _robot_pitch, float _robot_yaw,
                                  float _robot_depth_sensor,
                                  float _robot_traj_radius,
                                  float _robot_ref_pos_x, float _robot_ref_pos_y, float _robot_ref_pos_z, float _robot_dest_yaw,
                                  float _force_x, float _torque_yaw, float _force_z,
                                  float _thrust_force_1, float _thrust_force_2, float _thrust_force_3, float _thrust_force_4,
                                  float _leg_angle_1, float _leg_angle_2, float _leg_angle_3, float _leg_angle_4);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  设置控制器参数，控制模式函数目录
 *   按" ctrl + F " 对应查找
 *
 *     1. 设置PID参数
 *     2. 设置ADRC参数
 *     3. 设置S面参数
 *     4. 设置控制模式
 *
 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void setYawPIDPmt(float _k_p, float _k_i, float _k_d); // 设置PID参数
void setPitchPIDPmt(float _k_p, float _k_i, float _k_d);
void setXPIDPmt(float _k_p, float _k_i, float _k_d);
void setYPIDPmt(float _k_p, float _k_i, float _k_d);
void setDepthPIDPmt(float _k_p, float _k_i, float _k_d);
void setSpeedPIDPmt(float _k_p, float _k_i, float _k_d);
void setVelDepPIDPmt(float _k_p, float _k_i, float _k_d);
void setPosDepPIDPmt(float _k_p, float _k_i, float _k_d);

void setPxADRCPmt(float _k_p, float _k_d, float _band, float _b_0); // 设置ADRC参数
void setVxADRCPmt(float _k_p, float _k_d, float _band, float _b_0);
void setPyADRCPmt(float _k_p, float _k_d, float _band, float _b_0);
void setVyADRCPmt(float _k_p, float _k_d, float _band, float _b_0);
void setPzADRCPmt(float _k_p, float _k_d, float _band, float _b_0);
void setYawADRCPmt(float _k_p, float _k_d, float _band, float _b_0);

void setSpeedXTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0);
void setSpeedXNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit);
void setSpeedYTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0);
void setSpeedYNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit);
void setSpeedYawTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0);
void setSpeedYawNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit);
void setSpeedZTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0);
void setSpeedZNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit);
void setYawTDandESOPmt(float _k_p, float _k_d, float _band, float _b_0);

void setPxSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain); // 设置S面参数
void setVxSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain);
void setPySSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain);
void setVySSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain);
void setPzSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain);
void setYawSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain);

void setComb3DCtrlMode(int _mode); // 设置控制模式
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  机器人基本运动控制函数目录
 *   按" ctrl + F " 对应查找
 *
 *     1. 设置腿的形状
 *     2. 对机器人施加推力
 *     3. 设置腿的位置和角度
 *     4. 机器人步态控制
 *
 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void LegShape_H(); // 设置腿的形状
void LegShape_H_T();
void LegShape_X();

void simMotorForce(float _force_x, float _force_y, float _force_z,
                   float _torque_z,
                   float _roll, float _yaw, float _pitch); // 对机器人施加推力

void legSetPosition(float _x, float _y, float _z); // 设置腿的位置和角度
void legSetAngle(float _A, float _B, float _C);

void gaitControl(); // 机器人步态控制
void gait_stop();

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  三维控制函数目录
 *   按" ctrl + F " 对应查找
 *
 *     1. ADRC型三维控制
 *     2. S面型三维控制
 *     3. PID三维控制
 *
 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void simCombineADRC_3DCtrl(float _exp_p_x, float _exp_v_x,
                           float _exp_p_y, float _exp_v_y,
                           float _exp_p_z, float _exp_yaw,

                           float _cur_p_x, float _cur_v_x,
                           float _cur_p_y, float _cur_v_y,
                           float _cur_p_z, float _cur_yaw); // ADRC型三维控制

void simCombineSS_3DCtrl(float _exp_p_x, float _exp_v_x,
                         float _exp_p_y, float _exp_v_y,
                         float _exp_p_z, float _exp_yaw,

                         float _cur_p_x, float _cur_v_x,
                         float _cur_p_y, float _cur_v_y,
                         float _cur_p_z, float _cur_yaw); // S面型三维控制

void simCombineADRC_3DSpeedCtrl(float _exp_v_x, float _exp_v_y,
                                float _exp_v_yaw, float _exp_v_z,

                                float _cur_v_x, float _cur_v_y,
                                float _cur_v_yaw, float _cur_v_z);

// 根据角速度和角度进行控制
void simCombineADRC_3DSpeedCtrl_v2(float _exp_v_x, float _exp_v_y,
                                   float _exp_v_yaw, float _exp_v_z,
                                   float _exp_yaw,

                                   float _cur_v_x, float _cur_v_y,
                                   float _cur_v_yaw, float _cur_v_z,
                                   float _cur_yaw);

void simCombinePID_3DCtrl(float _exp_p_x, float _exp_v_x,
                          float _exp_p_y, float _exp_v_y,
                          float _exp_p_z, float _exp_yaw,

                          float _cur_p_x, float _cur_v_x,
                          float _cur_p_y, float _cur_v_y,
                          float _cur_p_z, float _cur_yaw); // PID 三维控制

void PidDepthControl_X(float _exp_p_z, float _cur_p_z);
void PidDepVelControl_X(float _exp_v_z, float _cur_v_z);
void PidYawControl_H(float _exp_yaw, float _cur_yaw);
void PidDepthYawControl_H(float _exp_p_z, float _exp_v_z, float _exp_yaw, float _cur_p_z, float _cur_v_z, float _cur_yaw);
void PidXYZControl_X(float _exp_p_x, float _exp_p_y, float _exp_p_z, float _cur_p_x, float _cur_p_y, float _cur_p_z, float _cur_yaw);
void PidXYZControl_X(float _exp_p_x, float _exp_p_y, float _exp_p_z, float _cur_p_x, float _cur_p_y, float _cur_p_z, float _cur_yaw, float cur_v_z);
void PidXYZControl_H(float _exp_p_x, float _exp_p_y, float _exp_p_z, float _cur_p_x, float _cur_p_y, float _cur_p_z, float _cur_yaw);
// 每次急停时 将控制器内部的全局变量清0
void clearADRCSpeedControllerVariables();

void ADRCPosition3DControl_G(float _exp_x, float _exp_y, float _exp_yaw, float _exp_z, float _cur_x, float _cur_y, float _cur_yaw, float _cur_z);
/***************************************hcy***************************************/

void ADRCYawControl_H(float _exp_yaw, float _cur_yaw); // hcy
void ADRCDepthControl_X(float _exp_z, float _cur_z);   // hcy

void ADRCPosition3DControl_H_v2(float _exp_x, float _exp_y, float _exp_z,
                                float _cur_x, float _cur_y, float _cur_yaw, float _cur_z);
void ADRCPosition3DControl_H_v2(float _exp_x, float _exp_y, float _exp_yaw, float _exp_z, // hcy:docking
                                float _cur_x, float _cur_y, float _cur_yaw, float _cur_z);

void ADRCPosition3DControl_X(float _exp_x, float _exp_y, float _exp_z,
                             float _cur_x, float _cur_y, float _cur_yaw, float _cur_z);

void setDebugPzADRCPmt_v2(float _r_0, float _w_c, float _w_0, float _b_0);                // hcy:4个参数的z调节
void setDebugPzADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0);    // hcy:5个参数的z调节（正在使用）
void setDebugPzADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0); // hcy:6个参数的z调节

void setDebugYawADRCPmt_v2(float _r_0, float _w_c, float _w_0, float _b_0);                // hcy:4个参数的yaw调节
void setDebugYawADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0);    // hcy:5个参数的yaw调节（正在使用）
void setDebugYawADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0); // hcy:6个参数的yaw调节

void setDebugPxADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0); // hcy:5个参数的x调节
void setDebugPyADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0); // hcy:5个参数的y调节

void setDebugPxADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0); // hcy:6个参数的x调节（正在使用）
void setDebugPyADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0); // hcy:6个参数的y调节（正在使用）
/*********************************************************************************/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*基于MPC运动控制
*********************************************************************************/
void initMpcOsqp();
void update_iter();
void attitudeMPCcontrol(Attitude e_attitude, Attitude c_attitude);
void f_tracking1(Position c_position, Attitude c_attitude);
void f_tracking3(Position c_position, Attitude c_attitude);
void yawidentification();
void surgeidentification();

void f_mpcTracking(Position c_position, Attitude c_attitude, Speed c_speed);

#endif // F_SINGLE_ROBOT_CONTROL_H
