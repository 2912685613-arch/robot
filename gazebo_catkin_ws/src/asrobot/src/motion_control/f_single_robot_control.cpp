#include "../../include/asrobot/f_single_robot_control.h"
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

/*######################################################################################*/
/*@ 获取订阅到的多机参考轨迹的信息
 */

int last_ref_num = 0;
int reference_Point_Num;
float vs_reference_PositionX;
float vs_reference_PositionY;
Eigen::MatrixXd robot1_reference_trajectory;
Eigen::MatrixXd robot2_reference_trajectory;
Eigen::MatrixXd robot3_reference_trajectory;
Eigen::MatrixXd virtureRobot_reference_trajectory;
VectoredThrustControl::Position Robot1_c_position;
VectoredThrustControl::Position Robot2_c_position;
VectoredThrustControl::Position Robot3_c_position;
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
// const int iMotionLoopRate = 5; // 控制输出5Hz
const int iMotionLoopRate = 30; // 控制输出Hz

/******************************   数据存储  **********************************/
float force_x = 0, force_y = 0, force_z = 0, force_yaw = 0;
float force_1 = 0, force_2 = 0, force_3 = 0, force_4 = 0;
float control_cmd_x = 0, control_cmd_yaw = 0;
float four_thrust1 = 0, four_thrust2 = 0, four_thrust3 = 0, four_thrust4 = 0; //%用于存储经过结算后实际给到机器人四个推进器上的力
float force_range_min = 0, force_range_max = 0;                               // 力的映射区间的范围
float force_for_surge = 0, force_for_yaw = 0, force_for_depth = 0;
SaveData::C_SaveData save_debugdata("/home/uuv_ws/src/debugdata.txt");
SaveData::C_SaveData save_debugdata1("/home/uuv_ws/src/debugdata1.txt");
SaveData::C_SaveData identi_data("/home/xiadebin/catkin_ws/src/gazebo_ros_robot/asrobot/data/robotPos.txt");
SaveData::C_SaveData this_robot_docking_data("/home/tortoise/gazebo_catkin_ws/src/gazebo_ros_robot/asrobot/data/docking_data3.txt"); // hcy

/******************************   早期PID调试用到的变量   **********************************/
int mode = 100;
int dir = 0;
float e_depthvalue = -1.28;
float e_navyaw = 90;
float c_depthvalue = -1.28;
float c_navyaw = 0;
float e_x = 0;
float e_y = 0;
float c_x = 0;
float c_y = 0;
float gait_R = 1000000;
int iter_num = 0;
Eigen::VectorXd MPC_real_force = Eigen::VectorXd::Zero(2);

Attitude e_attitude, c_attitude;
Speed e_speed, c_speed;
C_VectoredThrustControl MPC_vec_ctrl;
/*************************************************************************/
// MPC mcYawMPC;
// MatrixXf Su = mcYawMPC.ComputeSu();
// MatrixXf Sx = mcYawMPC.ComputeSx();
// MatrixXf A = mcYawMPC.ComputeConstrain(Su);
// GenerateTrajectory trajectory();
// MatrixXf Reftrajectory=trajectory.GenerateLine_5(num_trackingpoints, 0.05,0.5);//得到期望轨迹的数据矩阵
// MPC_trajectoryTracking tracking(num_trackingpoints, 25,5,2);
// MatrixXf tao=tracking.Generate_RefTao(Reftrajectory);

// MPC_trajectoryTrackingyaw tracking3(num_trackingpoints, 25,4,1);
// VectorXf tao1=tracking3.Generate_RefTao(Reftrajectory);
/*
 *@ some global variables in the MPC trajectory tracking
 *  MPC用到的全局变量
 *
 */
Eigen::Matrix2d MassMatrix;
Eigen::Matrix2d HydroNonLinear;
Eigen::Matrix2d HydroLinear;
DyModel ASR_simulation_model;
int num_trackingpoints;
MatrixXd leader_ref_trajectory;
MatrixXd ref_trajectory;         // 每个机器人自己的参考轨迹
MatrixXd vrRobot_ref_trajectory; // 虚拟机器人的参考轨迹
MatrixXd ref_controlInput;
double samplingtime = 0.05;
double desired_veclocity = 0.2;
const int Np = 15;
const int Nc = 8;
bool relax_mode = 1;
const int X_num = 5;
const int Y_num = 5;
const int U_num = 2;
double Ro = 10;

/*laterline formation tracking a line*/
// float q11 = 2;
// float q22 = 3;
// float q33 = 0.3;
// float q44 = 1;
// float q55 = 1;
/*lonerline formation tracking a line*/
// float q11 = 2;
// float q22 = 2;
// float q33 = 0.7;
// float q44 = 1;
// float q55 = 1;

/* triangle formation tracking a line*/
// float q11 = 2;
// float q22 = 2;
// float q33 = 0.5;
// float q44 = 1;
// float q55 = 1;

// float q11 = 2;
// float q22 = 2;
// float q33 = 0.6;
// float q44 = 1;
// float q55 = 1;

// 基于新的模型2021-01-18
// 没有山的直线参数
//  float q11 = 1;
//  float q22 = 5;
//  float q33 = 0.5;
//  float q44 = 10;
//  float q55 = 1;
// 有山的曲线参数
float q11 = 1;
float q22 = 5;
float q33 = 1;
float q44 = 10;
float q55 = 1;
float Init_q33 = q33;
float Init_q22 = q22;
float Init_q44 = q44;
MPCController MPC_tracking;
bool delta_mode = 1; // 1 is the delta mode
int Linear_mode = 2; // 1 or 2. 1 represents the first linear method; 2 represents the second linear method.
/**********************************************************************************************/
/******************************基于osqp的MPC初始化****************************************************/
// Model of te ASR
void initMpcOsqp()
{

    // MassMatrix << 3.3648, 0,
    //     0, 0.0103;

    // HydroNonLinear <<-1.0644, 0,
    //     0, -0.32;

    // HydroLinear << 5.2493, 0,
    //     0, 0.1711;
    float beishu = 0.00031 * 25 * 25;
    MassMatrix << 30.472 * beishu, 0,
        0, 12.9629 * beishu;

    HydroNonLinear << -5.2121 * beishu, 0,
        0, 90.4965 * beishu;

    HydroLinear << 29.4036 * beishu, 0,
        0, 5.623 * beishu;

    ASR_simulation_model.MassMatrix = MassMatrix;
    ASR_simulation_model.HydroNonLinMatrix = HydroNonLinear;
    ASR_simulation_model.HydroLinMatrix = HydroLinear;
    const Vector2d umax(5, 4);   // force_motor<4N radiu_motor=15cm length_leg=15cm force_yaw<2*4*0.15N
    const Vector2d umin(-1, -4); // umin<0 result in pos_x_0<0
                                 //  the weight coefficient of the relax factor
    const Vector2d dumax(0.5, 1);
    const Vector2d dumin(-0.5, -1);
    // GenerateTrajectory trajectory(0.1, desired_veclocity, num_trackingpoints + 2 * Np);
    // Line parameter(0.5,1,0.35 1 1);Sine (1,25 1 1 1)(1,25 1.5 1 1)(1,25 2 1 1) strait point(-5 -0.5 0.6)
    // ref_trajectory = trajectory.GenerateLine_5(); //GenerateLine_5,GenerateSquare_5,GenerateSine_5
    // ref_controlInput = trajectory.GenereteTau(ref_trajectory, ASR_simulation_model);
    if (!delta_mode)
    {
        cout << "coming into the nodelta mode" << endl;
        MPC_tracking = MPCController(ASR_simulation_model, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin);
    }

    if (delta_mode)
    {
        cout << "coming into the delta mode" << endl;
        MPC_tracking = MPCController(ASR_simulation_model, samplingtime, Np, Nc, X_num, U_num, Y_num, relax_mode, umax, umin, dumax, dumin);
    }
}
/******************************   -----------------  **********************************/

/******************************   近期ADRC调试用到的变量   **********************************/
C_Combine3DControl comb3Dctrl;           // 该类的内部有变量vec_ctrl 不要与外部的vec_ctrl混淆
C_Combine3DSpeedControl comb3DSpeedCtrl; // 该类的内部有变量speed_vec_ctrl 不要与外部的vec_ctrl混淆
C_VectoredThrustControl vec_ctrl;        // 给PID控制用的变量 vec_ctrl
C_GaitControl gait_ctrl;

C_Combine3DPositionControl comb3DPositionCtrl;       // hcy:v1版本调试位置用这个
C_Combine3DPositionControl_v2 comb3DPositionCtrl_v2; // hcy:v2版本调试位置用这个

CombineCtrl::C_Combine3DPidControl combine3DPidCtrl; // 目前用的最新的

// 期望变量
float exp_p_x, exp_v_x;
float exp_p_y, exp_v_y;
float exp_p_z, exp_v_z;
float exp_yaw, exp_roll, exp_pitch;
float exp_v_yaw, exp_v_roll, exp_v_pitch;
float exp_force;

// 反馈(实时采集)量
float cur_p_x, cur_v_x;
float cur_p_y, cur_v_y;
float cur_p_z, cur_v_z;
float cur_cla_vz;

float cur_yaw, cur_roll, cur_pitch;
float cur_v_yaw, cur_v_roll, cur_v_pitch;

///
// hcy:接受双目相机的期望和实际数据
float cam_exp_p_x;
float cam_exp_p_y;
float cam_exp_p_z;
float cam_exp_yaw, cam_exp_roll, cam_exp_pitch;

float cam_cur_p_x;
float cam_cur_p_y;
float cam_cur_p_z;
float cam_cur_yaw, cam_cur_roll, cam_cur_pitch;
// hcy:深度计数据
float depth_sensor_data;
// hcy:螺旋下降轨迹半径
float heli_traj_radius;
///
/******************************   数据存储-  **********************************/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void save_identi_data(float px, float py, float speed_x, float speed_y, float yawnagle, float speed_yaw,
                      float fx, float fyaw)
{
    cout << "conming into the save data mode" << endl;
    identi_data.write_data(px, py, speed_x, speed_y, yawnagle, speed_yaw, fx, fyaw);
}
void save_identi_data(float px, float py, float speed_x, float speed_y, float yawnagle, float speed_yaw,
                      float f1, float f2, float f3, float f4)
{
    cout << "conming into the save data mode" << endl;
    identi_data.write_data(px, py, speed_x, speed_y, yawnagle, speed_yaw, f1, f2, f3, f4);
}

/* * * * * * * * * * * * * * * * * * * * * * *hcy * * * * * * * * * * * * * * * * * * * * * * */
/*
void save_this_robot_data_H_mode(float px, float py, float speed_x, float speed_y, float yawnagle,
                                 float speed_yaw, float fx, float fyaw){
    cout << "H shape H shape H shape save data mode" << endl;
    this_robot_data_H.write_data(px, py, speed_x, speed_y, yawnagle, speed_yaw, fx, fyaw);
}

void save_this_robot_data_H_mode_global_cam(float px, float py, float speed_x, float speed_y,
                                            float yawnagle, float speed_yaw, float fx, float fyaw){
    cout << "H shape H shape H shape save data mode" << endl;
    this_robot_data_H_global_cam.write_data(px, py, speed_x, speed_y, yawnagle, speed_yaw, fx, fyaw);
}

void save_this_robot_data_X_mode(float px, float py, float speed_x, float speed_y, float yawnagle,
                                 float speed_yaw, float f1, float f2, float f3, float f4){
    cout << "X shape X shape X shape save data mode" << endl;
    this_robot_data_X.write_data(px, py, speed_x, speed_y, yawnagle, speed_yaw, f1, f2, f3, f4);
}

void save_thrust_data(int _iterm1,int _iterm, float f1, float f2, float f3, float f4){
    // cout << "conming into the save data mode" << endl;
    save_thrust.write_data(_iterm1,_iterm,f1, f2, f3, f4);
}
void save_identi_data(float px, float py, float speed_x, float speed_y, float yawnagle,
                      float speed_yaw, float f1, float f2, float f3, float f4){
    //cout << "conming into the save data mode" << endl;
    identi_data.write_data(px, py, speed_x, speed_y, yawnagle, speed_yaw, f1, f2, f3, f4);
}

// 较全的保存X型运动模式的数据
void save_this_robot_data_X_mode_2( float _pos_x, float _pos_y, float _pos_z,
                                    float _ref_pos_x, float _ref_pos_y, float _ref_pos_z,
                                    float _force_x, float _force_y, float _force_z,
                                    float _thrust_force_1, float _thrust_force_2, float _thrust_force_3, float _thrust_force_4)
{
    cout << "X shape X shape X shape save data mode v2" << endl;
    this_robot_data_X_2.write_data_2(_pos_x, _pos_y, _pos_z,
                                     _ref_pos_x, _ref_pos_y, _ref_pos_z,
                                     _force_x, _force_y, _force_z,
                                     _thrust_force_1, _thrust_force_2, _thrust_force_3, _thrust_force_4);
}
*/
// 较全的保存H型运动模式的数据
void save_this_robot_docking_data(float _robot_pos_x, float _robot_pos_y, float _robot_pos_z,
                                  float _robot_roll, float _robot_pitch, float _robot_yaw,
                                  float _robot_depth_sensor,
                                  float _robot_traj_radius,
                                  float _robot_ref_pos_x, float _robot_ref_pos_y, float _robot_ref_pos_z, float _robot_dest_yaw,
                                  float _force_x, float _torque_yaw, float _force_z,
                                  float _thrust_force_1, float _thrust_force_2, float _thrust_force_3, float _thrust_force_4,
                                  float _leg_angle_1, float _leg_angle_2, float _leg_angle_3, float _leg_angle_4)
{
    this_robot_docking_data.write_data_3(_robot_pos_x, _robot_pos_y, _robot_pos_z,
                                         _robot_roll, _robot_pitch, _robot_yaw,
                                         _robot_depth_sensor,
                                         _robot_traj_radius,
                                         _robot_ref_pos_x, _robot_ref_pos_y, _robot_ref_pos_z, _robot_dest_yaw,
                                         _force_x, _torque_yaw, _force_z,
                                         _thrust_force_1, _thrust_force_2, _thrust_force_3, _thrust_force_4,
                                         _leg_angle_1, _leg_angle_2, _leg_angle_3, _leg_angle_4);
    cout << " ############  saving docking data #############" << endl;
}
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
/*@
 * 功能： 设置PID参数： 设置航向角，俯仰角，深度，前进速度的PID控制器的参数
 *
 * 输入： Kp, Ki, Kd
 *
 * 输出： 无
 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 设置PID参数： 设置航向角，俯仰角，深度，前进速度的PID控制器的参数
 *
 * 输入： Kp, Ki, Kd
 *
 * 输出： 无
 */
// void setYawPIDPmt(float _k_p, float _k_i, float _k_d)
// {
//     comb3DPositionCtrl_v2.vec_ctrl.mcYawPID.setKpPara(_k_p);
//     comb3DPositionCtrl_v2.vec_ctrl.mcYawPID.setKiPara(_k_i);
//     comb3DPositionCtrl_v2.vec_ctrl.mcYawPID.setKdPara(_k_d);
//     comb3DPositionCtrl_v2.vec_ctrl.mcYawPID.enable = true;

//     cout << " yaw pid parameters are: "
//          << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
// }

// void setPitchPIDPmt(float _k_p, float _k_i, float _k_d)
// {
//     comb3DPositionCtrl_v2.vec_ctrl.mcPitchPID.setKpPara(_k_p);
//     comb3DPositionCtrl_v2.vec_ctrl.mcPitchPID.setKiPara(_k_i);
//     comb3DPositionCtrl_v2.vec_ctrl.mcPitchPID.setKdPara(_k_d);
//     comb3DPositionCtrl_v2.vec_ctrl.mcPitchPID.enable = true;
//     cout << "pitch pid parameters are: "
//          << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
// }

// void setDepthPIDPmt(float _k_p, float _k_i, float _k_d)
// {
//     comb3DPositionCtrl_v2.vec_ctrl.mcDepthPID.setKpPara(_k_p);
//     comb3DPositionCtrl_v2.vec_ctrl.mcDepthPID.setKiPara(_k_i);
//     comb3DPositionCtrl_v2.vec_ctrl.mcDepthPID.setKdPara(_k_d);
//     comb3DPositionCtrl_v2.vec_ctrl.mcDepthPID.enable = true;
//     cout << "depth pid parameters are: "
//          << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
// }

// void setSpeedPIDPmt(float _k_p, float _k_i, float _k_d)
// {
//     comb3DPositionCtrl_v2.vec_ctrl.mcSpeedPID.setKpPara(_k_p);
//     comb3DPositionCtrl_v2.vec_ctrl.mcSpeedPID.setKiPara(_k_i);
//     comb3DPositionCtrl_v2.vec_ctrl.mcSpeedPID.setKdPara(_k_d);
//     comb3DPositionCtrl_v2.vec_ctrl.mcSpeedPID.enable = true;
//     cout << "speed pid parameters are: "
//          << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
// }
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 设置ADRC参数： 设置位置x 速度x， 位置y， 速度y， 位置z， 航向角的 ADRC 控制器参数
 *
 * 输入：Kp, Kd, BAND(带宽), b0(补偿因子)
 *
 * 输出： 无
 */
void setPxADRCPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3Dctrl.adrc_Px_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    cout << "[ADRC Px] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

void setVxADRCPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3Dctrl.adrc_Vx_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    cout << "[ADRC Vx] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

void setPyADRCPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3Dctrl.adrc_Py_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    cout << "[ADRC Py] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

void setVyADRCPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3Dctrl.adrc_Vy_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    cout << "[ADRC Vy] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

void setPzADRCPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3Dctrl.adrc_Pz_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    cout << "[ADRC Pz] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

void setYawADRCPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3Dctrl.adrc_yaw_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    comb3Dctrl.adrc_yaw_ctrl.m_r_0 = 100;
    cout << "[ADRC yaw] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 设置用于速度追踪的ADRC参数 x速度 y速度 航向角速度 z方向的速度
 *
 * 输入：采样时间sample_time, 过渡过程加速度r0, 观测器带宽w, 补偿因子b0
 *    : 比例 k_p, 积分k_i, 微分k_d, 积分限幅i_limit
 *
 * 输出： 无
 */
// 速度x
void setSpeedXTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0)
{
    comb3DSpeedCtrl.adrc_speed_x_ctrl.SetTDandESOPmt(_sample_time, _r_0, _w, _b_0);
    cout << "[ADRC Vx] Parameters are: "
         << "_sample_time: " << _sample_time << " _r_0: " << _r_0 << " _w: " << _w << " _b_0: " << _b_0 << endl;
}
void setSpeedXNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit)
{
    comb3DSpeedCtrl.adrc_speed_x_ctrl.SetNLSEFPmt(_k_p, _k_i, _k_d, _i_limit);
    cout << "[ADRC Vx] Parameters are: "
         << "_k_p: " << _k_p << " _k_i: " << _k_i << " _k_d: " << _k_d << " _i_limit: " << _i_limit << endl;
}

// 速度y
void setSpeedYTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0)
{
    comb3DSpeedCtrl.adrc_speed_y_ctrl.SetTDandESOPmt(_sample_time, _r_0, _w, _b_0);
    cout << "[ADRC Vy] Parameters are: "
         << "_sample_time: " << _sample_time << " _r_0: " << _r_0 << " _w: " << _w << " _b_0: " << _b_0 << endl;
}
void setSpeedYNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit)
{
    comb3DSpeedCtrl.adrc_speed_y_ctrl.SetNLSEFPmt(_k_p, _k_i, _k_d, _i_limit);
    cout << "[ADRC Vy] Parameters are: "
         << "_k_p: " << _k_p << " _k_i: " << _k_i << " _k_d: " << _k_d << " _i_limit: " << _i_limit << endl;
}

// 速度yaw
void setSpeedYawTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0)
{
    comb3DSpeedCtrl.adrc_speed_yaw_ctrl.SetTDandESOPmt(_sample_time, _r_0, _w, _b_0);
    cout << "[ADRC Vyaw] Parameters are: "
         << "_sample_time: " << _sample_time << " _r_0: " << _r_0 << " _w: " << _w << " _b_0: " << _b_0 << endl;
}
void setSpeedYawNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit)
{
    comb3DSpeedCtrl.adrc_speed_yaw_ctrl.SetNLSEFPmt(_k_p, _k_i, _k_d, _i_limit);
    cout << "[ADRC Vyaw] Parameters are: "
         << "_k_p: " << _k_p << " _k_i: " << _k_i << " _k_d: " << _k_d << " _i_limit: " << _i_limit << endl;
}

// 速度z
void setSpeedZTDandESOPmt(float _sample_time, float _r_0, float _w, float _b_0)
{
    comb3DSpeedCtrl.adrc_speed_z_ctrl.SetTDandESOPmt(_sample_time, _r_0, _w, _b_0);
    cout << "[ADRC Vz] Parameters are: "
         << "_sample_time: " << _sample_time << " _r_0: " << _r_0 << " _w: " << _w << " _b_0: " << _b_0 << endl;
}
void setSpeedZNLSEFPmt(float _k_p, float _k_i, float _k_d, float _i_limit)
{
    comb3DSpeedCtrl.adrc_speed_z_ctrl.SetNLSEFPmt(_k_p, _k_i, _k_d, _i_limit);
    cout << "[ADRC Vz] Parameters are: "
         << "_k_p: " << _k_p << " _k_i: " << _k_i << " _k_d: " << _k_d << " _i_limit: " << _i_limit << endl;
}

// 航向角
void setYawTDandESOPmt(float _k_p, float _k_d, float _band, float _b_0)
{
    comb3DSpeedCtrl.adrc_yaw_ctrl.SetadrcPmt(_k_p, _k_d, _band, _b_0);
    comb3DSpeedCtrl.adrc_yaw_ctrl.m_r_0 = 100;
    cout << "[ADRC Yaw] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d << " _band: " << _band << " _b_0: " << _b_0 << endl;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 设置S面参数： 设置位置x 速度x， 位置y， 速度y， 位置z， 航向角的 ADRC 控制器参数
 *
 * 输入：Kp, Ki, Kd, _outside_gain(外部增益， 也可以看做是控制器输出的限幅值，这个量千万不要写0)
 *
 * 输出： 无
 */
void setPxSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain)
{
    comb3Dctrl.ss_Px_ctrl.setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
}

void setVxSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain)
{
    comb3Dctrl.ss_Vx_ctrl.setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
}

void setPySSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain)
{
    comb3Dctrl.ss_Py_ctrl.setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
}

void setVySSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain)
{
    comb3Dctrl.ss_Vy_ctrl.setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
}

void setPzSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain)
{
    comb3Dctrl.ss_Pz_ctrl.setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
}

void setYawSSPmt(float _k_p, float _k_i, float _k_d, float _outside_gain)
{
    comb3Dctrl.ss_yaw_ctrl.setSSCv2Pmt(_k_p, _k_i, _k_d, _outside_gain);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 设置控制模式： 设置机器人水下运动模式， 比如沿制定的x方向进行定点运动， 调整航向角等
 *
 * 输入：机器人运动模式 如 Px_CONTROL_MODE， YAW_CONTROL_MODE 等
 *
 * 输出： 无
 */
void setComb3DCtrlMode(int _mode)
{
    comb3Dctrl.debug_mode = _mode;
}
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
/*@腿的形状
 * 功能： 设置腿的形状： 设置水下运动模式腿的形状，该函数对腿型变换进行了复写
 *
 * 输入： 无
 *
 * 输出： 无
 */
void LegShape_H()
{
    vec_ctrl.yawLocomMode_FB();
    cout << " the leg shape is [H] !!!" << endl;
}

// 工型
void LegShape_H_T()
{
    vec_ctrl.yawLocomMode_LR();
    cout << " the leg shape is [H_T] !!!" << endl;
}

// X 形

void LegShape_X()
{
    vec_ctrl.yawLocomMode_X();
    cout << " the leg shape is [X] !!!" << endl;
}

void LegShape_G()
{
    vec_ctrl.shiftMode_G();
    cout << " the leg shape is [H_T] !!!" << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@施加推进力
 * 功能： 对机器人施加推力： 对仿真模型中的机器人施加推进力
 *
 * 输入： 一共7个量， xyz三个方向的力， 绕z轴的转矩， roll，yaw， pitch三个欧拉角
 *
 * 输出： 无
 */
void simMotorForce(float _force_x, float _force_y, float _force_z,
                   float _torque_z,
                   float _roll, float _yaw, float _pitch)
{
    vec_ctrl.motorforce(_force_x, _force_y, _force_z, _torque_z, _roll, _yaw, _pitch);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@腿末端位置 和 角度
 * 功能： 设置腿的位置和角度： 手动设置腿末端位置 和 角度
 *
 * 输入： xyz三个位置 ABC三个角度
 *
 * 输出： 无
 */

// 设置腿部位置
void legSetPosition(float _x, float _y, float _z)
{
    gait_ctrl.legsetposition(_x, _y, _z);
    cout << "the leg position is: "
         << " [x]: " << _x << " [y]: " << _y << " [z]: " << _z << endl;
}

// 设置腿部角度
void legSetAngle(float _A, float _B, float _C)
{
    gait_ctrl.legsetangle(_A, _B, _C);
    cout << "the leg angle is: "
         << " [agl1]: " << _A << " [agl2]: " << _B << " [agl3]: " << _C << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@ 步态
 * 功能： 机器人步态控制： 包括前进，后退，左转， 右转
 *
 * 输入：无
 *
 * 输出： 无
 */
void gaitControl()
{
    if (mode == 1)
    {
        if (dir == FORWARD) // gait
        {
            gait_ctrl.forward4(gait_R);
        }
        if (dir == LEFT)
        {
            gait_ctrl.round_left(gait_R);
        }
        if (dir == RIGHT)
        {
            gait_ctrl.round_right(gait_R);
        }

        if (dir == BACK)
        {
            gait_ctrl.back(gait_R);
        }
    }
    else
    {
        gaitstatu = Stop;
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void gait_stop()
{
    gait_ctrl.gait_stop();
}

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
/*@
 * 功能： ADRC型三维控制： 重写组合 ADRC型 3维控制 变成过程函数 该函数可对机器人进行3维控制
 *
 * 输入：机器人期望与实际的位置，速度，姿态
 *
 * 输出： 无
 */
// hcy:gazebo原程序p使用的
void simCombineADRC_3DCtrl(float _exp_p_x, float _exp_v_x,
                           float _exp_p_y, float _exp_v_y,
                           float _exp_p_z, float _exp_yaw,

                           float _cur_p_x, float _cur_v_x,
                           float _cur_p_y, float _cur_v_y,
                           float _cur_p_z, float _cur_yaw)
{
    // single integ and double integ control are for formation
    if (comb3Dctrl.debug_mode == ALL_CONTROL_MODE_SINGLE_INTEG || comb3Dctrl.debug_mode == ALL_CONTROL_MODE_DOUBLE_INTEG)
    {

        if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                 (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) <= finish_radius)
        {
            // vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0);
            vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
            cout << " ***************************************************in area now !!! " << endl;
            comb3Dctrl.is_judge_direction = false; // 在范围内,不判断方位
        }
        else
        {
            comb3Dctrl.judge_Direction(_exp_yaw, _cur_yaw);
            // cout << "judgeddddddddddddddddddddd" << endl;
            if (((_exp_yaw > -180) && (_exp_yaw < -173)) && ((_cur_yaw > 173) && (_cur_yaw < 180)))
            {
                _exp_yaw = -exp_yaw;
            }
            else if (((_exp_yaw > 173) && (_exp_yaw < 180)) && ((_cur_yaw > -180) && (_cur_yaw < -173)))
            {
                _exp_yaw = -exp_yaw;
            }
            comb3Dctrl.adrcSelectControllerAndOutput(comb3Dctrl.debug_mode, _exp_p_x, _exp_v_x,
                                                     _exp_p_y, _exp_v_y,
                                                     _exp_p_z, _exp_yaw,

                                                     _cur_p_x, _cur_v_x,
                                                     _cur_p_y, _cur_v_y,
                                                     _cur_p_z, _cur_yaw);
        }
    }
    else if (comb3Dctrl.debug_mode == ADJUST_PARAMETERS_MODE)
    { // this is for manual test
        comb3Dctrl.adrcSelectControllerAndOutput(comb3Dctrl.debug_mode, _exp_p_x, _exp_v_x,
                                                 _exp_p_y, _exp_v_y,
                                                 _exp_p_z, _exp_yaw,

                                                 _cur_p_x, _cur_v_x,
                                                 _cur_p_y, _cur_v_y,
                                                 _cur_p_z, _cur_yaw);
    }
    else
    {
        cout << "you are not in all control debug mode !!!" << endl;
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： S面型三维控制： 重写组合 S面型 3维控制 变成过程函数 该函数可对机器人进行3维控制
 *
 * 输入：机器人期望与实际的位置，速度，姿态
 *
 * 输出： 无
 */
void simCombineSS_3DCtrl(float _exp_p_x, float _exp_v_x,
                         float _exp_p_y, float _exp_v_y,
                         float _exp_p_z, float _exp_yaw,

                         float _cur_p_x, float _cur_v_x,
                         float _cur_p_y, float _cur_v_y,
                         float _cur_p_z, float _cur_yaw)
{
    if (comb3Dctrl.debug_mode == ALL_CONTROL_MODE_SINGLE_INTEG || comb3Dctrl.debug_mode == ALL_CONTROL_MODE_DOUBLE_INTEG)
    {
        if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                 (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) <= FINISH_RADIUS)
        {
            vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0);
            cout << " in area now !!! " << endl;
            comb3Dctrl.is_judge_direction = false; // 在范围内,不判断方位
        }
        else
        {
            comb3Dctrl.judge_Direction(_exp_yaw, _cur_yaw);
            // cout << "judgeddddddddddddddddddddd" << endl;
            comb3Dctrl.ssSelectControllerAndOutput(comb3Dctrl.debug_mode, _exp_p_x, _exp_v_x,
                                                   _exp_p_y, _exp_v_y,
                                                   _exp_p_z, _exp_yaw,

                                                   _cur_p_x, _cur_v_x,
                                                   _cur_p_y, _cur_v_y,
                                                   _cur_p_z, _cur_yaw);
        }
    }
    else
    {
        cout << "you are not in all control debug mode !!!" << endl;
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//===============================================================================================================
//   PID
//===============================================================================================================

void setYawPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.vec_ctrl.mcYawPID.setKpPara(_k_p);
    combine3DPidCtrl.vec_ctrl.mcYawPID.setKiPara(_k_i);
    combine3DPidCtrl.vec_ctrl.mcYawPID.setKdPara(_k_d);
    combine3DPidCtrl.vec_ctrl.mcYawPID.enable = true;

    cout << " yaw pid parameters are: "
         << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
}

void setPitchPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.vec_ctrl.mcPitchPID.setKpPara(_k_p);
    combine3DPidCtrl.vec_ctrl.mcPitchPID.setKiPara(_k_i);
    combine3DPidCtrl.vec_ctrl.mcPitchPID.setKdPara(_k_d);
    combine3DPidCtrl.vec_ctrl.mcPitchPID.enable = true;
    cout << "pitch pid parameters are: "
         << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
}

void setXPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.vec_ctrl.mcXPID.setKpPara(_k_p);
    combine3DPidCtrl.vec_ctrl.mcXPID.setKiPara(_k_i);
    combine3DPidCtrl.vec_ctrl.mcXPID.setKdPara(_k_d);
    combine3DPidCtrl.vec_ctrl.mcXPID.enable = true;

    cout << " yaw pid parameters are: "
         << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
}
void setYPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.vec_ctrl.mcYPID.setKpPara(_k_p);
    combine3DPidCtrl.vec_ctrl.mcYPID.setKiPara(_k_i);
    combine3DPidCtrl.vec_ctrl.mcYPID.setKdPara(_k_d);
    combine3DPidCtrl.vec_ctrl.mcYPID.enable = true;

    cout << " yaw pid parameters are: "
         << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
}

void setDepthPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.vec_ctrl.mcDepthPID.setKpPara(_k_p);
    combine3DPidCtrl.vec_ctrl.mcDepthPID.setKiPara(_k_i);
    combine3DPidCtrl.vec_ctrl.mcDepthPID.setKdPara(_k_d);
    combine3DPidCtrl.vec_ctrl.mcDepthPID.enable = true;
    cout << "depth pid parameters are: "
         << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
}

void setSpeedPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.vec_ctrl.mcSpeedPID.setKpPara(_k_p);
    combine3DPidCtrl.vec_ctrl.mcSpeedPID.setKiPara(_k_i);
    combine3DPidCtrl.vec_ctrl.mcSpeedPID.setKdPara(_k_d);
    combine3DPidCtrl.vec_ctrl.mcSpeedPID.enable = true;
    cout << "speed pid parameters are: "
         << "[k_p]: " << _k_p << " [k_i]: " << _k_i << " [k_d]: " << _k_d << endl;
}

void setVelDepPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.setHeaveInnerLoopParams(_k_p, _k_i, _k_d);
}

void setPosDepPIDPmt(float _k_p, float _k_i, float _k_d)
{
    combine3DPidCtrl.setHeaveOuterLoopParams(_k_p, _k_i, _k_d);
}

void simCombinePID_3DCtrl(float _exp_p_x, float _exp_p_y, float _exp_yaw, float _exp_p_z,
                          float _exp_v_x, float _exp_v_y, float _exp_v_z,
                          float _cur_p_x, float _cur_p_y, float _cur_yaw, float _cur_p_z,
                          float _cur_v_x, float _cur_v_y, float _cur_v_z)
{
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   _exp_p_x, _exp_p_y,
                                   _exp_yaw, _exp_p_z,
                                   _cur_p_x, _cur_p_y,
                                   _cur_yaw, _cur_p_z);
}

// 不需要传入期望的yaw，系统会自动计算，用于前进
void PidXYZControl_H(float _exp_p_x, float _exp_p_y, float _exp_p_z, float _cur_p_x, float _cur_p_y, float _cur_p_z, float _cur_yaw)
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    combine3DPidCtrl.updateControl(SHAPE_H,
                                   _exp_p_x, _exp_p_y,
                                   0, _exp_p_z,
                                   _cur_p_x, _cur_p_y,
                                   _cur_yaw, _cur_p_z);
}

void PidXYZControl_X(float _exp_p_x, float _exp_p_y, float _exp_p_z, float _cur_p_x, float _cur_p_y, float _cur_p_z, float _cur_yaw)
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   _exp_p_x, _exp_p_y,
                                   0, _exp_p_z,
                                   _cur_p_x, _cur_p_y,
                                   _cur_yaw, _cur_p_z);
}

void PidXYZControl_X(float _exp_p_x, float _exp_p_y, float _exp_p_z, float _cur_p_x, float _cur_p_y, float _cur_p_z, float _cur_yaw, float cur_v_z)
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    combine3DPidCtrl.updateControl(SHAPE_X,
                                    _exp_p_x, _exp_p_y, 0, _exp_p_z, 0, 0, 0, 
                                    _cur_p_x, _cur_p_y, _cur_yaw, _cur_p_z,
                                    0, 0, cur_v_z);
}

void PidDepVelControl_X(float _exp_v_z, float _cur_v_z)
{
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   0, 0, 0, 0, 0, 0, _exp_v_z,
                                   0, 0, 0, 0, 0, 0, _cur_v_z);
}
void PidDepthYawControl_H(float _exp_p_z, float _exp_v_z, float _exp_yaw, float _cur_p_z, float _cur_v_z, float _cur_yaw)
{
    combine3DPidCtrl.updateControl(SHAPE_H,
                                   0, 0,
                                   _exp_yaw, _exp_p_z,
                                   0, 0,
                                   _cur_yaw, _cur_p_z);
}

// H型航向深度
void PidYawControl_H(float _exp_yaw, float _cur_yaw)
{
    combine3DPidCtrl.updateControl(SHAPE_H,
                                   0, 0,
                                   _exp_yaw, 0,
                                   0, 0,
                                   _cur_yaw, 0);
}

// X型位置控制深度
void PidDepthControl_X(float _exp_p_z, float _cur_p_z)
{
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   0, 0,
                                   0, _exp_p_z,
                                   0, 0,
                                   0, _cur_p_z);
}

//===============================================================================================================
//
//===============================================================================================================

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： ADRC型三维控制： 重写组合 ADRC型 3维控制 变成过程函数 该函数可对机器人进行3维控制
 *
 * 输入：机器人期望与实际的位置，速度，姿态
 *
 * 输出： 无
 */
// hcy:gazebo原程序使用
void simCombineADRC_3DSpeedCtrl(float _exp_v_x, float _exp_v_y,
                                float _exp_v_yaw, float _exp_v_z,

                                float _cur_v_x, float _cur_v_y,
                                float _cur_v_yaw, float _cur_v_z)
{
    /// DEBUG 默认H型
    comb3DSpeedCtrl.adrcSelectSpeedControllerAndOutput(SHAPE_H,
                                                       _exp_v_x, _exp_v_y,
                                                       _exp_v_yaw, _exp_v_z,

                                                       _cur_v_x, _cur_v_y,
                                                       _cur_v_yaw, _cur_v_z);
    four_thrust1 = comb3DSpeedCtrl.speed_vec_ctrl.thrust1;
    four_thrust2 = comb3DSpeedCtrl.speed_vec_ctrl.thrust2;
    four_thrust3 = comb3DSpeedCtrl.speed_vec_ctrl.thrust3;
    four_thrust4 = comb3DSpeedCtrl.speed_vec_ctrl.thrust4;

    cout << "得到的四个推进器的推力为:" << " " << four_thrust1 << " " << four_thrust2 << " " << four_thrust3 << " " << four_thrust4 << endl;

    control_cmd_x = comb3DSpeedCtrl.controller_x_speed;
    control_cmd_yaw = comb3DSpeedCtrl.controller_yaw_speed;
    ///
}

// hcy:gazebo原程序使用
void simCombineADRC_3DSpeedCtrl_v2(float _exp_v_x, float _exp_v_y,
                                   float _exp_v_yaw, float _exp_v_z,
                                   float _exp_yaw,

                                   float _cur_v_x, float _cur_v_y,
                                   float _cur_v_yaw, float _cur_v_z,
                                   float _cur_yaw)
{
    /// DEBUG 默认H型
    comb3DSpeedCtrl.adrcSelectSpeedControllerAndOutput(SHAPE_H,
                                                       _exp_v_x, _exp_v_y,
                                                       _exp_v_yaw, _exp_v_z,
                                                       _exp_yaw,

                                                       _cur_v_x, _cur_v_y,
                                                       _cur_v_yaw, _cur_v_z,
                                                       _cur_yaw);
    four_thrust1 = comb3DSpeedCtrl.speed_vec_ctrl.thrust1;
    four_thrust2 = comb3DSpeedCtrl.speed_vec_ctrl.thrust2;
    four_thrust3 = comb3DSpeedCtrl.speed_vec_ctrl.thrust3;
    four_thrust4 = comb3DSpeedCtrl.speed_vec_ctrl.thrust4;

    cout << "得到的四个推进器的推力为:" << " " << four_thrust1 << " " << four_thrust2 << " " << four_thrust3 << " " << four_thrust4 << endl;

    control_cmd_x = comb3DSpeedCtrl.controller_x_speed;
    control_cmd_yaw = comb3DSpeedCtrl.controller_yaw;
}

void clearADRCSpeedControllerVariables()
{
    comb3DSpeedCtrl.adrc_speed_x_ctrl.clearGlobalVariables();
    comb3DSpeedCtrl.adrc_speed_y_ctrl.clearGlobalVariables();
    comb3DSpeedCtrl.adrc_speed_z_ctrl.clearGlobalVariables();
    comb3DSpeedCtrl.adrc_speed_yaw_ctrl.clearGlobalVariables();
    comb3DSpeedCtrl.adrc_yaw_ctrl.clearGlobalVariables();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * *hcy * * * * * * * * * * * * * * * * * * * * * * */
// hcy:（重要）H型只调节yaw
//  H型航向角调节
void ADRCYawControl_H(float _exp_yaw, float _cur_yaw)
{

    cout << "------ DEBUG Yaw ------ " << endl;
    comb3DPositionCtrl_v2.adrcSelectPositionControllerAndOutput(SHAPE_H, 0, 0, _exp_yaw, 0,
                                                                0, 0, _cur_yaw, 0);
}

// hcy:（重要）X型只调节z
// X型深度调节 //hcy//6.26
/**
 * @description: X型ADRC深度控制
 * @param {float} _exp_z  期望深度
 * @param {float} _cur_z  当前深度
 * @return {*}
 * @author: LJL
 */
void ADRCDepthControl_X(float _exp_z, float _cur_z)
{
    cout << "------ DEBUG Depth ------" << endl;
    comb3DPositionCtrl_v2.adrcSelectPositionControllerAndOutput(SHAPE_X, 0, 0, 0, _exp_z,
                                                                0, 0, 0, _cur_z);
    // comb3DSpeedCtrl.adrcSelectSpeedControllerAndOutput(SHAPE_X,0, 0,0, _exp_z,
    //                                                                                                                                             0, 0,0, _cur_z);
}

// H型位置调节 3维 需要机器人当前航向角
// ljl使用__exp_yaw，用于第一阶段接近目标，测试用不了
// void ADRCPosition3DControl_H_v2(float _exp_x, float _exp_y, float _exp_z,
//                                 float _cur_x, float _cur_y, float _cur_yaw, float _cur_z)
// {
//     cout << "------ H SHAPE DEBUG 3D POSITION ------ " << endl;
//     float __exp_yaw = atan2(_exp_y - _cur_y, _exp_x - _cur_x) * 57.3;
//     comb3DPositionCtrl.adrcSelectPositionControllerAndOutput(SHAPE_H,
//                                                              _exp_x, _exp_y,
//                                                              __exp_yaw, _exp_z,
//                                                              _cur_x, _cur_y,
//                                                              _cur_yaw, _cur_z);
// }

// hcy:新3.（重要）H型x,y,z,yaw总调节2，期望的yaw是形参传进来的。用于docking路径规划
//  H型位置调节 3维 需要机器人当前航向角

// ljl，第二阶段yaw
void ADRCPosition3DControl_H_v2(float _exp_x, float _exp_y, float _exp_yaw, float _exp_z,
                                float _cur_x, float _cur_y, float _cur_yaw, float _cur_z)
{
    cout << "------ H SHAPE DEBUG 3D POSITION ------ " << endl;
    float __exp_yaw = atan2(_exp_y - _cur_y, _exp_x - _cur_x) * 57.3;
    comb3DPositionCtrl_v2.adrcSelectPositionControllerAndOutput(SHAPE_H,
                                                                _exp_x, _exp_y,
                                                                __exp_yaw, _exp_z,

                                                                _cur_x, _cur_y,
                                                                _cur_yaw, _cur_z); // 改过__exp_yaw
}

void ADRCPosition3DControl_X(float _exp_x, float _exp_y, float _exp_z,
                             float _cur_x, float _cur_y, float _cur_yaw, float _cur_z)
{
    cout << "------ X SHAPE DEBUG 3D POSITION ------ " << endl;
    comb3DPositionCtrl_v2.adrcSelectPositionControllerAndOutput(SHAPE_X,
                                                                _exp_x, _exp_y,
                                                                0, _exp_z,
                                                                _cur_x, _cur_y,
                                                                _cur_yaw, _cur_z);
}

// X型位置调节 3维 需要机器人当前航向角
// ljl,测试用不了
// void ADRCPosition3DControl_X(float _exp_x, float _exp_y, float _exp_z,
//                              float _cur_x, float _cur_y, float _cur_yaw, float _cur_z)
// {
//     cout << "------ X SHAPE DEBUG 3D POSITION ------ " << endl;
//    comb3DPositionCtrl.adrcSelectPositionControllerAndOutput(SHAPE_X,
//                                                              _exp_x, _exp_y,
//                                                              0, _exp_z,
//                                                              _cur_x, _cur_y,
//                                                              _cur_yaw, _cur_z);
// }

// hcy:4个参数的z调节
void setDebugPzADRCPmt_v2(float _r_0, float _w_c, float _w_0, float _b_0)
{
    comb3DPositionCtrl_v2.adrc_z_ctrl.setadrcPosPmt(_r_0, _w_c, _w_0, _b_0);
    cout << "[ADRC DEBUG Pz] Parameters are: "
         << "_r_0: " << _r_0 << " _w_c: " << _w_c
         << " _w_0: " << _w_0 << " _b_0: " << _b_0 << endl;
}

// hcy:5个参数的z调节（正在使用）
void setDebugPzADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0)
{
    comb3DPositionCtrl_v2.adrc_z_ctrl.setadrcPosPmt(_r_0, _k_p, _k_d, _w_0, _b_0);
    cout << "[ADRC DEBUG Pz] Parameters are: "
         << "_r_0: " << _r_0 << " _k_p: " << _k_p << " k_d: " << _k_d
         << " _w_0: " << _w_0 << " _b_0: " << _b_0 << endl;
}

// hcy:6个参数的z调节
void setDebugPzADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0)
{
    comb3DPositionCtrl.adrc_z_ctrl.SetadrcPosPmt(_k_p, _k_d, _h, _r, _w, _b_0);
    cout << "[ADRC DEBUG Pz] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d
         << " _h: " << _h << " _r: " << _r
         << " _w: " << _w << " _b_0: " << _b_0 << endl;
}

// hcy:4个参数的yaw调节
void setDebugYawADRCPmt_v2(float _r_0, float _w_c, float _w_0, float _b_0)
{
    comb3DPositionCtrl_v2.adrc_yaw_ctrl.setadrcPosPmt(_r_0, _w_c, _w_0, _b_0);
    cout << "[ADRC DEBUG yaw] Parameters are: "
         << "_r_0: " << _r_0 << " _w_c: " << _w_c
         << " _w_0: " << _w_0 << " _b_0: " << _b_0 << endl;
}

// hcy:5个参数的yaw调节（正在使用）
void setDebugYawADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0)
{
    comb3DPositionCtrl_v2.adrc_yaw_ctrl.setadrcPosPmt(_r_0, _k_p, _k_d, _w_0, _b_0);
    cout << "[ADRC DEBUG yaw] Parameters are: "
         << "_r_0: " << _r_0 << " _k_p: " << _k_p << " k_d: " << _k_d
         << " _w_0: " << _w_0 << " _b_0: " << _b_0 << endl;
}

// hcy:6个参数的yaw调节
void setDebugYawADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0)
{
    comb3DPositionCtrl.adrc_yaw_ctrl.SetadrcPosPmt(_k_p, _k_d, _h, _r, _w, _b_0);
    cout << "[ADRC DEBUG yaw] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d
         << " _h: " << _h << " _r: " << _r
         << " _w: " << _w << " _b_0: " << _b_0 << endl;
}

// hcy:5个参数的x调节
void setDebugPxADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0)
{
    comb3DPositionCtrl_v2.adrc_x_ctrl.setadrcPosPmt(_r_0, _k_p, _k_d, _w_0, _b_0);
    cout << "[ADRC DEBUG Px] Parameters are: "
         << "_r_0: " << _r_0 << " _k_p: " << _k_p << " k_d: " << _k_d
         << " _w_0: " << _w_0 << " _b_0: " << _b_0 << endl;
}

// hcy:6个参数的x调节（正在使用）
void setDebugPxADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0)
{
    comb3DPositionCtrl.adrc_x_ctrl.SetadrcPosPmt(_k_p, _k_d, _h, _r, _w, _b_0);
    cout << "[ADRC DEBUG Px] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d
         << " _h: " << _h << " _r: " << _r
         << " _w: " << _w << " _b_0: " << _b_0 << endl;
}

// hcy:5个参数的y调节
void setDebugPyADRCPmt_v2(float _r_0, float _k_p, float _k_d, float _w_0, float _b_0)
{
    comb3DPositionCtrl_v2.adrc_y_ctrl.setadrcPosPmt(_r_0, _k_p, _k_d, _w_0, _b_0);
    cout << "[ADRC DEBUG Py] Parameters are: "
         << "_r_0: " << _r_0 << " _k_p: " << _k_p << " k_d: " << _k_d
         << " _w_0: " << _w_0 << " _b_0: " << _b_0 << endl;
}

// hcy:6个参数的y调节（正在使用）
void setDebugPyADRCPmt(float _k_p, float _k_d, float _h, float _r, float _w, float _b_0)
{
    comb3DPositionCtrl.adrc_y_ctrl.SetadrcPosPmt(_k_p, _k_d, _h, _r, _w, _b_0);
    cout << "[ADRC DEBUG Py] Parameters are: "
         << "_k_p: " << _k_p << " _k_d: " << _k_d
         << " _h: " << _h << " _r: " << _r
         << " _w: " << _w << " _b_0: " << _b_0 << endl;
}
// Su Yizhou:抓取模式水下运动

void ADRCPosition3DControl_G(float _exp_x, float _exp_y, float _exp_yaw, float _exp_z,
                             float _cur_x, float _cur_y, float _cur_yaw, float _cur_z)
{

    float __exp_yaw = atan2(_exp_y - _cur_y, _exp_x - _cur_x) * 57.3 - 40;
    if (__exp_yaw < -180)
        __exp_yaw = __exp_yaw + 360;
    comb3DPositionCtrl_v2.adrcSelectPositionControllerAndOutput(SHAPE_G,
                                                                _exp_x, _exp_y,
                                                                __exp_yaw, _exp_z,

                                                                _cur_x, _cur_y,
                                                                _cur_yaw, _cur_z);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// void attitudeMPCcontrol(Attitude e_attitude, Attitude c_attitude)
// {
//     float fz1 = 0, fz2 = 0, fz3 = 0, fz4 = 0;
//     float reference = e_attitude.yaw;
//     printf("e_yaw:%.4f \n", reference);
//     float c_yaw = c_attitude.yaw;
//     printf("c_yaw:%.4f \n", c_yaw);
//     float fx1, fx2, fx3, fx4; //horizontal force
//     float torque = mcYawMPC.QpSolverset(Su, Sx, reference, A, c_yaw);
//     printf("torque:%.4f \n", torque);
//     if (reference - c_yaw > 180 || reference - c_yaw < -180)
//         torque = -torque;
//     fx1 = 1.3 - 0.35 * torque;
//     fx4 = 1.3 + 0.35 * torque;
//     float sv = 0.5; //向前走
//     fx2 = fx4 + sv;
//     fx3 = fx1 + sv;
//     printf("hxhfx1:%.4f     hxhfx2:%.4f     fx3:%.4f     fx4:%.4f  \n", fx1, fx2, fx3, fx4);

//     float force_x = fx1 + fx4 - fx2 - fx3;
//     float force_z = fz1 + fz4 + fz2 + fz3;
//     float torque_z = (fx2 + fx4 - fx1 - fx3) * 0.02758;

//     vec_ctrl.motorforce(force_x, 0, force_z, torque_z, c_attitude.roll, c_attitude.yaw, c_attitude.pitch);
// }
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// MPC trajectory_tracking1
// void f_tracking1(Position c_position, Attitude c_attitude)
//{
//     if(comb3Dctrl.debug_mode==MPC_MODE)
//     {
//   float fz1=0,fz2=0,fz3=0,fz4=0;
//   VectorXf x_real_current=VectorXf::Zero(5);
//   VectorXf x_real_last=tracking.getstate();
//   cout<<"x_real_last:"<<x_real_last(0)<<";"<<x_real_last(1)<<";"<<x_real_last(2)<<";"<<x_real_last(3)<<";"<<x_real_last(4)<<endl;
//   x_real_current(0)=c_position.x;
//   x_real_current(1)=c_position.y;
//   x_real_current(2)=c_attitude.yaw;
//   cout<<"x_real_yaw"<<x_real_current(2)<<endl;
//  // cout<<"samplingtime"<<samplingtime<<endl;
//   x_real_current(3)=sqrt((c_position.x-x_real_last(0))*(c_position.x-x_real_last(0))+(c_position.y-x_real_last(1))*(c_position.y-x_real_last(1)))/0.05;
//   x_real_current(4)=(c_attitude.yaw-x_real_last(2))/0.05;
//   Vector2f force=tracking.Generate_realTao( Reftrajectory, tao, x_real_current);
//   tracking.setstate(x_real_current);
//   printf("force0:%.4f     force1:%.4f  \n", force(0),force(1));

//        float force_x=force(0);
//        float force_z=fz1+fz4+fz2+fz3;
//        float torque_z=force(1);
//        cout<<"have get the control input"<<endl;

//        vec_ctrl.motorforce(force_x, 0,force_z,torque_z,c_attitude.roll, c_attitude.yaw, c_attitude.pitch);
//    }
//}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// MPC constant velocity trajectory_tracking
// void f_tracking3(Position c_position, Attitude c_attitude)
//{
//     if(comb3Dctrl.debug_mode==MPC_MODE)
//     {

//  float fz1=0,fz2=0,fz3=0,fz4=0;
//  VectorXf x_real_current=VectorXf::Zero(4);
//  VectorXf x_real_last=tracking3.getstate();
//  x_real_current(0)=c_position.x;
//  x_real_current(1)=c_position.y;
//  x_real_current(2)=c_attitude.yaw;
//  x_real_current(3)=(c_attitude.yaw-x_real_last(2))/0.05;
////  float force=tracking3.Generate_realTao( Reftrajectory, tao1, x_real_current, 0.5);
////  tracking3.setstate(x_real_current);
//  float force=0;

//   printf("force:%.4f \n", force);
//   printf("positionx:%.4f  positiony:%.4f yaw:%.4f\n yawvelocity:%.4f", x_real_current(0),x_real_current(1),x_real_current(2),x_real_current(3) );
//   float  fx1, fx2, fx3, fx4;//horizontal force

//  /*fx1=1.1-0.25*force/0.028325;// max of the force 2.3N, 1/4*2.3=0.575N
//  fx4=1.1+0.25*force/0.028325;//range in 0.1 to 2.1
//  float sv=0.5;
//  float compen=0.4;
//  fx2=fx4+sv;
//  fx3=fx1+compen;*/                //according to above the max fx is 2.375N， the minimum fx is 0.725N

//  /*fx1=1.1-0.25*force;// max of the force 2.3N, 1/4*2.3=0.575N
//  fx4=1.1+0.25*force;//range in 0.1 to 2.1
//  float sv=0.5;
//  fx2=fx4+sv;
//  fx3=fx1+sv;*/                //according to above the max fx is 2.375N， the minimum fx is 0.725N

// if(force>=0)
// {
//  fx1=0.5*force/0.028325+0.4;
//  fx2=0.9;
//  fx3=0.5*force/0.028325+0.4;
//  fx4=0.4;
// }
//  if(force<0)
// {
//  fx2=-0.5*force/0.028325+0.4;
//  fx1=0.4;
//  fx4=-0.5*force/0.028325+0.4;
//  fx3=0.9;
// }

//  printf("hxhfx1:%.4f     hxhfx2:%.4f     fx3:%.4f     fx4:%.4f  \n", fx1,fx2,fx3,fx4);

//  if (fx1>2.5) fx1=2.500;
//  if (fx2>2.5) fx2=2.500;
//  if (fx3>2.5) fx3=2.500;
//  if (fx1>2.5) fx1=2.500;

//       float force_x=fx1+fx4-fx2-fx3;
//       float  force_z=fz1+fz4+fz2+fz3;
//        float torque_z=(fx2+fx4-fx1-fx3)*0.02758;

//        vec_ctrl.motorforce(force_x, 0,force_z,torque_z,c_attitude.roll, c_attitude.yaw, c_attitude.pitch);

//}
//}
void yawidentification()
{
    float f1, f2, f3, f4;
    float fz1 = 0, fz2 = 0, fz3 = 0, fz4 = 0;
    static int num = 1;
    float time = num * 0.05;
    // add sine signal
    // f1=(sin(0.05*time)+1);
    // f2=0.4;
    // add square signal
    /* if num>600
  {
    f1=1.0;
    f2=0.4;
  }
  elseif 600<num<1200
  {
   f1=1.5;
   f2=0.4;
  }
 else num>1200
{
    f1=1.0;
    f2=0.4;

}*/
    // add constant signal
    f1 = 1.2;
    f2 = 0.4;
    f3 = f1 + 0.1;
    f4 = f2;
    printf("fx1:%.4f     fx2:%.4f     fx3:%.4f     fx4:%.4f  \n", f1, f2, f3, f4);
    float force_x = f1 + f4 - f2 - f3;
    float force_z = fz1 + fz4 + fz2 + fz3;
    float torque_z = (f2 + f4 - f1 - f3) * 0.02758; // clockwise>0

    vec_ctrl.motorforce(force_x, 0, force_z, torque_z, c_attitude.roll, c_attitude.yaw, c_attitude.pitch);
    num = num + 1;
}
void surgeidentification()
{
    static int num1 = 1;
    float time = num1 * 0.05;
    float f1 = (sin(0.1 * time) + 1);
    float f2 = 0.4;
    float f3 = f2;
    float f4 = f1;
    float fz1 = 0, fz2 = 0, fz3 = 0, fz4 = 0;
    printf("fx1:%.4f     fx2:%.4f     fx3:%.4f     fx4:%.4f  \n", f1, f2, f3, f4);
    float force_x = f1 + f4 - f2 - f3;
    float force_z = fz1 + fz4 + fz2 + fz3;
    float torque_z = (f2 + f4 - f1 - f3) * 0.02758;

    vec_ctrl.motorforce(force_x, 0, force_z, torque_z, c_attitude.roll, c_attitude.yaw, c_attitude.pitch);
    num1 = num1 + 1;
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void update_iter()
{
    if (iter_num < num_trackingpoints)
    {
        iter_num = iter_num + 1;
    }
}

/*************************************基于OSQP的MPC******************************************************/
/**
 * MPC trajectory tracking based on the OSQP and qpoases library
 *
 *
 *
 * */
//         force_2 = sqrt(abs(force_2));
//         force_3 = sqrt(abs(force_3));
//         force_4 = sqrt(abs(force_4));
//         MPC_vec_ctrl.thrusterforce2speed(force_1, force_2, force_3, force_4);
//     }
// }
void f_mpcTracking(Position c_position, Attitude c_attitude, Speed c_speed)
{
    if (ref_trajectory.rows() <= 0)
    {
        //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$不同平台上这个参考轨迹不一样$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        ref_trajectory = robot1_reference_trajectory;
        vrRobot_ref_trajectory = virtureRobot_reference_trajectory;
        // leader_ref_trajectory=robot1_reference_trajectory;
    }

    num_trackingpoints = ref_trajectory.rows();
    double fz1 = 0, fz2 = 0, fz3 = 0, fz4 = 0;
    cout << "reference point num is :" << ref_trajectory.rows() << endl;
    vs_reference_PositionX = (Robot1_c_position.x + Robot2_c_position.x + Robot3_c_position.x) / 3;
    cout << "robot1 X:" << Robot1_c_position.x << endl;
    cout << "robot2 X:" << Robot2_c_position.x << endl;
    cout << "robot3 X:" << Robot3_c_position.x << endl;
    cout << "virture position X:" << vs_reference_PositionX << "   " << vs_reference_PositionY << endl;
    //  cout<<"*****多机的中心位置:****"<<vs_reference_PositionX<<"     "<<vs_reference_PositionY<<endl;
    // ready work
    if (comb3Dctrl.debug_mode == MPC_MODE)
    {
        if (ref_trajectory.rows() > 0)
        {
            /**************************获取机器人当前的状态信息**********************************/
            // update_iter();
            cout << "-----------------------This is the " << iter_num << "------------------------" << endl;

            VectorXd x_real_current = VectorXd::Zero(X_num);
            cout << "Please update the currentState of the robot" << endl;
            x_real_current(0) = c_position.x;
            x_real_current(1) = c_position.y;
            x_real_current(2) = c_attitude.yaw * PI / 180;
            x_real_current(3) = sqrt(c_speed.x * c_speed.x + c_speed.y * c_speed.y);
            x_real_current(4) = c_speed.yaw;

            /**********************确定当前的参考点****************************************/
            switch (SINGLE_ROBOT)
            {
            case SINGLE_ROBOT:
            {
                cout << "当前在单机模式" << endl;
                //  if(iter_num==1)
                //  {
                vector<float> ref_point;
                for (int i = 0; i < ref_trajectory.rows(); i++)
                {
                    ref_point.push_back(ref_trajectory(i, 0));
                }
                ref_point.push_back(x_real_current(0));
                sort(ref_point.begin(), ref_point.end());
                vector<float>::iterator it = find(ref_point.begin(), ref_point.end(), x_real_current(0));
                int index = distance(ref_point.begin(), it);
                cout << "当前的参考点为:" << index << endl;
                iter_num = index + 1;
                //   }
            };
            break;
            case MULTI_ROBOT:
            {
                cout << "当前在多机模式" << endl;

                //  if(iter_num==1)
                //  {
                vector<float> ref_point;
                for (int i = 0; i < vrRobot_ref_trajectory.rows(); i++)
                {
                    ref_point.push_back(vrRobot_ref_trajectory(i, 0));
                }
                ref_point.push_back(vs_reference_PositionX);
                sort(ref_point.begin(), ref_point.end());
                vector<float>::iterator it = find(ref_point.begin(), ref_point.end(), vs_reference_PositionX);
                int index = distance(ref_point.begin(), it);
                cout << "当前的参考点为:" << index << endl;
                iter_num = index + 1;
                //  }
                //  if(iter_num==1)
                //  {
                // vector<float> ref_point;
                // for (int i = 0; i < leader_ref_trajectory.rows(); i++)
                // {
                //     ref_point.push_back(leader_ref_trajectory(i, 0));
                // }
                // ref_point.push_back(Robot1_c_position.x);
                // sort(ref_point.begin(), ref_point.end());
                // vector<float>::iterator it = find(ref_point.begin(), ref_point.end(), Robot1_c_position.x);
                // int index = distance(ref_point.begin(), it);
                // cout << "当前的参考点为:" << index << endl;
                // iter_num = index + 1;
                //  }
            };
            break;
            default:
            {
                cout << " please set [robot mode] correctly " << endl;
            };
            break;
            }

            /*****************更新MPC_tracking中的机器人状态信息和参考点的信息*******************/
            MPC_tracking.Update(iter_num, ref_trajectory, MPC_real_force, x_real_current);
            // cout << " update of the currentState of the robot has finished" << endl;

            /*************************计算x_piao和kesi*************************************/
            VectorXd x_refer = ref_trajectory.block(iter_num - 1, 0, 1, X_num).transpose();
            VectorXd x_piao = x_real_current - x_refer;
            if (x_piao(2) < -PI)
            {
                x_piao(2) = x_piao(2) + 2 * PI;
            }
            if (x_piao(2) > PI)
            {
                x_piao(2) = x_piao(2) + 2 * PI;
            }

            VectorXd kesi(X_num + U_num);
            kesi.head(X_num) = x_piao;
            kesi.segment(X_num, U_num) = MPC_real_force;

            // 根据想_piao调节参数Q

            // 如果机器人从参考轨迹右侧出发
            //  if (x_piao(1)<-0.1 && abs(x_piao(2))<0.1)
            //  {
            //      q22=2*Init_q22;
            //      q33=Init_q33;
            //      q44=Init_q44;
            //  }
            //  else if (x_piao(1)>=-0.1 &&x_piao(1)<0)
            //  {
            //      q22=0.5*Init_q22;
            //      q33=Init_q33;
            //      cout<<"进入关注模式"<<endl;
            //      q44=Init_q44;
            //  }

            // else if(x_piao(1)<0.1 && x_piao(1)>0.05 && abs(x_piao(2))<0.1)
            // {
            //     q22=Init_q22;
            //     q33=Init_q33;
            //     q44=0.2*Init_q44;

            // }
            //   else if( x_piao(1)<0.2 &&x_piao(1)>=0.1 && abs(x_piao(2))<0.1)
            // {
            //     q22=Init_q22;
            //     q33=Init_q33;
            //     q44=0.5*Init_q44;
            // }
            // else if (x_piao(1)>=0.2 && abs(x_piao(2))<0.05)
            // {
            //     q33=10*Init_q33;
            //     q22=Init_q22;
            //     q44=Init_q44;
            //     cout<<"coming into the range 0 ~0.005"<<endl;
            // }

            // else
            // {
            //    q22=Init_q22;
            //    q33=Init_q33;
            //    q44=Init_q44;
            // }
            // 如果机器人从参考轨迹右侧出发
            if (x_piao(1) > 0.1)
            {
                q22 = Init_q22;
                q33 = Init_q33 * 0.1;
            }
            else if (x_piao(1) < -0.1 && x_piao(2) > 0.01)
            {
                q22 = 2 * Init_q22;
                q33 = 0.1 * Init_q33;
                q44 = Init_q44 * 0.1;
            }
            else if (x_piao(1) > -0.1 && x_piao(1) < 0 && x_piao(2) > -0.1 && x_piao(2) < 0)
            {
                q22 = 0.5 * Init_q22;
                q33 = Init_q33;
                cout << "进入关注模式" << endl;
                q44 = Init_q44;
            }
            else if (x_piao(1) < -0.1 && x_piao(2) < -0.1)
            {
                q22 = 2 * Init_q22;
                q33 = Init_q33;
                cout << "进入关注模式" << endl;
                q44 = Init_q44;
            }

            else
            {

                q22 = Init_q22;
                q33 = Init_q33;
                q44 = Init_q44;
            }

            // cout<<" Computation of the X_piao has finished "<<endl;
            cout << "************ error = real - ref **************" << endl;
            cout << "X:" << x_piao(0) << endl;
            cout << "Y:" << x_piao(1) << endl;
            cout << "Yaw:" << x_piao(2) << endl;
            cout << "x_speed:" << x_piao(3) << endl;
            cout << "yaw_speed:" << x_piao(4) << endl;
            cout << "************     ref     **************" << endl;
            cout << "X:" << x_refer(0) << endl;
            cout << "Y:" << x_refer(1) << endl;
            cout << "Yaw:" << x_refer(2) << endl;
            cout << "x_speed:" << x_refer(3) << endl;
            cout << "yaw_speed:" << x_refer(4) << endl;

            /*******************调节参数矩阵Q**************************************/

            MatrixXd Q = MPC_tracking.DebugQMatrix(q11, q22, q33, q44, q55); // 主要调节这三个参数

            /*******************更新控制输入control_cmd********************************/
            VectorXd real_force; // 局部变量,每次循环都会更新

            if (Linear_mode == 1)
            {
                if (!delta_mode)
                {
                    real_force = MPC_tracking.Update_Control_cmd(Q, Ro, x_piao, 1); // 返回的是二次优化的最优解
                    real_force(0) = real_force(0) + ref_controlInput(iter_num - 1, 0);
                    real_force(1) = real_force(1) + ref_controlInput(iter_num - 1, 1);
                }
                // VectorXd real_force =MPC_tracking.Update2_Control_cmd( Q, Ro,iter_num,ref_trajectory,1);
                if (delta_mode)
                {
                    cout << "kesi is:" << kesi << endl;
                    real_force = MPC_tracking.Update_Control_cmd(Q, Ro, kesi, 1); // 返回的是二次优化最优解
                    cout << "MPC_real_force:" << MPC_real_force(0) << "*****" << MPC_real_force(1) << endl;
                    real_force(0) = MPC_real_force(0) + real_force(0);
                    real_force(1) = MPC_real_force(1) + real_force(1); // du+u
                }
            }
            if (Linear_mode == 2)
            {

                if (!delta_mode)
                {
                    real_force = MPC_tracking.Update2_Control_cmd(Q, Ro, iter_num, ref_trajectory, 1); // 1 osqp 2 qpoase
                }
                // VectorXd real_force =MPC_tracking.Update2_Control_cmd( Q, Ro,iter_num,ref_trajectory,1);
                if (delta_mode)
                {
                    real_force = MPC_tracking.Update2_Control_cmd(Q, Ro, iter_num, ref_trajectory, 1);
                    cout << "MPC_real_force:" << MPC_real_force(0) << "*****" << MPC_real_force(1) << endl;
                    real_force(0) = MPC_real_force(0) + real_force(0);
                    real_force(1) = MPC_real_force(1) + real_force(1);
                }
            }
            MPC_real_force = real_force;
            cout << "**************************************" << endl;
            cout << "control_cmd of surge:" << real_force(0) << endl;
            cout << "control_cmd of yaw:" << real_force(1) << endl;
            cout << "**************************************" << endl;
            // printf("force0:%.4d     force1:%.4d  \n", real_force(0),real_force(1));
            // cout<<"***********force1:"<<ref_controlInput(iter_num-1,0)<<"    force2:"<<ref_controlInput(iter_num-1,1)<<endl;
            save_debugdata.write_data(iter_num, x_piao, real_force);
            save_debugdata1.write_data(iter_num, x_real_current);
            // put the force into the system -----method1
            // double force_x=real_force(0);
            // double force_z=fz1+fz4+fz2+fz3;
            // double torque_z=real_force(1);
            // cout<<"have get the control input"<<endl;
            //  vec_ctrl.motorforce(force_x, 0,force_z,torque_z,c_attitude.roll, c_attitude.yaw, c_attitude.pitch);

            /**********************解算控制输入到四个推进器上*****************************/

            double force_1 = 0, force_2 = 0, force_3 = 0, force_4 = 0;
            if (real_force(0) > 0) // calculate f_v
            {
                force_2 = real_force(0) / 2;
                force_3 = real_force(0) / 2;
            }
            else
            {
                force_1 = -real_force(0) / 2;
                force_4 = -real_force(0) / 2;
            }

            if (real_force(1) > 0) // calculate f_yaw
            {

                force_1 = force_1 + real_force(1) / 2;
                force_3 = force_3 + real_force(1) / 2;
            }
            else
            {
                force_2 = force_2 - real_force(1) / 2;
                force_4 = force_4 - real_force(1) / 2;
            }

            force_1 = sqrt(abs(force_1)); //???????
            force_2 = sqrt(abs(force_2));
            force_3 = sqrt(abs(force_3));
            force_4 = sqrt(abs(force_4));
            cout << "force_1:" << force_1 << "     "
                 << "force_2:" << force_2 << "     "
                 << "force_3:" << force_3 << "     "
                 << "force_4:" << force_4 << endl;

            MPC_vec_ctrl.thrusterforce2speed(force_1 / 1.5, force_2 / 1.5, force_3 / 1.5, force_4 / 1.5);
            // MPC_vec_ctrl.thrusterforce2speed(force_1 , force_2, force_3 , force_4 );
            if (iter_num == num_trackingpoints - 3)
            {
                setComb3DCtrlMode(0);
                vec_ctrl.thrusterforce2speed(0, 0, 0, 0); // 推进器停转
                legSetAngle(0, 0, 0);
            }
        }
    }
}
