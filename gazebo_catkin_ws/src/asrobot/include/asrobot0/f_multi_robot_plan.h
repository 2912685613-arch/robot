#ifndef F_MULTI_ROBOT_PLAN_H
#define F_MULTI_ROBOT_PLAN_H

/*********************     ROS组件和消息头文件          ***********************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <asrobot/status.h>
#include "asrobot/pwm.h"
#include "asrobot/camera_data.h"
#include "asrobot/path_3d.h"
#include "asrobot/path_based_threeSpine.h"
#include "asrobot/mul_ref_trajectory.h"

#include "asrobot/mc.h"
#include "asrobot/multi_robot8.h"
#include "asrobot/quadrotor2robot.h"
#include "asrobot/camera82robot.h"
#include "asrobot/achievedWaypoint.h"
#include "asrobot/status.h"

//hcy
#include "asrobot/quadrotor2robot.h"
//#include "quad2robot/quadrotor2robot.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include <sensor_msgs/image_encodings.h>
#include <gazebo_msgs/ModelStates.h>
/*********************     -----------------          ***********************/

/*********************         C++标准库头文件           ***********************/
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <time.h>
#include <string>
#include <vector>
/*********************     -----------------          ***********************/


/*********************         图像处理头文件             ***********************/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//hcy
#include <opencv2/video/tracking.hpp>
//#include <opencv2/tracking.hpp>

#include <opencv2/opencv.hpp>

//hcy
//#include <opencv2/tracking/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


/*********************         矩阵运算库头文件             ***********************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
/*********************        ---------------          ***********************/


/*********************         自定义库头文件             ***********************/
#include "visualservoing.h"
#include "c_multi_agent_algorithm.h"
#include "c_filter_group.h"
#include "c_save_data.h"

#include "c_docking.h"//hcy
/*********************        ---------------          ***********************/

using namespace cv;
using namespace std;
using namespace Eigen;

using namespace MultiAgentAlgorithm;
using namespace FilterGroup;
using namespace SaveData;

using namespace docking;//hcy


#define     SELF_ID                 2
#define     DURATION                1  // 定时器间隔时间
#define     CONV_SPEED              0   // 一致性收敛速度
#define     NUM_AGENT               3   // 机器人数量
#define     DIMENSION               3   // 位置维度

#define     SINGLE_MODE             1   // 单机模式
#define     FORMATION_MODE          2   // 编队模式
#define     DEBUG_MODE              3
#define     SINGLE_PATH_TRACK       4


#define SINGLE_ROBOT_DEBUG 5//hcy
#define DOCKING_MODE 6//hcy
#define TRACKING_MODE 7 //hcy

#define TRACKING_ARUCO_MODE 8 // LJL

// 编队状态切换标志位
const int FORMATION_INIT = 1;
const int FORMATION_MAINTAIN = 2;
const int FORMATION_RUNNING = 3;

/*######################################################################################*/
/*@ 总目录
 *  按" ctrl + F " 对应查找
 *
 *    1. 多机路径规划变量目录
 *    2. 多机编队函数目录
 *    3. 保存数据函数目录
 *
 *
/*######################################################################################*/




/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  多机路径规划变量目录
 *   按" ctrl + F " 对应查找
 *
 *     1. 基本变量
 *     2. 多机编队需要用到的矩阵
 *     3. gazebo输出的深度计和陀螺仪信息
 *     4. 机器人自身位置与速度
 *     5. 编队所有机器人自身信息和无人机自身位置信息
 *     6. 多机编队算法
 *     7. 地图与路径规划相关变量
 *     8. 静态轨迹追踪变量
 *     9. 动态轨迹追踪变量
 *    10. 保存数据变量
 *    11. 滤波器
 */


/******************************   基本变量   **********************************/
extern const int self_id;             // 机器人自身id号
extern const int iDecisionLoopRate;         // 主函数循环频率
extern bool is_initial_multi_msg;     // 初始化多机信息标志位
/******************************   ------  **********************************/


/******************************   多机编队需要用到的矩阵   **********************************/
extern MatrixXf all_robots_cur_pos;    // 所有智能体当前位置
extern MatrixXf all_robots_tar_pos;    // 所有机器人目标位置
extern VectorXf this_robot_exp_pos;    // 本台机器人的目标位置
extern MatrixXf minus_laplace;         // 负拉普拉斯矩阵

extern MatrixXf laplace_n;             // n维拉普拉斯矩阵
extern MatrixXf laplace_n_1;           // 带虚拟leader的拉普拉斯矩阵 n+1维

extern VectorXf ref_pos;               // 参考点的位置
extern VectorXf ref_speed;             // 参考点的速度
extern MatrixXf robots_devia_pos;      // 所有机器人相对于参考点的偏差
extern MatrixXf robots_devia_speed;    // 偏差速度,变形会用到
extern MatrixXf robots_cur_pos;        // 所有机器人的当前位置
extern MatrixXf robots_cur_speed;      // 所有机器人当前的速度
extern VectorXf this_robot_exp_speed;  // 这台机器人期望达到的速度


/******************************   -----------------  **********************************/


/******************************   gazebo输出的深度计和陀螺仪信息   **********************************/
extern const float pressurePerMeter;
extern const float pressureInAir;      // 深度计初始值为101kpa
extern float depth;
extern double pitch;
extern double roll;
extern double yaw;
extern float yaw_velocity;
extern float x_linear_acceleration;
extern float y_linear_acceleration;
/******************************   --------------------  **********************************/



/******************************   机器人自身位置与速度   **********************************/
extern float X_position;
extern float Y_position;
extern float Z_position;
extern int epx[4];
extern int epy[4];
extern int X_speed;
extern int Y_speed;
extern int Z_speed;
/******************************   --------------------  **********************************/



/******************************   编队所有机器人自身信息和无人机自身位置信息  **********************************/
extern MatrixXf robotsInfofromQuadrotor;  // 前两行是位置x，y，后两行是速度x，y
extern VectorXf quadInfo;
/******************************   --------------------  **********************************/




/******************************   多机编队算法   **********************************/
extern VirtualStructure vsRobots;       // 三角形虚拟结构
extern APF apf_agent;                   // 人工势场防撞
extern C_DistrInfoConcencus con_agent;  // 分布式一致性修正

extern C_SingleIntegConcensusFormation singint_cons; // 采用集中式,输出速度的一致性算法
extern float triangleShapeSideLength;   // 三角形编队边长
extern float lineShapeLength;           // 直线型线长
extern int formationShape;              // 编队形状标志
extern float delta_distance;            // 与正规形状的误差阈值
extern bool is_initial_consensusControl; // 一致性初始化标志位
extern bool is_shape_built;

/******************************   --------------  **********************************/


/******************************   地图与路径规划相关变量  **********************************/
extern int imgScale;
extern Eigen::MatrixXf path_3d;     // 3d路径点
extern cv::Mat revMapImage;         // 地图
extern cv::Mat convert2worldMap;    // This need to be revised.
extern float vs_reference_x;
extern float vs_reference_y;
extern float vs_curYaw;             // 虚拟结构初始化的参考点
extern VectorXf achievedWaypointVec; // 3台机器人到达的路径点的数量

/******************************   -----------  **********************************/


/******************************   静态轨迹追踪变量  **********************************/
// 暂时还没有使用静态轨迹追踪
/******************************   -------------  **********************************/



/******************************   动态轨迹追踪变量  **********************************/
extern MatrixXf virtual_leader_new_3d_path; // 第(k)次路径
extern MatrixXf virtual_leader_old_3d_path; // (k-1)次路径
extern VectorXf virtual_leader_curpoint;  // used in multi-robot
extern VectorXf path_3d_exp_point;
extern int point_count;
extern float switch_next_point_radius;
/******************************   ----------------------  **********************************/


extern Eigen::MatrixXf path_points;  //单机跟踪路径
extern Vector2f target_point; //目标位置
extern bool init_flag;//单机初始化标志位


/******************************    保存数据变量    **********************************/
extern C_SaveData save_data;
/******************************   -----------------  **********************************/


/******************************   滤波器   **********************************/
extern C_KalmanFilter_v2 KF_v2_robot1_z; // 5台机器人各自的 z 坐标滤波器
extern C_KalmanFilter_v2 KF_v2_robot2_z;
extern C_KalmanFilter_v2 KF_v2_robot3_z;

/******************************   ---------  **********************************/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
extern C_KalmanFilter_WhiteNoise yaw_vel_filter;
extern float yaw_vel_filter_output;


///hcy
/******************************   双目返回的位置姿态数据   **********************************/
extern float cam_pos_x, cam_pos_y, cam_pos_z;
extern float cam_roll, cam_pitch, cam_yaw;

/******************************   ---------  **********************************/

// 新定义的aruco码到机体位姿
extern float aruco_pos_x, aruco_pos_y, aruco_pos_z;
extern float aruco_roll, aruco_pitch, aruco_yaw;

extern MatrixXf docking_plan_path_points;
extern VectorXf docking_exp_robot_pos;
extern VectorXf docking_cur_robot_pos;
extern int docking_point_count;
extern C_Docking doc;
extern C_Docking_v2 doc_v2;//hcy
extern bool docking_flag;
extern bool tracking_flag; //hcy
extern bool tracking_aruco_flag; // ljl
///

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  多机编队函数目录
 *   按" ctrl + F " 对应查找
 *
 *     1. 多机编队初始化   多机编队初始化v2
 *     2. 编队路径规划   检查编队形状   输出编队期望位置    建立队形
 *     3. 静态轨迹追踪
 *     4. 动态轨迹追踪初始化
 *     5. 单机模式下机器人跟踪动态路径
 *     6. 编队模式下机器人跟踪动态路径    编队模式下追踪动态路径
 *     7. 初始化并跟踪路径   初始化并跟踪路径v2
 *
 */
/******************************************************************************************/
/*                                        获取多机的参考轨迹                                     */
/******************************************************************************************/
extern referenceTrajectory mul_ref_trajectory_objective;
extern Eigen::MatrixXf trajectory_threeSpline;
extern MatrixXf robot1_ref_trajectory;
extern MatrixXf robot2_ref_trajectory;
extern MatrixXf robot3_ref_trajectory;
extern MatrixXf virture_ref_trajectory;
void update_mul_ref_Info(MatrixXf ref, int shape);




/******************************************************************************************/
/*                                        老版本编队规划                                     */
/******************************************************************************************/
void RobotsFormationInit(int _dimension, int _num_robots);   // 多机编队初始化
VectorXf RobotsFormationPlan(VectorXf virtual_leader_pos,float i_con,float i_apfi,float s);// 编队路径规划
void pathTrackingInit(); // 动态轨迹追踪初始化
void dynamicPathTracking();// 单机模式下机器人跟踪动态路径
void dynamicMultiPathPoints();// 编队模式下机器人跟踪动态路径
void initRobotPosAndPlanPath(int _mode);  // 初始化并跟踪路径 （适用于单机或多机）
void UpdateTargetPoint(); //单机跟踪路径点更新
/******************************************************************************************/


/******************************************************************************************/
/*                                        新版本编队规划                                     */
/******************************************************************************************/
void formationControlInit(int _dimension, int _num_robots); // 多机编队初始化v2
bool checkFormationShapeBuilding(int i_formationShape);// 检查编队形状
VectorXf formationControlPosition(VectorXf curWaypoint, int i_formationShape,
                                  float vs_yaw, float i_con, float i_apfi, float scale); // 输出编队期望位置
bool formationShapeBuilding(VectorXf curWaypoint, int i_formationShape,
                            float vs_yaw, float i_con, float i_apf, float scale);// 建立队形
void multiRobotTrackDynamicPath(); // 编队模式下追踪动态路径
void initRobotsFormationAndPlanDynamicPath();// 初始化并跟踪路径v2
/******************************************************************************************/
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/





/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  保存数据函数目录
 *   按" ctrl + F " 对应查找
 *
 *     1. 保存默认数据
 *     2. 保存机器人自己的数据
 *     3. 保存3台机器人的数据
 *     4. 保存5台机器人的数据
 *
 */
/******************************************************************************************/
void saveData();  // 保存默认数据
void saveSingleRobotData(int _self_id, float _robot_pos_x, float _robot_pos_y, float _robot_pos_z, float _robot_yaw );// 保存机器人自己的数据
void saveSingleRobotAllDatas(int _self_id, float _robot_pos_x, float _robot_pos_y, float _robot_pos_z, float _robot_velocity_x, float _robot_velocity_y,float _x_linear_acceleration,float _y_linear_acceleration, float _robot_yaw,float _yaw_velocity );// 保存机器人自己的数据
void saveThreeRobotsData(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                         float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                         float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw);// 保存3台机器人的数据

void saveFiveRobotsData(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                        float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                        float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw,
                        float _robot4_pos_x, float _robot4_pos_y, float _robot4_pos_z, float _robot4_yaw,
                        float _robot5_pos_x, float _robot5_pos_y, float _robot5_pos_z, float _robot5_yaw); // 保存5台机器人的数据
/******************************************************************************************/

/******************************   ----------------------  **********************************/
/******************************   ----------------------  **********************************/


/*######################################################################################*/
/*@ multiple Stereo camera-based underwater Formation 总目录
 *  按" ctrl + F " 对应查找
 *
 *    1. 多机变量目录
 *    2. Kalman Filter for position and velocity
 *    3. Transform Robot position to world(virtual linkage structure)
 *    4. multiple stereo camera-based formation control function
 *
 *
/*######################################################################################*/



/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  1. 多机变量目录
 *   按" ctrl + F " 对应查找
 *
 *     1. virtuallinkagestructure
 *     2. robot position from eight camera
 *     3. kalman filter for pos_z
 *     4. kalman filter for position and velocity
 *
 */

/******************************   virtuallinkagestructure   **********************************/
extern float fAlphaInVLS, fLeftBetaInVLS, fRightBetaInVLS;
extern float fLeftLinkageLengthInVLS, fRightLinkageLengthInVLS;

extern Eigen::Vector3f vls_reference_point;
extern VirtualLinkageStructure vls;

/********************************* Matrix for formation control ******************************/
extern MatrixXf curRobotsPos8;
extern MatrixXf estRobotVel8;
extern MatrixXf expRobotsPos8;
extern VectorXf expRobotPos8;
extern MatrixXf minusLaplace8;

/******************************** Artificial potenrial function*******************************/
extern APF apfRobot8;

/******************************** Consensus control ******************************************/
extern C_DistrInfoConcencus consensusControl;     // 分布式一致性控制器


/**************************** robot position from eight camera   *****************************/
extern Eigen::MatrixXf robotsInfofromCamera8;
extern Eigen::VectorXf isrobot;
extern Eigen::VectorXf robotContour;
extern float robot1_pos_x8, robot1_pos_y8, robot1_pos_z8, robot1_vel_x8, robot1_vel_y8, robot1_vel_z8;
extern float robot2_pos_x8, robot2_pos_y8, robot2_pos_z8, robot2_vel_x8, robot2_vel_y8, robot2_vel_z8;
extern float robot3_pos_x8, robot3_pos_y8, robot3_pos_z8, robot3_vel_x8, robot3_vel_y8, robot3_vel_z8;

extern float camera8_roll, camera8_yaw, camera8_pitch, camera8_roll_vel, camera8_yaw_vel, camera8_pitch_vel;
extern asrobot::multi_robot8 multiRobotInfo;

extern Eigen::VectorXf robot1PosInVL;
extern Eigen::VectorXf robot3PosInVL;
extern Eigen::AngleAxisf rotation_vector;

/******************************   kalman filter for pos_z   **********************************/
extern C_KalmanFilter_v2 KF_v2_robot1_z; // 3台机器人各自的 z 坐标滤波器
extern C_KalmanFilter_v2 KF_v2_robot2_z;
extern C_KalmanFilter_v2 KF_v2_robot3_z;
extern VectorXf pressure_sensor_z;
extern VectorXf cur_sensors_z_1;
extern VectorXf cur_sensors_z_3;

/******************************   kalman filter for position and velocity   **********************************/
extern RNG rng1;
extern const int stateNum;
extern const int measureNum ;
extern const int winHeight ;
extern const int winWidth ;
extern KalmanFilter _kf_robot1;
extern Mat measurement_robot1 ;
extern KalmanFilter _kf_robot2;
extern Mat measurement_robot2 ;
extern KalmanFilter _kf_robot3;
extern Mat measurement_robot3 ;


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  2. Kalman Filter
 *   按" ctrl + F " 对应查找
 *
 *     1. kalmanfilter_init
 *     2. kalmanfilter_estimation for formation(FORMATION_MODE_1_2)
 *     3. kalmanfilter_estimation for formation(FORMATION_MODE_2_1)
 */

void kalmanfilter_init();

void kalmanfilter_estimation_1_2(MatrixXf &curRobotsPosEst_, MatrixXf &curRobotsVelEst_);

void kalmanfilter_estimation_2_1(MatrixXf &curRobotsPosEst_, MatrixXf &curRobotsVelEst_);

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  3. Transform to world
 *
 *     1. Transform to world(Virtual Linkage structure)
 *
 */

Eigen::VectorXf transPos2VL(Eigen::Vector3f robot1_, Eigen::VectorXf robot2_, float roll, float yaw, float pitch);


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  4. multiple stereo camera-based formation control function
 *
 *     1. initMultiCameraFormationCtrl
 *     2. multiCameraFormationCtrl
 */

void initMultiCameraFormationCtrl();

void multiCameraFormationCtrl();



//hcy
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// 用于docking的函数
void calcDockingPathPoint(void);
void updateDockingPathPoint(void);
void dockingPlanPathProcess(void);

// hcy:2代docking用到的函数
void calcDockingPathPoint_v2(void);
void updateDockingPathPoint_v2(void);
void dockingPlanPathProcess_v2(void);
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


#endif // F_MULTI_ROBOT_PLAN_H
