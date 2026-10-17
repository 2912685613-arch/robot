#include "../../include/asrobot/f_multi_robot_plan.h"
#define CONSENTRATE_CONSENSUS 0

/*######################################################################################*/
/*@ Quadrotor visoin-based Formation 总目录
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
 */

/******************************   基本变量   **********************************/
const int self_id = 1;             // 机器人自身id号  1
const int iDecisionLoopRate = 5;   // 主函数循环频率
bool is_initial_multi_msg = false; // 初始化多机信息标志位
/******************************   ----  **********************************/

/******************************   多机编队需要用到的矩阵   **********************************/
MatrixXf all_robots_cur_pos(DIMENSION, NUM_AGENT); // 所有智能体当前位置
MatrixXf all_robots_tar_pos(DIMENSION, NUM_AGENT); // 所有机器人目标位置
VectorXf this_robot_exp_pos(DIMENSION);            // 本台机器人的期望位置
MatrixXf minus_laplace(NUM_AGENT, NUM_AGENT);      // 负拉普拉斯矩阵

// 速度追踪版本的变量表
VectorXf ref_pos(DIMENSION);                       // 参考点的位置
VectorXf ref_speed(DIMENSION);                     // 参考点的速度
MatrixXf robots_devia_pos(DIMENSION, NUM_AGENT);   // 所有机器人相对于参考点的偏差
MatrixXf robots_devia_speed(DIMENSION, NUM_AGENT); // 偏差速度,变形会用到
MatrixXf robots_cur_pos(DIMENSION, NUM_AGENT);     // 所有机器人的当前位置
MatrixXf robots_cur_speed(DIMENSION, NUM_AGENT);   // 所有机器人当前的速度
VectorXf this_robot_exp_speed(DIMENSION);          // 这台机器人期望达到的速度

MatrixXf laplace_n(NUM_AGENT, NUM_AGENT);           // n维拉普拉斯矩阵
MatrixXf laplace_n_1(NUM_AGENT + 1, NUM_AGENT + 1); // 带虚拟leader的拉普拉斯矩阵 n+1维
/******************************   -----------------  **********************************/

/******************************   gazebo输出的深度计和陀螺仪信息   **********************************/
const float pressurePerMeter = 9.80638;
const float pressureInAir = 101.325; // 深度计初始值为101kpa
float depth = 1000;
double pitch = 0, roll = 0, yaw = 0;
float vz = 0;
float yaw_velocity = 0, x_linear_acceleration = 0, y_linear_acceleration = 0;

/******************************   --------------------  **********************************/

/******************************   机器人自身位置与速度   **********************************/
float X_position = 0, Y_position = 0, Z_position = 0;
int epx[4] = {200, 200, 550, 550};
int epy[4] = {100, 350, 350, 100};
int X_speed = 0, Y_speed = 0, Z_speed = 0;
/******************************   --------------------  **********************************/

/******************************   编队所有机器人自身信息和无人机自身位置信息  **********************************/
Eigen::MatrixXf robotsInfofromQuadrotor(4, 5); // 前两行是位置x，y，后两行是速度x，y
Eigen::VectorXf quadInfo(12, 1);
/******************************   --------------------------------------  **********************************/

/******************************   多机编队算法   **********************************/
VirtualStructure vsRobots;                    // 三角形虚拟结构
APF apf_agent;                                // 人工势场防撞
C_DistrInfoConcencus con_agent;               // 分布式一致性修正
C_SingleIntegConcensusFormation singint_cons; // 采用集中式,输出速度的一致性算法

float triangleShapeSideLength = 3; // 三角形编队边长
// float lineShapeLength = 6.2;      // 直线型线长
float lineShapeLength = 3;
int formationShape = TRIANGULAR_FORMATION; // 编队形状标志
float delta_distance = 1;                  // 与正规形状的误差阈值

bool is_initial_consensusControl = false; // 一致性初始化标志位
bool is_shape_built = false;              // has the formation shape building accomplished?

VectorXf vs_start_point; // initial virtual leader point
VectorXf tempVec;        // a transient variant
float vs_start_yaw;      // the initial yaw angle of virtual structure

/******************************   --------------  **********************************/

/******************************   地图与路径规划相关变量  **********************************/
int imgScale;
MatrixXf path_3d;                                            // 3d路径点
cv::Mat revMapImage;                                         // 地图
cv::Mat convert2worldMap;                                    // This need to be revised.
float vs_reference_x = 0, vs_reference_y = 0, vs_curYaw = 0; // 虚拟结构初始化的参考点
VectorXf achievedWaypointVec(3);                             // 3台机器人到达的路径点的数量 接收型变量
/******************************   -----------  **********************************/

/******************************   静态轨迹追踪变量  **********************************/
// 暂时还没有使用静态轨迹追踪
/******************************   -------------  **********************************/

/******************************   动态轨迹追踪变量  **********************************/
MatrixXf virtual_leader_new_3d_path; // 第(k)次路径
MatrixXf virtual_leader_old_3d_path; // (k-1)次路径
VectorXf virtual_leader_curpoint;    // used in multi-robot
VectorXf path_3d_exp_point;
int point_count = 0;
float switch_next_point_radius = 0.35;
/******************************   --------------  **********************************/

/******************************       DEBUG           ********************************/
MatrixXf path_points;   // 单机路径点
Vector2f target_point;  // 目标点
bool init_flag = false; // 单机初始化标志位
/*****************************                         *******************************/

/******************************    保存数据变量    **********************************/
// C_SaveData save_data;
C_SaveData save_data("/home/uuv_ws/src/data.txt"); // 也可以使用这个实例化， 在your path中输入你想保存文件的位置
/******************************   -----------------  **********************************/

C_KalmanFilter_WhiteNoise yaw_vel_filter;
float yaw_vel_filter_output;

/// hcy
/******************************   双目返回的位置姿态数据   **********************************/
float cam_pos_x, cam_pos_y, cam_pos_z;
float cam_roll, cam_pitch, cam_yaw;
// 新定义的aruco码到机体位姿
float aruco_pos_x, aruco_pos_y, aruco_pos_z;
float aruco_roll, aruco_pitch, aruco_yaw;

/******************************  利用双目计算出来的路径点 *************************/
MatrixXf docking_plan_path_points(3, 3);
VectorXf docking_exp_robot_pos(3);
VectorXf docking_cur_robot_pos(3);
int docking_point_count = 0;
int docking_point_count_v2 = 0; // hcy
C_Docking doc;
C_Docking_v2 doc_v2;              // hcy
bool docking_flag = false;        // hcy
bool tracking_flag = false;       // hcy
bool tracking_aruco_flag = false; // ljl
/*****************************************************************************/
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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************************************************************************************/
/*                                        获取多机的参考轨迹                                     */
/******************************************************************************************/
Eigen::MatrixXf trajectory_threeSpline;
referenceTrajectory mul_ref_trajectory_objective;
MatrixXf robot1_ref_trajectory;
MatrixXf robot2_ref_trajectory;
MatrixXf robot3_ref_trajectory;
MatrixXf virture_ref_trajectory;
void update_mul_ref_Info(MatrixXf ref, int shape)
{
    mul_ref_trajectory_objective = referenceTrajectory(3, lineShapeLength, triangleShapeSideLength, ref);
    mul_ref_trajectory_objective.updata_robot_ref_Info(shape);
    virture_ref_trajectory = mul_ref_trajectory_objective.virtureRobot_ref_Info_;
    robot1_ref_trajectory = mul_ref_trajectory_objective.robot1_ref_Info_;
    robot2_ref_trajectory = mul_ref_trajectory_objective.robot2_ref_Info_;
    robot3_ref_trajectory = mul_ref_trajectory_objective.robot3_ref_Info_;
}

/*@
 * 功能： 多机编队初始化：对多机编队矩阵中的各个矩阵， 多机算法进行初始化
 *
 * 输入：
 *   _dimension:  位置维度 取 1，或 2， 或 3
 *   _num_robots: 机器人个数
 *
 * 输出： 无
 */

void RobotsFormationInit(int _dimension, int _num_robots)
{

    // 各个矩阵初始化
    all_robots_cur_pos.setZero(_dimension, _num_robots);
    all_robots_tar_pos.setZero(_dimension, _num_robots);
    this_robot_exp_pos.setZero(_dimension, 1);
    minus_laplace.setZero(_num_robots, _num_robots);

    // 根据机器人数量选择拉普拉斯矩阵
    switch (_num_robots)
    {
    case 3:
    {
        minus_laplace << -1, 1, 0,
            0, -1, 1,
            1, 0, -1;
    };
    break;
    case 5:
    {
        minus_laplace << -1, 1, 0, 0, 0,
            1, -2, 1, 0, 0,
            0, 1, -2, 1, 0,
            0, 0, 1, -2, 1,
            0, 0, 0, 1, -1;
    };
    break;
    default:
    {
        cout << "please intialize correctly, [_num_robots] is 3 or 5" << endl;
    }
    }

    // 虚拟结构初始化 计算目标点位置
    float vs_yaw = vs_curYaw;
    vsRobots.vs_pointInWorld.x = path_3d(0, 0);
    vsRobots.vs_pointInWorld.y = path_3d(1, 0);
    vsRobots.vs_pointInWorld.z = path_3d(2, 0);
    vsRobots.setFormationParams(_num_robots, 1, 2, vs_yaw);

    vsRobots.calcRobotsPosInTriangularForm();
    all_robots_tar_pos = vsRobots.transform2WorldFrame();

    // 一致性初始化
    all_robots_cur_pos = all_robots_tar_pos;
    con_agent.InitDistrConcensusPmts(self_id, CONV_SPEED, minus_laplace, all_robots_cur_pos);

    // 人工势场参数初始化
    apf_agent.set_krep(0.1);
    apf_agent.set_r(0.5);
    apf_agent.set_r0(2);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 多机编队初始化v2： 对多机编队矩阵中的各个矩阵， 多机算法进行初始化
 *
 * 输入：
 *   _dimension:  位置维度 取 1，或 2， 或 3
 *   _num_robots: 机器人个数
 *
 * 输出： 无
 */
void formationControlInit(int _dimension, int _num_robots)
{
    all_robots_cur_pos.setZero(_dimension, _num_robots); // the current position of all robots
    all_robots_tar_pos.setZero(_dimension, _num_robots); // virtual structure position
    this_robot_exp_pos.setZero(_dimension, 1);

    // 根据机器人数量选择拉普拉斯矩阵
    switch (_num_robots)
    {
    case 3:
    {
        minus_laplace << -1, 1, 0,
            0, -1, 1,
            1, 0, -1;
    };
    break;
    case 5:
    {
        minus_laplace << -1, 1, 0, 0, 0,
            1, -2, 1, 0, 0,
            0, 1, -2, 1, 0,
            0, 0, 1, -2, 1,
            0, 0, 0, 1, -1;
    };
    break;
    default:
    {
        cout << "please intialize correctly, [_num_robots] is 3 or 5" << endl;
    }
    }

    // 添加虚拟leader位置
    vsRobots.setFormationParams(_num_robots, lineShapeLength, triangleShapeSideLength, 0);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队路径规划：编队运行时，对机器人期望路径点进行规划
 *
 * 输入：
 *   virtual_leader_pos:    虚拟leader的位置
 *   i_con :                一致性输出权重
 *   i_apfi:                人工势场内部防撞的权重
 *   s     :                步长， s越大，规划的期望点离目标点越远
 * 输出： 无
 */
VectorXf RobotsFormationPlan(VectorXf virtual_leader_pos, float i_con, float i_apfi, float s)
{

    VectorXf vec_agent_con;  // 一致性输出的步长
    VectorXf vec_agent_apfi; // 虚拟结构输出的步长

    VectorXf whole_vec(3, 1); // 总的步长

    // 虚拟结构，计算虚拟leader位置和人工势场
    vsRobots.vs_pointInWorld.x = virtual_leader_pos(0);
    vsRobots.vs_pointInWorld.y = virtual_leader_pos(1);
    vsRobots.vs_pointInWorld.z = virtual_leader_pos(2);
    vsRobots.vs_yaw = vs_curYaw;

    vsRobots.calcRobotsPosInTriangularForm();
    all_robots_tar_pos = vsRobots.transform2WorldFrame();
    cout << "all_robots_tar_pos: " << all_robots_tar_pos << endl;
    cout << "vsRobots.vs_yaw" << vsRobots.vs_yaw << endl;

    // 一致性 接收目标点，输出步长
    con_agent.CalDistrConcensusOutput(all_robots_cur_pos, all_robots_tar_pos, vec_agent_con);
    vec_agent_con = vec_agent_con - all_robots_cur_pos.col(self_id - 1);

    // 人工势场内部防撞 输出的步长
    vec_agent_apfi = apf_agent.apf_InterRepulsion(all_robots_cur_pos.col(self_id - 1), all_robots_cur_pos);

    // 总步长
    whole_vec = i_con * vec_agent_con + i_apfi * vec_agent_apfi;

    this_robot_exp_pos = all_robots_cur_pos.col(self_id - 1) + s * whole_vec;

    return this_robot_exp_pos;
    //    return all_robots_tar_pos.block(0, self_id-1, 3, 1);
    //    return vec_agent_con;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 检查编队形状： 对多机编队矩阵中的各个矩阵， 多机算法进行初始化
 *
 * 输入：
 *   i_formationShape:  输入编队形状 三角形或1字形
 *
 * 输出： 无
 */
bool checkFormationShapeBuilding(int i_formationShape)
{
    float distance_r1_r2 = sqrt((all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 1)) * (all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 1)) + (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 1)) * (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 1)));
    float distance_r2_r3 = sqrt((all_robots_cur_pos(0, 1) - all_robots_cur_pos(0, 2)) * (all_robots_cur_pos(0, 1) - all_robots_cur_pos(0, 2)) + (all_robots_cur_pos(1, 1) - all_robots_cur_pos(1, 2)) * (all_robots_cur_pos(1, 1) - all_robots_cur_pos(1, 2)));
    float distance_r1_r3 = sqrt((all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 2)) * (all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 2)) + (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 2)) * (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 2)));

    float distance_error_r1_r2 = 0, distance_error_r2_r3 = 0, distance_error_r1_r3 = 0;
    if (TRIANGULAR_FORMATION == i_formationShape)
    {
        distance_error_r1_r2 = abs(distance_r1_r2 - triangleShapeSideLength);
        distance_error_r2_r3 = abs(distance_r2_r3 - triangleShapeSideLength);
        distance_error_r1_r3 = abs(distance_r1_r3 - triangleShapeSideLength);
    }
    else if (LATERAL_LINE_FORMATION == i_formationShape)
    {
        distance_error_r1_r2 = abs(distance_r1_r2 - lineShapeLength / 2);
        distance_error_r2_r3 = abs(distance_r2_r3 - lineShapeLength / 2);
        distance_error_r1_r3 = abs(distance_r1_r3 - lineShapeLength);
    }
    else if (LONGITUDINAL_LINE_FORMATION == i_formationShape)
    {
        distance_error_r1_r2 = abs(distance_r1_r2 - lineShapeLength / 2);
        distance_error_r2_r3 = abs(distance_r2_r3 - lineShapeLength / 2);
        distance_error_r1_r3 = abs(distance_r1_r3 - lineShapeLength);
    }

    if (distance_error_r1_r2 + distance_error_r2_r3 + distance_error_r1_r3 < delta_distance)
        is_shape_built = true;
    return is_shape_built;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 输出编队期望位置 ： 对多机编队矩阵中的各个矩阵， 多机算法进行初始化
 *
 * 输入：
 *   curWaypoint      :  虚拟结构路径点
 *   i_formationShape : 编队形状
 *   vs_yaw           : 虚拟结构航向角
 *   i_con            : 一致性权重
 *   i_apfi           : 内部防撞权重
 *   scale            : 步长（原始输出的倍数）
 *
 * 输出： 无
 */
VectorXf formationControlPosition(VectorXf curWaypoint, int i_formationShape, float vs_yaw, float i_con, float i_apfi, float scale)
{
    VectorXf vec_agent_con;
    VectorXf vec_agent_apfi;

    vsRobots.vs_pointInWorld.x = curWaypoint(0);
    vsRobots.vs_pointInWorld.y = curWaypoint(1);
    vsRobots.vs_pointInWorld.z = curWaypoint(2);
    vsRobots.vs_yaw = vs_yaw; // 弧度

    if (TRIANGULAR_FORMATION == i_formationShape)
    {
        cout << "------------------------------------------------------------  Triangle formatioin shape..." << endl;
        vsRobots.vs_yaw -= M_PI / 2;
        vsRobots.calcRobotsPosInTriangularForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }
    else if (LATERAL_LINE_FORMATION == i_formationShape)
    {
        cout << "------------------------------------------------------------  1 line formatioin shape..." << endl;

        vsRobots.calcRobotsPosInLatLineForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }
    else if (LONGITUDINAL_LINE_FORMATION == i_formationShape)
    {
        cout << "------------------------------------------------------------  1_T line formatioin shape..." << endl;

        vsRobots.calcRobotsPosInLonLineForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }

    cout << "all_robots_tar_pos: " << all_robots_tar_pos << endl;

    //  一致性
    if (is_initial_consensusControl == false)
    {
        con_agent.InitDistrConcensusPmts(self_id, CONV_SPEED, minus_laplace, all_robots_tar_pos);
        is_initial_consensusControl = true;
    }

    con_agent.CalDistrConcensusOutput(all_robots_cur_pos, all_robots_tar_pos, vec_agent_con);
    vec_agent_con = vec_agent_con - all_robots_cur_pos.col(self_id - 1);

    // 人工势场防撞
    vec_agent_apfi = apf_agent.apf_InterRepulsion(all_robots_cur_pos.col(self_id - 1), all_robots_cur_pos);

    /// DEBUG
    VectorXf vec_agent_apfi_temp(3);
    vec_agent_apfi_temp << 0, 0, 0;
    vec_agent_apfi = vec_agent_apfi_temp;
    ///

    // 总输出
    this_robot_exp_pos = all_robots_cur_pos.col(self_id - 1) + (i_con * vec_agent_con + i_apfi * vec_agent_apfi) * scale;

    return this_robot_exp_pos;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 建立队形 ： 建立三角形或1字形
 *
 * 输入：
 *   curWaypoint      :  虚拟结构路径点
 *   i_formationShape : 编队形状
 *   vs_yaw           : 虚拟结构航向角
 *   i_con            : 一致性权重
 *   i_apfi           : 内部防撞权重
 *   scale            : 步长（原始输出的倍数）
 *
 * 输出： 无
 */
bool formationShapeBuilding(VectorXf curWaypoint, int i_formationShape, float vs_yaw, float i_con, float i_apf, float scale)
{
    //    bool is_shape_built = false;
    path_3d_exp_point = formationControlPosition(curWaypoint, i_formationShape, vs_yaw, i_con, i_apf, scale);

    return checkFormationShapeBuilding(i_formationShape);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队模式下追踪动态路径 ： 编队模式下与其他机器人配合取出动态路径点
 *
 * 输入： 无
 *
 * 输出： 无
 */
void multiRobotTrackDynamicPath()
{
    virtual_leader_new_3d_path = path_3d; // receive new 3d path in time
    cout << " virtual_leader_new_3d_path: " << endl
         << virtual_leader_new_3d_path.transpose() << endl;
    cout << " all_robot_tar_pos " << endl
         << all_robots_tar_pos << endl;
    cout << " vsRobots.vs_pointInWorld:  " << vsRobots.vs_pointInWorld << endl
         << "vsRobots.vs_yaw:  " << vsRobots.vs_yaw * 180 / M_PI << endl;

    virtual_leader_curpoint = virtual_leader_old_3d_path.col(point_count);

    /// DEBUG
    //    vs_curYaw = 0;
    if (point_count + 1 < virtual_leader_old_3d_path.cols())
    {
        tempVec = virtual_leader_old_3d_path.col(point_count + 1) - virtual_leader_old_3d_path.col(point_count);

        //        vs_curYaw = atan2(tempVec(1), tempVec(0)) * 180 / M_PI;
        vs_curYaw = atan2(tempVec(1), tempVec(0));
    }

    /// DEBUG

    path_3d_exp_point = formationControlPosition(virtual_leader_curpoint, formationShape, vs_curYaw, 1, 10, 1);

    if ((virtual_leader_old_3d_path.cols() <= 0))
    {
        cout << " the point matrix is empty !!!" << endl;
    }
    else
    {
        if (point_count < virtual_leader_old_3d_path.cols() - 1)
        {

            cout << "calculating !!!  ===  " << point_count << "----" << virtual_leader_old_3d_path.cols() << endl;

            // x^2 + y^2 judge the distance
            if (sqrt(((path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) * (path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1))) + ((path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)) * (path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)))) < switch_next_point_radius)
            {

                if ((point_count < achievedWaypointVec(2 - 1)) ||
                    (point_count < achievedWaypointVec(3 - 1)))
                {
                    point_count += 1;
                }
                else if ((point_count == achievedWaypointVec(2 - 1)) &&
                         (point_count == achievedWaypointVec(3 - 1)))
                {
                    point_count += 1;
                }
                else
                {
                    point_count = point_count;
                }

                if (virtual_leader_new_3d_path.cols() != virtual_leader_old_3d_path.cols())
                {
                    if (virtual_leader_new_3d_path.cols() > 5) // only larger than 5 to update old path points
                    {
                        if (sqrt(((virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count)) *
                                  (virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count))) +
                                 ((virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)) *
                                  (virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)))) < 1) // the distance only smaller than 1m, then update the old path
                        {
                            virtual_leader_old_3d_path = virtual_leader_new_3d_path;
                            point_count = 0;
                        }
                    }
                }
            }
        }
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/***************************************************************************************/
/*                                 静态轨迹追踪                                          */
/***************************************************************************************/
#define STATIC_PATH_DEBUG 0
#if STATIC_PATH_DEBUG

MatrixXf virtual_leader_points;
VectorXf virtual_leader_curpoint;
VectorXf path_3d_exp_point;
int point_count = 0;
float switch_next_point_radius = 0.35;

void pathPointsInit()
{
    virtual_leader_points = path_3d;
    // get the expect initial value
    path_3d_exp_point = all_robots_tar_pos.block(0, self_id - 1, 3, 1);
    cout << "path point matrix was initialized!!!" << endl;
}

void singleOutputPathPoints()
{
    // cout << "virtual_leader_points.cols() ::: " << virtual_leader_points.cols() <<  endl;
    // cout << "virtual_leader_points.rows() ::: " << virtual_leader_points.rows() <<  endl;
    // cout << "robotsInfofromQuadrotor  " <<endl<<  robotsInfofromQuadrotor << endl;
    cout << "path_3d ::: " << endl
         << path_3d.transpose() << endl;

    if ((virtual_leader_points.cols() <= 0))
    {
        cout << " the point matrix is empty !!!" << endl;
    }
    else
    {
        if (point_count < virtual_leader_points.cols())
        {
            motion.e_x = virtual_leader_points(0, point_count);
            motion.e_y = virtual_leader_points(1, point_count);
            motion.e_z = 0;
            motion.e_navyaw = atan2(virtual_leader_points(1, point_count) - robotsInfofromQuadrotor(1, self_id - 1), virtual_leader_points(0, point_count) - robotsInfofromQuadrotor(0, self_id - 1)) * 57.3;

            cout << "calculating !!!  ===  " << point_count << "----" << virtual_leader_points.cols() << endl;

            // x^2 + y^2 judge the distance
            if (sqrt(((virtual_leader_points(0, point_count) - robotsInfofromQuadrotor(0, self_id - 1)) * (virtual_leader_points(0, point_count) - robotsInfofromQuadrotor(0, self_id - 1))) + ((virtual_leader_points(1, point_count) - robotsInfofromQuadrotor(1, self_id - 1)) * (virtual_leader_points(1, point_count) - robotsInfofromQuadrotor(1, self_id - 1)))) < switch_next_point_radius)
            {
                point_count += 1;
            }
        }
        else
        {
            cout << "point_count is full" << endl;
            cout << "tracking is finished" << endl;
        }
    }

    if (point_count == virtual_leader_points.cols())
    {
        motion.e_x = virtual_leader_points(0, point_count - 1);
        motion.e_y = virtual_leader_points(1, point_count - 1);
        motion.e_z = 0;
        motion.e_navyaw = atan2(virtual_leader_points(1, point_count - 1) - robotsInfofromQuadrotor(1, self_id - 1), virtual_leader_points(0, point_count - 1) - robotsInfofromQuadrotor(0, self_id - 1)) * 57.3;
    }

    motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
    motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
    motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1);
    motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1);
    motion.c_z = -depth;
    motion.c_navyaw = yaw;
    motion.c_navroll = roll;
    motion.c_navpitch = pitch;

    cout << "self_id: " << self_id << endl;
    cout << "e_x: " << motion.e_x << "    " << "e_y:  " << motion.e_y << endl;
    cout << "c_x: " << motion.c_x << "    " << "c_vx: " << motion.c_speedx << endl;
    cout << "c_y: " << motion.c_y << "    " << "c_vy: " << motion.c_speedy << endl;
    cout << "c_z: " << motion.c_z << "    " << "c_yaw: " << motion.c_navyaw << endl;
}

void outputPathPoints()
{
    if ((virtual_leader_points.cols() <= 0) || (virtual_leader_points.rows() <= 0))
    {
        cout << " the point matrix is empty !!!" << endl;
    }
    else
    {
        if (point_count < virtual_leader_points.cols())
        {
            // x^2 + y^2 judge the distance
            if (sqrt(((path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) * (path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1))) + ((path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)) * (path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)))) < switch_next_point_radius)
            {
                virtual_leader_curpoint = virtual_leader_points.block(0, point_count, 3, 1);
                path_3d_exp_point = RobotsFormationPlan(virtual_leader_curpoint, 1, 0, 1);
                point_count += 1;
            }
        }
    }

    motion.e_x = path_3d_exp_point(0);
    motion.e_y = path_3d_exp_point(1);
    motion.e_z = 0;

    motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
    motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
    motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1);
    motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1);
    motion.c_z = -depth;
    motion.c_navyaw = yaw;
    motion.c_navroll = roll;
    motion.c_navpitch = pitch;

    cout << "self_id: " << self_id << endl;
    cout << "c_x: " << motion.c_x << "    " << "c_vx: " << motion.c_speedx << endl;
    cout << "c_y: " << motion.c_y << "    " << "c_vy: " << motion.c_speedy << endl;
    cout << "c_z: " << motion.c_z << "    " << "c_yaw: " << motion.c_navyaw << endl;
}

#endif
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                           动态轨迹追踪  可追踪实时规划好的轨迹                             */
/***************************************************************************************/
#define DYNAMIC_PATH_DEBUG 1
#if DYNAMIC_PATH_DEBUG
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 动态轨迹追踪初始化： 路径追踪初始化，对上次路径，当前路径，起始点分别进行初始化
 *
 * 输入： 无
 *
 * 输出： 无
 */
void pathTrackingInit()
{
    // 1 initial new and old [ virtual_leader_path_3d ]
    virtual_leader_new_3d_path = path_3d;
    virtual_leader_old_3d_path = path_3d;

    virtual_leader_curpoint = path_3d.block(0, 0, 3, 1);
    // 2 get the expect initial 3d path points
    path_3d_exp_point = all_robots_tar_pos.block(0, self_id - 1, 3, 1);
    cout << "path point matrix was initialized!!!" << endl;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 单机模式下机器人跟踪路径： 其中 [virtual_leader_old_3d_path]变量 作为机器人的路径点
 *
 * 输入： 无
 *
 * 输出： 无
 */
void dynamicPathTracking()
{
    virtual_leader_new_3d_path = path_3d; // receive new 3d path in time
    cout << " virtual_leader_new_3d_path: " << endl
         << virtual_leader_new_3d_path.transpose() << endl;

    if ((virtual_leader_old_3d_path.cols() <= 0))
    {
        cout << " the point matrix is empty !!!" << endl;
    }
    else
    {
        if (point_count < virtual_leader_old_3d_path.cols())
        {

            cout << "calculating !!!  ===  " << point_count << "----" << virtual_leader_old_3d_path.cols() << endl;

            // x^2 + y^2 judge the distance
            if (sqrt(((virtual_leader_old_3d_path(0, point_count) - robotsInfofromQuadrotor(0, self_id - 1)) * (virtual_leader_old_3d_path(0, point_count) - robotsInfofromQuadrotor(0, self_id - 1))) + ((virtual_leader_old_3d_path(1, point_count) - robotsInfofromQuadrotor(1, self_id - 1)) * (virtual_leader_old_3d_path(1, point_count) - robotsInfofromQuadrotor(1, self_id - 1)))) < switch_next_point_radius)
            {
                point_count += 1;

                if (virtual_leader_new_3d_path.cols() != virtual_leader_old_3d_path.cols())
                {
                    if (virtual_leader_new_3d_path.cols() > 3) // only larger than 3 to update old path points
                    {
                        if (sqrt(((virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count)) *
                                  (virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count))) +
                                 ((virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)) *
                                  (virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)))) < 1) // the distance only smaller than 1m, then update the old path
                        {
                            virtual_leader_old_3d_path = virtual_leader_new_3d_path;
                            point_count = 0;
                        }
                    }
                }
            }
        }
        else
        {
            cout << "point_count is full" << endl;
            cout << "tracking is finished" << endl;
        }
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队模式下机器人跟踪路径：  其中 [path_3d_exp_point]变量 作为机器人每次需要到达的点
 *
 * 输入： 无
 *
 * 输出： 无
 */
void dynamicMultiPathPoints()
{
    virtual_leader_new_3d_path = path_3d; // receive new 3d path in time
    cout << " virtual_leader_new_3d_path: " << endl
         << virtual_leader_new_3d_path.transpose() << endl;

    cout << "  all_robots_tar_pos " << endl
         << all_robots_tar_pos << endl;

    cout << "vsRobots.vs_pointInWorld:" << vsRobots.vs_pointInWorld << endl
         << "vsRobots.vs_yaw:" << vsRobots.vs_yaw << endl;

    cout << "*****************path_3d_exp_point: " << path_3d_exp_point << endl;

    if ((virtual_leader_old_3d_path.cols() <= 0))
    {
        cout << " the point matrix is empty !!!" << endl;
    }
    else
    {
        if (point_count < virtual_leader_old_3d_path.cols())
        {

            cout << "calculating !!!  ===  " << point_count << "----" << virtual_leader_old_3d_path.cols() << endl;

            virtual_leader_curpoint = virtual_leader_old_3d_path.block(0, point_count, 3, 1);
            path_3d_exp_point = RobotsFormationPlan(virtual_leader_curpoint, 1, 0, 1);

            // x^2 + y^2 judge the distance
            if (sqrt(((path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) * (path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1))) + ((path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)) * (path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)))) < switch_next_point_radius)
            {
                // 各路point互相锁定
                if ((point_count < achievedWaypointVec(1 - 1)) ||
                    (point_count < achievedWaypointVec(3 - 1)))
                {
                    point_count += 1;
                }
                else if ((point_count == achievedWaypointVec(1 - 1)) &&
                         (point_count == achievedWaypointVec(3 - 1)))
                {
                    point_count += 1;
                }
                else
                {
                    point_count = point_count;
                }

                if (virtual_leader_new_3d_path.cols() != virtual_leader_old_3d_path.cols())
                {
                    if (virtual_leader_new_3d_path.cols() > 5) // only larger than 5 to update old path points
                    {
                        if (sqrt(((virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count)) *
                                  (virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count))) +
                                 ((virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)) *
                                  (virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)))) < 1) // the distance only smaller than 1m, then update the old path
                        {
                            virtual_leader_old_3d_path = virtual_leader_new_3d_path;
                            point_count = 0;
                        }
                    }
                }
            }
        }
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#endif

#if CONSENTRATE_CONSENSUS

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 多机编队初始化v3,集中式一致性实现,速度输出版本： 对多机编队矩阵中的各个矩阵， 多机算法进行初始化
 *
 * 输入：
 *   _dimension:  位置维度 取 1，或 2， 或 3
 *   _num_robots: 机器人个数
 *
 * 输出： 无
 */
void formationInit_v2(int _self_id, int _dimension, int _num_robots, float _conv_speed)
{

    // 基本矩阵都给初始值
    all_robots_cur_pos.setZero(_dimension, _num_robots); // the current position of all robots
    all_robots_tar_pos.setZero(_dimension, _num_robots); // virtual structure position

    // 基本矩阵全部给初值0
    ref_pos.setZero(_dimension, _num_robots);
    ref_speed.setZero(_dimension, _num_robots);
    robots_devia_pos.setZero(_dimension, _num_robots);
    robots_devia_speed.setZero(_dimension, _num_robots);
    robots_cur_pos.setZero(_dimension, _num_robots);
    robots_cur_speed.setZero(_dimension, _num_robots);

    // 根据机器人数量选择拉普拉斯矩阵
    switch (_num_robots)
    {
    case 3:
    {
        laplace_n << 1, 0, -1,
            -1, 1, 0,
            0, -1, 1;
        laplace_n_1 << 2, 0, -1, -1,
            -1, 2, 0, -1,
            0, -1, 2, -1,
            0, 0, 0, 0;
    };
    break;
    default:
    {
        cout << "please intialize correctly, [_num_robots] is 3" << endl;
    }
    }

    // 初始化虚拟结构参数
    vsRobots.setFormationParams(_num_robots, lineShapeLength, triangleShapeSideLength, 0);

    // 初始化一致性参数
    singint_cons = C_SingleIntegConcensusFormation(_self_id, _num_robots, _dimension);

    // 设置一致收敛速度
    singint_cons.setGamma(_conv_speed);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void pathTrackingInit_v2()
{
    // 1 initial new and old [ virtual_leader_path_3d ]
    virtual_leader_new_3d_path = path_3d;
    virtual_leader_old_3d_path = path_3d;

    virtual_leader_curpoint = path_3d.block(0, 0, 3, 1);
    // 2 get the expect initial 3d path points
    path_3d_exp_point = all_robots_tar_pos.block(0, self_id - 1, 3, 1);
    cout << "path point matrix was initialized!!!" << endl;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队形成或编队形状保持时需要用该函数提供该机器人期望速度
 *
 * 输入：
 *   i_formationShape :  编队形状
 *
 * 输出： 该机器人期望的速度
 */
VectorXf formationMaintain(VectorXf _initWayPoint, int _i_formationShape, float _initYaw)
{

    vsRobots.vs_pointInWorld.x = _initWayPoint(0);
    vsRobots.vs_pointInWorld.y = _initWayPoint(1);
    vsRobots.vs_pointInWorld.z = _initWayPoint(2);
    vsRobots.vs_yaw = _initYaw;

    // 计算各种形状下的 各个机器人的位置坐标
    if (TRIANGULAR_FORMATION == _i_formationShape)
    {
        cout << "------------------------------------------------------------  Triangle formatioin shape..." << endl;
        vsRobots.calcRobotsPosInTriangularForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }
    else if (LATERAL_LINE_FORMATION == _i_formationShape)
    {
        cout << "------------------------------------------------------------  1 line formatioin shape..." << endl;

        vsRobots.calcRobotsPosInLatLineForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }
    else if (LONGITUDINAL_LINE_FORMATION == _i_formationShape)
    {
        cout << "------------------------------------------------------------  1_T line formatioin shape..." << endl;

        vsRobots.calcRobotsPosInLonLineForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }

    cout << "all_robots_tar_pos: " << all_robots_tar_pos << endl;

    // 提取他们的相对位置偏差
    for (int i = 0; i < all_robots_tar_pos.rows(); i++)
    {
        robots_devia_pos.col(i) = all_robots_tar_pos.col(i) - ref_pos;
    }

    // 编队保持需要参考坐标,所有机器人距离参考点的偏差坐标,所有机器人的当前位置
    // 输出当前机器人的期望速度
    this_robot_exp_speed = singint_cons.calFormationMaintainOutput(ref_pos, robots_devia_pos, robots_cur_pos);

    return this_robot_exp_speed;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队运行时需要用该函数提供该机器人期望速度
 *
 * 输入：
 *   i_formationShape :  编队形状
 *
 * 输出： 该机器人期望的速度
 */
bool is_initialized = false;
VectorXf last_ref_pos(3);
float time_step = 1; // 把这个数值调大,参考速度就会变小
VectorXf formationRunning(VectorXf _runningWayPoint, int _i_formationShape, float _running_Yaw)
{

    vsRobots.vs_pointInWorld.x = _runningWayPoint(0);
    vsRobots.vs_pointInWorld.y = _runningWayPoint(1);
    vsRobots.vs_pointInWorld.z = _runningWayPoint(2);
    vsRobots.vs_yaw = _running_Yaw;

    // 计算各种形状下的 各个机器人的位置坐标
    if (TRIANGULAR_FORMATION == _i_formationShape)
    {
        cout << "------------------------------------------------------------  Triangle formatioin shape..." << endl;
        vsRobots.calcRobotsPosInTriangularForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }
    else if (LATERAL_LINE_FORMATION == _i_formationShape)
    {
        cout << "------------------------------------------------------------  1 line formatioin shape..." << endl;

        vsRobots.calcRobotsPosInLatLineForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }
    else if (LONGITUDINAL_LINE_FORMATION == _i_formationShape)
    {
        cout << "------------------------------------------------------------  1_T line formatioin shape..." << endl;

        vsRobots.calcRobotsPosInLonLineForm();
        all_robots_tar_pos = vsRobots.transform2WorldFrame();
    }

    cout << "all_robots_tar_pos: " << all_robots_tar_pos << endl;

    // 提取他们的相对位置偏差
    for (int i = 0; i < all_robots_tar_pos.rows(); i++)
    {
        robots_devia_pos.col(i) = all_robots_tar_pos.col(i) - ref_pos;
    }

    VectorXf __vec(3);
    __vec << 0, 0, 0;
    // 偏差的速度为0
    for (int i = 0; i < all_robots_tar_pos.rows(); i++)
    {
        robots_devia_speed.col(i) = __vec;
    }

    if (is_initialized == false)
    {
        ref_speed = __vec;
        last_ref_pos = ref_pos;
    }
    else
    {
        ref_speed = 1 / time_step * (ref_pos - last_ref_pos);
    }

    // 编队保持需要参考坐标,所有机器人距离参考点的偏差坐标,所有机器人的当前位置
    // 输出当前机器人的期望速度
    this_robot_exp_speed = singint_cons.calFormationTrackingOutput(ref_pos, ref_speed,
                                                                   robots_devia_pos, robots_devia_speed,
                                                                   robots_cur_pos);

    return this_robot_exp_speed;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 建立队形 ： 建立三角形或1字形
 *
 * 输入：
 *   curWaypoint      :  虚拟结构路径点
 *   i_formationShape : 编队形状
 *   vs_yaw           : 虚拟结构航向角
 *   i_con            : 一致性权重
 *   i_apfi           : 内部防撞权重
 *   scale            : 步长（原始输出的倍数）
 *
 * 输出： 无
 */

bool checkFormationShapeBuilding_v2(int i_formationShape)
{
    float distance_r1_r2 = sqrt((all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 1)) * (all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 1)) + (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 1)) * (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 1)));
    float distance_r2_r3 = sqrt((all_robots_cur_pos(0, 1) - all_robots_cur_pos(0, 2)) * (all_robots_cur_pos(0, 1) - all_robots_cur_pos(0, 2)) + (all_robots_cur_pos(1, 1) - all_robots_cur_pos(1, 2)) * (all_robots_cur_pos(1, 1) - all_robots_cur_pos(1, 2)));
    float distance_r1_r3 = sqrt((all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 2)) * (all_robots_cur_pos(0, 0) - all_robots_cur_pos(0, 2)) + (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 2)) * (all_robots_cur_pos(1, 0) - all_robots_cur_pos(1, 2)));

    float distance_error_r1_r2 = 0, distance_error_r2_r3 = 0, distance_error_r1_r3 = 0;
    if (TRIANGULAR_FORMATION == i_formationShape)
    {
        distance_error_r1_r2 = abs(distance_r1_r2 - triangleShapeSideLength);
        distance_error_r2_r3 = abs(distance_r2_r3 - triangleShapeSideLength);
        distance_error_r1_r3 = abs(distance_r1_r3 - triangleShapeSideLength);
    }
    else if (LATERAL_LINE_FORMATION == i_formationShape)
    {
        distance_error_r1_r2 = abs(distance_r1_r2 - lineShapeLength / 2);
        distance_error_r2_r3 = abs(distance_r2_r3 - lineShapeLength / 2);
        distance_error_r1_r3 = abs(distance_r1_r3 - lineShapeLength);
    }
    else if (LONGITUDINAL_LINE_FORMATION == i_formationShape)
    {
        distance_error_r1_r2 = abs(distance_r1_r2 - lineShapeLength / 2);
        distance_error_r2_r3 = abs(distance_r2_r3 - lineShapeLength / 2);
        distance_error_r1_r3 = abs(distance_r1_r3 - lineShapeLength);
    }

    if (distance_error_r1_r2 + distance_error_r2_r3 + distance_error_r1_r3 < delta_distance)
        is_shape_built = true;
    return is_shape_built;
}

bool formationShapeBuilding_v2(VectorXf curWaypoint, int i_formationShape, float vs_yaw)
{
    //    bool is_shape_built = false;
    path_3d_exp_point = formationMaintain(curWaypoint, i_formationShape, vs_yaw);

    return checkFormationShapeBuilding_v2(i_formationShape);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队模式下追踪动态路径 ： 编队模式下与其他机器人配合取出动态路径点
 *
 * 输入： 无
 *
 * 输出： 无
 */
void multiRobotTrackDynamicPath_v2()
{
    virtual_leader_new_3d_path = path_3d; // receive new 3d path in time
    cout << " virtual_leader_new_3d_path: " << endl
         << virtual_leader_new_3d_path.transpose() << endl;
    cout << " all_robot_tar_pos " << endl
         << all_robots_tar_pos << endl;
    cout << " vsRobots.vs_pointInWorld:  " << vsRobots.vs_pointInWorld << endl
         << "vsRobots.vs_yaw:  " << vsRobots.vs_yaw * 180 / M_PI << endl;

    virtual_leader_curpoint = virtual_leader_old_3d_path.block(0, point_count, 3, 1);

    this_robot_exp_speed = formationRunning(virtual_leader_curpoint, formationShape, vsRobots.vs_yaw);

    /// DEBUG HERE  点过于稀疏..增加点数

    if ((virtual_leader_old_3d_path.cols() <= 0))
    {
        cout << " the point matrix is empty !!!" << endl;
    }
    else
    {
        if (point_count < virtual_leader_old_3d_path.cols())
        {

            cout << "calculating !!!  ===  " << point_count << "----" << virtual_leader_old_3d_path.cols() << endl;

            // x^2 + y^2 judge the distance
            if (sqrt(((path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) * (path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1))) + ((path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)) * (path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1)))) < switch_next_point_radius)
            {

                if ((point_count < achievedWaypointVec(1 - 1)) ||
                    (point_count < achievedWaypointVec(3 - 1)))
                {
                    point_count += 1;
                }
                else if ((point_count == achievedWaypointVec(1 - 1)) &&
                         (point_count == achievedWaypointVec(3 - 1)))
                {
                    point_count += 1;
                }
                else
                {
                    point_count = point_count;
                }

                if (virtual_leader_new_3d_path.cols() != virtual_leader_old_3d_path.cols())
                {
                    if (virtual_leader_new_3d_path.cols() > 5) // only larger than 5 to update old path points
                    {
                        if (sqrt(((virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count)) *
                                  (virtual_leader_new_3d_path(0, 0) - virtual_leader_old_3d_path(0, point_count))) +
                                 ((virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)) *
                                  (virtual_leader_new_3d_path(1, 0) - virtual_leader_old_3d_path(1, point_count)))) < 1) // the distance only smaller than 1m, then update the old path
                        {
                            virtual_leader_old_3d_path = virtual_leader_new_3d_path;
                            point_count = 0;
                        }
                    }
                }
            }
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队初始化, 编队保持,编队运行
 *
 * 输入：
 *   i_formationShape :  编队形状
 *
 * 输出： 该机器人期望的速度
 */

void formationInitAndMaintainAndRunning()
{
    if (is_initial_multi_msg == false)
    {
        cout << "Initialize formation control" << endl;
        if (path_3d.cols() > 5)
        {
            // formation initialization
            // 各个基础变量初始化
            formationInit_v2(1, DIMENSION, NUM_AGENT, 1);
            // 新老路径初始化
            pathTrackingInit_v2();

            // 虚拟结构起始点
            vs_start_point = virtual_leader_old_3d_path.block(0, point_count, 3, 1);
            tempVec = virtual_leader_old_3d_path.block(0, point_count + 1, 3, 1) - virtual_leader_old_3d_path.block(0, point_count, 3, 1);
            ;

            // 虚拟结构起始航向角
            vs_start_yaw = atan2(tempVec(1), tempVec(0)) * 180 / M_PI;

            // 设置为true, 表示已经完成初始化
            is_initial_multi_msg = true;
        }
        else
        {
            // 发过来的路径太少,重新初始化
            cout << "the number of whole points is : " << path_3d.cols() << endl;
            cout << "[path_3d] is less than 5 !!! Can not build formation shape..." << endl;
            is_initial_multi_msg = false;
        }
    }
    else
    {
        if (!formationShapeBuilding_v2(vs_start_point, formationShape, vs_start_yaw))
        {
            cout << "Waiting for formation shape building......" << endl;
        }
        else
        {
            cout << "******************** dynamic path tracking ......*******************" << endl;
            // dynamicPathTracking();
            // dynamicMultiPathPoints();
            multiRobotTrackDynamicPath_v2();
            // singleOutputPathPoints();
            // outputPathPoints();
            // AgentsFormation(path_3d.col(0), 1,0,1);
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 初始化并跟踪路径： 设置机器人的初始位置，并进行实时路径规划 由 [decision]结点 直接调用
 *
 * 输入： 控制模式： 单机模式或编队模式
 *
 * 输出： 无
 */
void initRobotPosAndPlanPath(int _mode)
{
    if (is_initial_multi_msg == false)
    {
        if (path_3d.cols() > 0)
        {
            RobotsFormationInit(DIMENSION, NUM_AGENT);
            pathTrackingInit();
            cout << "path_3d.cols():  " << path_3d.cols() << endl;
            cout << "initialized successfully !!!!!!!" << endl;
            is_initial_multi_msg = true;
        }
        else
        {
            cout << "path_3d.cols():  " << path_3d.cols() << endl;
            cout << "[path_3d] is empty !!!" << endl;
            is_initial_multi_msg = false;
        }
    }
    else
    {
        if (_mode == SINGLE_MODE)
        {
            dynamicPathTracking(); // 启动单机路径追踪
        }
        else if (_mode == FORMATION_MODE)
        {
            dynamicMultiPathPoints(); // 启动编队模式下的路径追踪
        }
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 初始化并跟踪路径v2： 设置机器人的初始位置，并进行实时路径规划 由 [decision]结点 直接调用
 *
 * 输入： 控制模式： 单机模式或编队模式
 *
 * 输出： 无
 */
// void initRobotsFormationAndPlanDynamicPath()
//{
//     if(is_initial_multi_msg == false){
//         cout << "Initialize formation control" << endl;
//         if(path_3d.cols() > 5){
//             // formation initialization
//             formationControlInit(DIMENSION, NUM_AGENT);
//             pathTrackingInit();
//             is_initial_multi_msg = true;
//             VectorXf curWaypoint = virtual_leader_old_3d_path.block(0, point_count, 3, 1);
//             VectorXf tempVec = virtual_leader_old_3d_path.block(0, point_count + 1, 3, 1)
//                     - virtual_leader_old_3d_path.block(0, point_count, 3, 1);;

//            float vs_yaw_temp = atan2(tempVec(1), tempVec(0)) * 180 / M_PI;
//            while( !formationShapeBuilding(curWaypoint, formationShape, vs_yaw_temp, 1, 10, 1) ){
//                // Waiting for formation shape building.
//                cout << "Waiting for formation shape building......" << endl;
//                if(is_initial_multi_msg == false)
//                    break;
//                ros::spinOnce();
//                //loop_rate.sleep();
//            }

//        }else {
//            cout << "the number of whole points is : " << path_3d.cols() <<endl;
//            cout << "[path_3d] is less than 5 !!! Can not build formation shape..." << endl;
//            is_initial_multi_msg = false;
//        }
//    } else {
//        cout << "******************** dynamic path tracking ......*******************" << endl;
//        // dynamicPathTracking();
////            dynamicMultiPathPoints();
//        multiRobotTrackDynamicPath();
//        // singleOutputPathPoints();
//        // outputPathPoints();
//        // AgentsFormation(path_3d.col(0), 1,0,1);
//    }
//}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void initRobotsFormationAndPlanDynamicPath()
{

    if (is_initial_multi_msg == false)
    {
        cout << "Initialize formation control" << endl;
        if (path_3d.cols() > 5)
        {
            // formation initialization
            formationControlInit(DIMENSION, NUM_AGENT);
            pathTrackingInit();

            vs_start_point = virtual_leader_old_3d_path.col(0);
            tempVec = virtual_leader_old_3d_path.col(1) - virtual_leader_old_3d_path.col(0);

            //            vs_start_yaw = atan2(tempVec(1), tempVec(0)) * 180 / M_PI;
            vs_start_yaw = atan2(tempVec(1), tempVec(0));
            is_initial_multi_msg = true;
        }
        else
        {
            cout << "the number of whole points is : " << path_3d.cols() << endl;
            cout << "[path_3d] is less than 5 !!! Can not build formation shape..." << endl;
            is_initial_multi_msg = false;
        }
    }
    else
    {
        /// DEBUG
        //        VectorXf vs_start_temp_point(3);
        //        vs_start_temp_point << 5, 0, 0;
        //        vs_start_point = vs_start_temp_point;

        //        float vs_start_yaw_temp = 0;
        //        vs_start_yaw = vs_start_yaw_temp;

        //        cout << " vs_start_point: " <<  vs_start_point  <<endl;
        ///
        if (!formationShapeBuilding(vs_start_point, formationShape, vs_start_yaw, 1, 10, 1))
        {
            cout << "Waiting for formation shape building......" << endl;
        }
        else
        {
            cout << "******************** dynamic path tracking ......*******************" << endl;
            // dynamicPathTracking();
            // dynamicMultiPathPoints();
            multiRobotTrackDynamicPath();
            // singleOutputPathPoints();
            // outputPathPoints();
            // AgentsFormation(path_3d.col(0), 1,0,1);
        }
    }
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

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

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 保存机器人运行过程中的位姿数据
 *
 * 输入： 无
 *
 * 输出： 无
 */
// 保存默认数据
void saveData()
{
    save_data.write_data();
}

// 保存机器人自己的数据
void saveSingleRobotData(int _self_id, float _robot_pos_x, float _robot_pos_y, float _robot_pos_z, float _robot_yaw)
{
    save_data.write_data(_self_id, _robot_pos_x, _robot_pos_y, _robot_pos_z, _robot_yaw);
}

void saveSingleRobotAllDatas(int _self_id, float _robot_pos_x, float _robot_pos_y, float _robot_pos_z, float _robot_velocity_x, float _robot_velocity_y, float _x_linear_acceleration, float _y_linear_acceleration, float _robot_yaw, float _yaw_velocity)
{
    // save_data.write_data( _self_id,  _robot_pos_x,  _robot_pos_y,  _robot_pos_z, _robot_velocity_x,  _robot_velocity_y, _x_linear_acceleration,_y_linear_acceleration,_robot_yaw, _yaw_velocity);
}

// 保存3台机器人的数据
void saveThreeRobotsData(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                         float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                         float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw)
{
    save_data.write_data(_robot1_pos_x, _robot1_pos_y, _robot1_pos_z, _robot1_yaw,
                         _robot2_pos_x, _robot2_pos_y, _robot2_pos_z, _robot2_yaw,
                         _robot3_pos_x, _robot3_pos_y, _robot3_pos_z, _robot3_yaw);
}

// 保存5台机器人的数据
void saveFiveRobotsData(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                        float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                        float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw,
                        float _robot4_pos_x, float _robot4_pos_y, float _robot4_pos_z, float _robot4_yaw,
                        float _robot5_pos_x, float _robot5_pos_y, float _robot5_pos_z, float _robot5_yaw)
{
    save_data.write_data(_robot1_pos_x, _robot1_pos_y, _robot1_pos_z, _robot1_yaw,
                         _robot2_pos_x, _robot2_pos_y, _robot2_pos_z, _robot2_yaw,
                         _robot3_pos_x, _robot3_pos_y, _robot3_pos_z, _robot3_yaw,
                         _robot4_pos_x, _robot4_pos_y, _robot4_pos_z, _robot4_yaw,
                         _robot5_pos_x, _robot5_pos_y, _robot5_pos_z, _robot5_yaw);
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

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
float fAlphaInVLS = 120;
float fLeftBetaInVLS = 0, fRightBetaInVLS = 0;
float fLeftLinkageLengthInVLS = 1.2;
float fRightLinkageLengthInVLS = 1.2;

Eigen::Vector3f vls_reference_point(0, 0, 0);
VirtualLinkageStructure vls(0, vls_reference_point, fAlphaInVLS, fLeftBetaInVLS, fRightBetaInVLS,
                            fLeftLinkageLengthInVLS, fRightLinkageLengthInVLS);

/********************************* Matrix for formation control ******************************/
Eigen::MatrixXf curRobotsPos8(DIMENSION, NUM_AGENT);
Eigen::MatrixXf estRobotVel8(DIMENSION, NUM_AGENT);
Eigen::MatrixXf expRobotsPos8(DIMENSION, NUM_AGENT);
Eigen::VectorXf expRobotPos8(DIMENSION);
Eigen::MatrixXf minusLaplace8(NUM_AGENT, NUM_AGENT);

/******************************** Artificial potenrial function*******************************/
APF apfRobot8;

/******************************** Consensus control ******************************************/
C_DistrInfoConcencus consensusControl; // 分布式一致性控制器

/**************************** robot position from eight camera   *****************************/
Eigen::MatrixXf robotsInfofromCamera8(DIMENSION, NUM_AGENT);
Eigen::VectorXf isrobot(NUM_AGENT);
Eigen::VectorXf robotContour(NUM_AGENT);
float robot1_pos_x8, robot1_pos_y8, robot1_pos_z8, robot1_vel_x8, robot1_vel_y8, robot1_vel_z8;
float robot2_pos_x8, robot2_pos_y8, robot2_pos_z8, robot2_vel_x8, robot2_vel_y8, robot2_vel_z8;
float robot3_pos_x8, robot3_pos_y8, robot3_pos_z8, robot3_vel_x8, robot3_vel_y8, robot3_vel_z8;

float camera8_roll, camera8_yaw, camera8_pitch, camera8_roll_vel, camera8_yaw_vel, camera8_pitch_vel;
// asrobot::multi_robot8 multiRobotInfo;

Eigen::VectorXf robot1PosInVL(DIMENSION);
Eigen::VectorXf robot3PosInVL(DIMENSION);
Eigen::AngleAxisf rotation_vector;

/******************************   kalman filter for pos_z   **********************************/
C_KalmanFilter_v2 KF_v2_robot1_z; // 3台机器人各自的 z 坐标滤波器
C_KalmanFilter_v2 KF_v2_robot2_z;
C_KalmanFilter_v2 KF_v2_robot3_z;
VectorXf pressure_sensor_z(3);
VectorXf cur_sensors_z_1(2);
VectorXf cur_sensors_z_3(2);

/******************************   kalman filter for position and velocity   **********************************/
RNG rng1;
const int stateNum = 6;
const int measureNum = 3;
const int winHeight = 10;
const int winWidth = 17;
KalmanFilter _kf_robot1(stateNum, measureNum, 0);
Mat measurement_robot1 = Mat::zeros(measureNum, 1, CV_32F);
KalmanFilter _kf_robot2(stateNum, measureNum, 0);
Mat measurement_robot2 = Mat::zeros(measureNum, 1, CV_32F);
KalmanFilter _kf_robot3(stateNum, measureNum, 0);
Mat measurement_robot3 = Mat::zeros(measureNum, 1, CV_32F);

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  2. Kalman Filter for position and velocity
 *   按" ctrl + F " 对应查找
 *
 *     1. kalmanfilter_init
 *     2. kalmanfilter_estimation for formation(FORMATION_MODE_1_2)
 *     3. kalmanfilter_estimation for formation(FORMATION_MODE_2_1)
 */

void kalmanfilter_init()
{
    _kf_robot1.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                                   0, 1, 0, 0, 1, 0,
                                   0, 0, 1, 0, 0, 1,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1);
    setIdentity(_kf_robot1.measurementMatrix);                      // 测量矩阵H
    setIdentity(_kf_robot1.processNoiseCov, Scalar::all(1e-5));     // 系统噪声方差矩阵Q
    setIdentity(_kf_robot1.measurementNoiseCov, Scalar::all(1e-1)); // 测量噪声方差矩阵R
    setIdentity(_kf_robot1.errorCovPost, Scalar::all(1));           // 后验错误估计协方差矩阵P
    rng1.fill(_kf_robot1.statePost, RNG::UNIFORM,
              0, winHeight > winWidth ? winWidth : winHeight); // 初始状态值x(0)

    _kf_robot2.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                                   0, 1, 0, 0, 1, 0,
                                   0, 0, 1, 0, 0, 1,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1);
    setIdentity(_kf_robot2.measurementMatrix);                      // 测量矩阵H
    setIdentity(_kf_robot2.processNoiseCov, Scalar::all(1e-5));     // 系统噪声方差矩阵Q
    setIdentity(_kf_robot2.measurementNoiseCov, Scalar::all(1e-1)); // 测量噪声方差矩阵R
    setIdentity(_kf_robot2.errorCovPost, Scalar::all(1));           // 后验错误估计协方差矩阵P
    rng1.fill(_kf_robot2.statePost, RNG::UNIFORM,
              0, winHeight > winWidth ? winWidth : winHeight); // 初始状态值x(0)

    _kf_robot3.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
                                   0, 1, 0, 0, 1, 0,
                                   0, 0, 1, 0, 0, 1,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1);
    setIdentity(_kf_robot3.measurementMatrix);                   // 测量矩阵H
    setIdentity(_kf_robot3.processNoiseCov, Scalar::all(1e-5));  // 系统噪声方差矩阵Q
    setIdentity(_kf_robot3.measurementNoiseCov, Scalar::all(1)); // 测量噪声方差矩阵R
    setIdentity(_kf_robot3.errorCovPost, Scalar::all(1));        // 后验错误估计协方差矩阵P
    rng1.fill(_kf_robot3.statePost, RNG::UNIFORM,
              0, winHeight > winWidth ? winWidth : winHeight); // 初始状态值x(0)
}

void kalmanfilter_estimation_1_2(MatrixXf &curRobotsPosEst_, MatrixXf &curRobotsVelEst_)
{
    Mat prediction_state;
    if (isrobot(0))
    {
        ROS_INFO("Robot 1: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 0), curRobotsPosEst_(1, 0),
                 curRobotsPosEst_(2, 0));
        measurement_robot1.at<float>(0) = curRobotsPosEst_(0, 0);
        measurement_robot1.at<float>(1) = curRobotsPosEst_(1, 0);
        measurement_robot1.at<float>(2) = curRobotsPosEst_(2, 0);
        prediction_state = _kf_robot1.predict();
        curRobotsPosEst_(0, 0) = prediction_state.at<float>(0);
        curRobotsPosEst_(1, 0) = prediction_state.at<float>(1);
        curRobotsPosEst_(2, 0) = prediction_state.at<float>(2);
        curRobotsVelEst_(0, 0) = 200 * prediction_state.at<float>(3);
        curRobotsVelEst_(1, 0) = 200 * prediction_state.at<float>(4);
        curRobotsVelEst_(2, 0) = 200 * prediction_state.at<float>(5);
        _kf_robot1.correct(measurement_robot1);
        ROS_INFO("kf Robot 1: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 0), curRobotsPosEst_(1, 0),
                 curRobotsPosEst_(2, 0));
        ROS_INFO("kf Robot 1: vel_x=%f, vel_y=%f, vel_z=%f", curRobotsVelEst_(0, 0), curRobotsVelEst_(1, 0),
                 curRobotsVelEst_(2, 0));
    }

    if (isrobot(2))
    {
        ROS_INFO("Robot 3: cur_x= %f, cur_y= %f, cur_z= %f", curRobotsPosEst_(0, 2), curRobotsPosEst_(1, 2),
                 curRobotsPosEst_(2, 1));
        measurement_robot3.at<float>(0) = (float)curRobotsPosEst_(0, 2);
        measurement_robot3.at<float>(1) = (float)curRobotsPosEst_(1, 2);
        measurement_robot3.at<float>(2) = (float)curRobotsPosEst_(2, 2);
        prediction_state = _kf_robot2.predict();
        curRobotsPosEst_(0, 2) = prediction_state.at<float>(0);
        curRobotsPosEst_(1, 2) = prediction_state.at<float>(1);
        curRobotsPosEst_(2, 2) = prediction_state.at<float>(2);
        curRobotsVelEst_(0, 2) = 200 * prediction_state.at<float>(3);
        curRobotsVelEst_(1, 2) = 200 * prediction_state.at<float>(4);
        curRobotsVelEst_(2, 2) = 200 * prediction_state.at<float>(5);
        _kf_robot2.correct(measurement_robot3);
        ROS_INFO("kf Robot 3: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 2), curRobotsPosEst_(1, 2),
                 curRobotsPosEst_(2, 2));
        ROS_INFO("kf Robot 3: vel_x=%f, vel_y=%f, vel_z=%f", curRobotsVelEst_(0, 2), curRobotsVelEst_(1, 2),
                 curRobotsVelEst_(2, 2));
    }
}

void kalmanfilter_estimation_2_1(MatrixXf &curRobotsPosEst_, MatrixXf &curRobotsVelEst_)
{
    Mat prediction_state;

    ROS_INFO("Robot 1: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 0), curRobotsPosEst_(1, 0),
             curRobotsPosEst_(2, 0));
    measurement_robot1.at<float>(0) = curRobotsPosEst_(0, 0);
    measurement_robot1.at<float>(1) = curRobotsPosEst_(1, 0);
    measurement_robot1.at<float>(2) = curRobotsPosEst_(2, 0);
    prediction_state = _kf_robot1.predict();
    curRobotsPosEst_(0, 0) = prediction_state.at<float>(0);
    curRobotsPosEst_(1, 0) = prediction_state.at<float>(1);
    curRobotsPosEst_(2, 0) = prediction_state.at<float>(2);
    curRobotsVelEst_(0, 0) = 20 * prediction_state.at<float>(3);
    curRobotsVelEst_(1, 0) = 20 * prediction_state.at<float>(4);
    curRobotsVelEst_(2, 0) = 20 * prediction_state.at<float>(5);
    _kf_robot1.correct(measurement_robot1);
    ROS_INFO("kf Robot 1: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 0), curRobotsPosEst_(1, 0),
             curRobotsPosEst_(2, 0));
    ROS_INFO("kf Robot 1: vel_x=%f, vel_y=%f, vel_z=%f", curRobotsVelEst_(0, 0), curRobotsVelEst_(1, 0),
             curRobotsVelEst_(2, 0));

    ROS_INFO("Robot 3: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 2), curRobotsPosEst_(1, 2),
             curRobotsPosEst_(2, 2));
    measurement_robot3.at<float>(0) = curRobotsPosEst_(0, 2);
    measurement_robot3.at<float>(1) = curRobotsPosEst_(1, 2);
    measurement_robot3.at<float>(2) = curRobotsPosEst_(2, 2);
    prediction_state = _kf_robot2.predict();
    curRobotsPosEst_(0, 2) = prediction_state.at<float>(0);
    curRobotsPosEst_(1, 2) = prediction_state.at<float>(1);
    curRobotsPosEst_(2, 2) = prediction_state.at<float>(2);
    curRobotsVelEst_(0, 2) = 20 * prediction_state.at<float>(3);
    curRobotsVelEst_(1, 2) = 20 * prediction_state.at<float>(4);
    curRobotsVelEst_(2, 2) = 20 * prediction_state.at<float>(5);
    _kf_robot2.correct(measurement_robot3);
    ROS_INFO("kf Robot 3: cur_x=%f, cur_y=%f, cur_z=%f", curRobotsPosEst_(0, 2), curRobotsPosEst_(1, 2),
             curRobotsPosEst_(2, 2));
    ROS_INFO("kf Robot 3: vel_x=%f, vel_y=%f, vel_z=%f", curRobotsVelEst_(0, 2), curRobotsVelEst_(1, 2),
             curRobotsVelEst_(2, 2));
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  3. Transform to world
 *
 *     1. Transform to world(Virtual Linkage structure)
 *
 */

Eigen::VectorXf transPos2VL(Eigen::Vector3f robot1_, Eigen::VectorXf robot2_, float roll, float yaw, float pitch)
{

    Eigen::Vector3f eulerAngle(M_PI * yaw / 180, M_PI * pitch / 180,
                               M_PI * roll / 180);
    Eigen::AngleAxisf rollAngle(AngleAxisf(eulerAngle[2], Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(AngleAxisf(eulerAngle[1], Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(AngleAxisf(eulerAngle[0], Vector3f::UnitZ()));
    rotation_vector = yawAngle * pitchAngle * rollAngle;
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity(); // 虽然称为 3f ,实质上是 4*4 的矩阵
    T.rotate(rotation_vector);                           // 按照 rotation_vector 进行旋转
    T.pretranslate(robot1_);                             // 把平移向量设成 (1,3,4)

    Eigen::VectorXf robot2_Exten(4, 1);
    Eigen::VectorXf vec(4, 1);
    if (4 != robot2_.cols())
    {
        robot2_Exten(0) = robot2_(0);
        robot2_Exten(1) = robot2_(1);
        robot2_Exten(2) = robot2_(2);
        robot2_Exten(3) = 1;
        vec = T.matrix().inverse() * robot2_Exten;
    }
    else
    {
        vec = T.matrix().inverse() * robot2_;
    }
    Eigen::VectorXf temp(3, 1);
    temp(0) = vec(0);
    temp(1) = vec(1);
    temp(2) = vec(2);
    return temp;
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  4. multiple stereo camera-based formation control function
 *
 *     1. initMultiCameraFormationCtrl
 *     2. multiCameraFormationCtrl
 */
void initMultiCamera8FormationCtrl(int dimension_, int robotNum_)
{

    curRobotsPos8.setZero(dimension_, robotNum_);
    expRobotsPos8.setZero(dimension_, robotNum_);
    expRobotPos8.setZero(dimension_, 1);
    minusLaplace8.setZero(robotNum_, robotNum_);

    switch (robotNum_)
    {
    case 3:
    {
        minusLaplace8 << -1, 1, 0,
            0, -1, 1,
            1, 0, -1;
    };
    break;
    case 5:
    {
        minusLaplace8 << -1, 1, 0, 0, 0,
            1, -2, 1, 0, 0,
            0, 1, -2, 1, 0,
            0, 0, 1, -2, 1,
            0, 0, 0, 1, -1;
    };
    break;
    default:
    {
        cout << "please intialize laplace correctly, [robotNum_] is 3 or 5" << endl;
    }
    }

    // 1. 虚拟连杆初始化
    // 在定义时已完成初始化
    vls.setVLS_Angle_RYP(roll, yaw, pitch);
    expRobotsPos8 = vls.calcVL_Pos();

    // 2. 人工势场初始化
    apfRobot8.set_krep(0.1);
    apfRobot8.set_r(0.5);
    apfRobot8.set_r0(2);

    // 3. 一致性编队
    curRobotsPos8 = expRobotPos8;
    consensusControl.InitDistrConcensusPmts(self_id, CONV_SPEED, minusLaplace8, curRobotsPos8);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*@
 * 功能： 编队路径规划：编队运行时，对机器人期望路径点进行规划
 *
 * 输入：
 *   virtual_leader_pos:    虚拟leader的位置
 *   i_con :                一致性输出权重
 *   i_apfi:                人工势场内部防撞的权重
 *   s     :                步长， s越大，规划的期望点离目标点越远
 * 输出： 无
 */
Eigen::VectorXf multiCamera8FormationCtrl(VectorXf virtual_leader_pos, float i_con, float i_apfi, float s)
{

    VectorXf consensusPosOutput; // 一致性输出的位置
    VectorXf apfPosOutput;       // 虚拟结构输出的位置
    VectorXf posOutput(3, 1);    // 总的位置

    // 虚拟连杆结构，计算虚拟leader位置和人工势场
    //    vls.

    //    vsRobots.vs_pointInWorld.x = virtual_leader_pos(0);
    //    vsRobots.vs_pointInWorld.y = virtual_leader_pos(1);
    //    vsRobots.vs_pointInWorld.z = virtual_leader_pos(2);
    //    vsRobots.vs_yaw = vs_curYaw;

    //    vsRobots.calcRobotsPosInTriangularForm();
    //    all_robots_tar_pos = vsRobots.transform2WorldFrame();
    //    cout << " All_robots_tar_pos: " << all_robots_tar_pos << endl;
    //    cout << " vsRobots.vs_yaw" << vsRobots.vs_yaw << endl;

    // 一致性 接收目标点，输出位置
    consensusControl.CalDistrConcensusOutput(curRobotsPos8, expRobotsPos8, consensusPosOutput);
    consensusPosOutput = consensusPosOutput - curRobotsPos8.col(self_id - 1);

    // 人工势场内部防撞 输出位置
    apfPosOutput = apf_agent.apf_InterRepulsion(curRobotsPos8.col(self_id - 1), curRobotsPos8);

    // 人工势场 障碍物防撞 位置输出

    // 总位置输出
    posOutput = i_con * consensusPosOutput + i_apfi * apfPosOutput;

    expRobotPos8 = curRobotsPos8.col(self_id - 1) + s * posOutput;

    return expRobotPos8;
    //    return all_robots_tar_pos.block(0, self_id-1, 3, 1);
    //    return consensusPosOutput;
}

// hcy
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*@  用于docking的函数
 *   按" ctrl + F " 对应查找
 */

// 计算路径点
bool is_initial_docking_path = false;
int path_flag = 0;
void calcDockingPathPoint()
{
    float __start_yaw;
    __start_yaw = atan2(cam_pos_y, cam_pos_x + 0.3) / M_PI * 180;
    // 如果机器人在后半部分区域(-120 - 120)
    if (__start_yaw >= -docking_yaw_limit1 && __start_yaw <= docking_yaw_limit1)
    {
        cout << "######### initial docking path failed !!!!! ##############" << endl;
        is_initial_docking_path = false;
    }
    // 如果在 120-170
    else if ((__start_yaw > docking_yaw_limit1) && (__start_yaw < docking_yaw_limit2))
    {
        doc.planCirclePathLeft(cam_pos_x, cam_pos_y, cam_pos_z);
        is_initial_docking_path = true;
        path_flag = 1;
    }
    // 如果在-170 - -120之间，规划右侧圆形路径
    else if ((__start_yaw > -docking_yaw_limit2) && (__start_yaw < -docking_yaw_limit1))
    {
        doc.planCirclePathRight(cam_pos_x, cam_pos_y, cam_pos_z);
        is_initial_docking_path = true;
        path_flag = 2;
    }
    //  如果在10度以内 规划直线路径
    else if (((__start_yaw >= -180) && (__start_yaw <= -docking_yaw_limit2)) || ((__start_yaw >= docking_yaw_limit2) && (__start_yaw <= 180)))
    {
        doc.planLinePath(cam_pos_x, cam_pos_y, cam_pos_z);
        is_initial_docking_path = true;
        path_flag = 3;
    }
    // 取出路径
    docking_plan_path_points = doc.outputExpPath(); // hy:3x3矩阵
    // 给初始点
    docking_exp_robot_pos = docking_plan_path_points.col(0); // hy:列向量

#if DOCKING_DEBUG
    // 把第一眼识别到的值记录下来
    cam_pos_x_init_var = cam_pos_x;
    cam_pos_y_init_var = cam_pos_y;
    cam_pos_z_init_var = cam_pos_z;
    cam_yaw_init_var = cam_yaw;
#endif
}

// 根据机器人的状况更新路径点
void updateDockingPathPoint()
{
    // hy:机器人在新全局坐标系下的位置
    docking_cur_robot_pos(0) = cam_pos_x;
    docking_cur_robot_pos(1) = cam_pos_y;
    docking_cur_robot_pos(2) = cam_pos_z;

    // hy:机器人期望点位置，是一个向量，从第0列开始
    docking_exp_robot_pos = docking_plan_path_points.col(docking_point_count);

    // 根据距离判断 小于0.1m就更新下一个点
    // if( (sqrt( pow( docking_exp_robot_pos(0) - docking_cur_robot_pos(0) , 2)
    //         + pow(docking_exp_robot_pos(1) - docking_cur_robot_pos(1) , 2) ) <= 0.4) ||
    //         (docking_exp_robot_pos(0) < docking_cur_robot_pos(0)))//hcy:如果机器人在新全局坐标系下的期望x位置
    //                                                                                                                                 //小于实际x位置，也更新下一个点
    if (sqrt(pow(docking_exp_robot_pos(0) - docking_cur_robot_pos(0), 2) + pow(docking_exp_robot_pos(1) - docking_cur_robot_pos(1), 2)) <= 0.4)
    {
        if (docking_point_count < (docking_plan_path_points.cols() - 1)) // hy:如果小于规划点的总数
        {
            docking_point_count++;
        }

        else if (docking_point_count == (docking_plan_path_points.cols() - 1)) // hy:如果等于规划点的总数
        {
            docking_point_count == docking_point_count; // 到达最后一个点的时候 还要++ 那就加多了，再撤回来
        }
    }
    is_initial_docking_path = false; // hcy

    cout << " docking_plan_path_points: " << endl
         << docking_plan_path_points << endl;
    cout << " **********  docking_point_count   ************: " << docking_point_count << endl;
    cout << " docking_exp_robot_pos: " << docking_exp_robot_pos.transpose() << endl;
}

// docking 总流程
void dockingPlanPathProcess()
{
    if (is_initial_docking_path == false)
    {
        calcDockingPathPoint();
    }
    else
    {
        if (path_flag == 1)
        {
            cout << "%%%%%  left circle %%%%%%" << endl;
        }
        else if (path_flag == 2)
        {
            cout << "%%%%%  right circle %%%%%%" << endl;
        }
        else if (path_flag == 3)
        {
            cout << "%%%%%  line %%%%%%" << endl;
        }

#if DOCKING_DEBUG
        cout << "cam_pos_x_init_var: " << cam_pos_x_init_var << endl;
        cout << "cam_pos_y_init_var: " << cam_pos_y_init_var << endl;
        cout << "cam_pos_z_init_var: " << cam_pos_z_init_var << endl;
        cout << "cam_yaw_init_var: " << cam_yaw_init_var << endl;
#endif

        updateDockingPathPoint();
    }
}

// hcy:2代 docking
// 计算路径点
bool is_initial_docking_path_v2 = false;
int path_flag_v2 = 0;
void calcDockingPathPoint_v2()
{
    float __start_yaw;
    __start_yaw = atan2(cam_pos_y, cam_pos_x + 0.3) / M_PI * 180;
    // 如果机器人在后半部分区域
    if (__start_yaw >= -docking_yaw_limit1 && __start_yaw <= docking_yaw_limit1)
    {
        cout << "######### initial docking path failed !!!!! ##############" << endl;
        is_initial_docking_path_v2 = false;
    }
    // 如果在 120-170
    else if ((__start_yaw > docking_yaw_limit1) && (__start_yaw < docking_yaw_limit2))
    {
        doc_v2.planCirclePathLeft(cam_pos_x, cam_pos_y, cam_pos_z);
        is_initial_docking_path_v2 = true;
        path_flag_v2 = 1;
    }
    // 如果在-170 - -120之间，规划右侧圆形路径
    else if ((__start_yaw > -docking_yaw_limit2) && (__start_yaw < -docking_yaw_limit1))
    {
        doc_v2.planCirclePathRight(cam_pos_x, cam_pos_y, cam_pos_z);
        is_initial_docking_path_v2 = true;
        path_flag_v2 = 2;
    }
    //  如果在10度以内 规划直线路径
    else if (((__start_yaw >= -180) && (__start_yaw <= -docking_yaw_limit2)) || ((__start_yaw >= docking_yaw_limit2) && (__start_yaw <= 180)))
    {
        doc_v2.planLinePath(cam_pos_x, cam_pos_y, cam_pos_z);
        is_initial_docking_path_v2 = true;
        path_flag_v2 = 3;
    }

    // 将最后的直线段轨迹规划出来
    doc_v2.planEndLinePath();
    // 取出路径
    docking_plan_path_points = doc_v2.outputExpPath();
    // 给初始点
    docking_exp_robot_pos = docking_plan_path_points.col(0);

#if DOCKING_DEBUG
    // 把第一眼识别到的值记录下来
    cam_pos_x_init_var = cam_pos_x;
    cam_pos_y_init_var = cam_pos_y;
    cam_pos_z_init_var = cam_pos_z;
    cam_yaw_init_var = cam_yaw;
#endif
}

// 根据机器人的状况更新路径点
void updateDockingPathPoint_v2()
{
    docking_cur_robot_pos(0) = cam_pos_x;
    docking_cur_robot_pos(1) = cam_pos_y;
    docking_cur_robot_pos(2) = cam_pos_z;

    docking_exp_robot_pos = docking_plan_path_points.col(docking_point_count_v2);

    if (docking_point_count_v2 > doc_v2.heli_num_point)
    {
        path_flag_v2 = 4;
    }
    cout << "docking_point_count_v2: " << docking_point_count_v2 << endl;
    cout << "docking_plan_path_points.cols(): " << docking_plan_path_points.cols() << endl;
    if (docking_point_count_v2 < (docking_plan_path_points.cols() - 1))
    {
        docking_point_count_v2++; // 到达最后一个点的时候 还要++ 那就加多了
    }
    else if (docking_point_count_v2 == (docking_plan_path_points.cols() - 1))
    {
        ;
    }

    cout << " docking_plan_path_points: " << endl
         << docking_plan_path_points << endl;
    cout << " **********  docking_point_count   ************: " << docking_point_count_v2 << endl;
    cout << " docking_exp_robot_pos: " << docking_exp_robot_pos.transpose() << endl;
}

// docking 总流程
void dockingPlanPathProcess_v2()
{
    if (is_initial_docking_path_v2 == false)
    {
        calcDockingPathPoint_v2();
    }
    else
    {
        if (path_flag_v2 == 1)
        {
            cout << "%%%%%  left circle %%%%%%" << endl;
        }
        else if (path_flag_v2 == 2)
        {
            cout << "%%%%%  right circle %%%%%%" << endl;
        }
        else if (path_flag_v2 == 3)
        {
            cout << "%%%%%  line %%%%%%" << endl;
        }
        else if (path_flag_v2 == 4)
        {
            cout << "%%%%%  end line %%%%%%" << endl;
        }

#if DOCKING_DEBUG
        cout << "cam_pos_x_init_var: " << cam_pos_x_init_var << endl;
        cout << "cam_pos_y_init_var: " << cam_pos_y_init_var << endl;
        cout << "cam_pos_z_init_var: " << cam_pos_z_init_var << endl;
        cout << "cam_yaw_init_var: " << cam_yaw_init_var << endl;
#endif

        updateDockingPathPoint_v2();
    }
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
