
#include "../../include/asrobot/f_multi_robot_plan.h"

#define PUBLISH_SWITCH 1   // 发布开关
#define SUBSCRIBE_SWITCH 1 // 接收开关
#define TIMER_SWITCH 1     // 定时器开关

#define MOTION_PUB_PRINT_SWITCH 0     // hcy
#define BIONOCULAR_SUB_PRINT_SWITCH 0 // hcy
#define DOCKING_SWITCH 0              // hcy
#define TRACKING_SWITCH 0             // hcy
#define SINGLE_ROBOT_DEBUG_SWITCH 1   // 深度、yaw单独调节开这个

#define TRACKING_ARUCO_SWITCH 1 // ljl
/***************************************************************************************/
/*                                                                                     */
/*                                 全局发布变量                                          */
/*                                                                                     */
/***************************************************************************************/
#if PUBLISH_SWITCH

ros::Publisher mc_pub; // 发布单体机器人各种信息 如位置，速度，航向角等
// ros::Publisher multi_pub;           // 发布自己的位置
ros::Publisher achievedWaypoint; // 发布运行的点数， 用来进行点数同步
ros::Publisher trajectory_pub;
ros::Publisher muli_RobotPosition_pub;

Eigen::MatrixXf robot_pos(3, 2);  // ljl存储gazebo的位置
Eigen::MatrixXf robot1_pos(3, 2); // ljl存储gazebo的位置
Eigen::MatrixXf robot2_pos(3, 2); // ljl存储gazebo的位置

#endif

std::string robot_name;
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 全局消息变量（发布）                                    */
/*                                                                                     */
/***************************************************************************************/
#if PUBLISH_SWITCH

asrobot::mc motion;                     // 包含该机器人的位置，速度，航向等
asrobot::achievedWaypoint waypoint_msg; // 包含所有机器人到达的点数
asrobot::mul_ref_trajectory mul_ref_trajectory_msg;
asrobot::quadrotor2robot multi_RObotPosition_msg;
// asrobot::multirobot multiRobotInfo;             // 发布机器人自己的位置s

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 全局订阅变量                                          */
/*                                                                                     */
/***************************************************************************************/
#if SUBSCRIBE_SWITCH

ros::Subscriber sub_path_3d;             // 订阅无人机发过来的路径
ros::Subscriber sub_path_threeSpline;    // 订阅无人机发过来的轨迹
ros::Subscriber quadrotor_sub;           // 订阅无人机发过来的所有机器人位置
image_transport::Subscriber sub_contour; // 订阅图片轮廓
ros::Subscriber achiv_sub;               // 订阅其他机器人发布的点数
ros::Subscriber dep_sub;                 // 订阅深度计发来的信息
ros::Subscriber imu_sub;                 // 订阅惯导发来的信息
ros::Subscriber sub_cmd;                 // 订阅手动调试指令
ros::Subscriber model_states_sub;        // 订阅 gazebo 发布的机器人状态

ros::Subscriber biono_camera_sub; // 订阅双目//hcy
ros::Subscriber down_camera_sub;  // 订阅下视//ljl
#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 定时器                                               */
/*                                                                                     */
/***************************************************************************************/
#if TIMER_SWITCH

ros::Timer timer_saveData;

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 消息发布函数                                          */
/*                                                                                     */
/***************************************************************************************/
#if PUBLISH_SWITCH

void UpdateTargetPoint()
{
    float switch_radius = 0.5;

    cout << "-------------------------" << path_points.cols() << "---------------------" << endl;
    if (path_points.cols() > 0)
        for (int num = 0; num < path_points.cols(); num++)
        {
            target_point(0) = path_points(0, num);
            target_point(1) = path_points(1, num);

            if (sqrt((target_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) * (target_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) + (target_point(1) - robotsInfofromQuadrotor(1, self_id - 1)) * (target_point(1) - robotsInfofromQuadrotor(1, self_id - 1))) < switch_radius)
                continue;
            else
                break;
        }
    else
        target_point << 0, 0;
    cout << "point_x" << target_point(0) << "point_y" << target_point(0) << endl;
}

/*@
 * * * * * * * * * * * * * *  向单机控制器发送机器人位姿信息  * * * * * * * * * * * * * * * * * * *
 */
// 所有视觉数据保留源图时间，接收墙钟另存，防止暂停后把旧缓存当作新测量。
bool aruco_valid = false;
ros::Time aruco_stamp, feedback_stamp;
ros::SteadyTime aruco_received_wall, feedback_received_wall;
double body_vx = 0, body_vy = 0, body_vz = 0, body_yaw_deg = 0, body_yaw_rate = 0;
bool feedback_valid = false;

void motion_control_pub(int _mode)
{
    switch (_mode)
    {
    case SINGLE_MODE:
    {
        if (virtual_leader_old_3d_path.cols() <= 0)
        {
            cout << "[the virtual_leader_old_3d_path] is empty !!!" << endl;
        }
        else
        {
            if (point_count < virtual_leader_old_3d_path.cols())
            {
                motion.e_x = virtual_leader_old_3d_path(0, point_count);
                motion.e_y = virtual_leader_old_3d_path(1, point_count);
                motion.e_z = 0;
                motion.e_navyaw = atan2(virtual_leader_old_3d_path(1, point_count) - robotsInfofromQuadrotor(1, self_id - 1), virtual_leader_old_3d_path(0, point_count) - robotsInfofromQuadrotor(0, self_id - 1)) * 57.3;
            }
            if (point_count == virtual_leader_old_3d_path.cols())
            {
                motion.e_x = virtual_leader_old_3d_path(0, point_count - 1);
                motion.e_y = virtual_leader_old_3d_path(1, point_count - 1);
                motion.e_z = 0;
                motion.e_navyaw = atan2(virtual_leader_old_3d_path(1, point_count - 1) - robotsInfofromQuadrotor(1, self_id - 1), virtual_leader_old_3d_path(0, point_count - 1) - robotsInfofromQuadrotor(0, self_id - 1)) * 57.3;
            }

            motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
            motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
            motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1);
            motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1);
            motion.c_z = -depth;
            motion.c_navyaw = yaw;
            motion.c_navroll = roll;
            motion.c_navpitch = pitch;
            motion.c_navyawvelocity = yaw_vel_filter_output;
            motion.c_acceleration_x = x_linear_acceleration;

            cout << "self_id: " << self_id << endl;
            cout << "e_x: " << motion.e_x << "    "
                 << "e_y:  " << motion.e_y << endl;
            cout << "c_x: " << motion.c_x << "    "
                 << "c_vx: " << motion.c_speedx << endl;
            cout << "c_y: " << motion.c_y << "    "
                 << "c_vy: " << motion.c_speedy << endl;
            cout << "c_z: " << motion.c_z << "    "
                 << "c_yaw: " << motion.c_navyaw << endl;
        }
    };
    break;

    case FORMATION_MODE:
    {
        if (path_3d_exp_point.rows() <= 0)
        {
            cout << " [path_3d_exp_point] is empty !!! " << endl;
        }
        else
        {
            motion.e_x = path_3d_exp_point(0);
            motion.e_y = path_3d_exp_point(1);
            motion.e_z = 0;
            motion.e_navyaw = atan2(path_3d_exp_point(1) - robotsInfofromQuadrotor(1, self_id - 1), path_3d_exp_point(0) - robotsInfofromQuadrotor(0, self_id - 1)) * 57.3;

            motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
            motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
            motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1);
            motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1);
            motion.c_z = -depth;
            motion.c_navyaw = yaw;
            motion.c_navroll = roll;
            motion.c_navpitch = pitch;
            motion.c_navyawvelocity = yaw_vel_filter_output;
            cout << "self_id: " << self_id << endl;
            cout << "e_x: " << motion.e_x << "    "
                 << "e_y:  " << motion.e_y << endl;
            cout << "c_x: " << motion.c_x << "    "
                 << "c_vx: " << motion.c_speedx << endl;
            cout << "c_y: " << motion.c_y << "    "
                 << "c_vy: " << motion.c_speedy << endl;
            cout << "c_z: " << motion.c_z << "    "
                 << "c_yaw: " << motion.c_navyaw << endl;
            cout << "path_3d_exp_point: " << path_3d_exp_point.transpose() << endl;
        }
    };
    break;

    case DEBUG_MODE:
    {

        motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
        motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
        motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1); // this is the speed of the robot in the x-axis of the world frame
        motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1); // this is the speed of the robot in the y-axis of the world frame
        motion.c_z = -depth;
        motion.c_navyaw = yaw;
        motion.c_navroll = roll;
        motion.c_navpitch = pitch;
        motion.c_navyawvelocity = yaw_vel_filter_output;
        motion.c_acceleration_x = x_linear_acceleration;

        cout << "self_id: " << self_id << endl;
        cout << "e_x: " << motion.e_x << "    "
             << "e_y:  " << motion.e_y << endl;
        cout << "c_x: " << motion.c_x << "    "
             << "c_vx: " << motion.c_speedx << endl;
        cout << "c_y: " << motion.c_y << "    "
             << "c_vy: " << motion.c_speedy << endl;
        cout << "c_z: " << motion.c_z << "    "
             << "c_yaw: " << motion.c_navyaw << endl;
        cout << "path_3d_exp_point: " << path_3d_exp_point.transpose() << endl;
    };
    break;

    case SINGLE_PATH_TRACK:
    {

        motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
        motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
        motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1); // this is the speed of the robot in the x-axis of the world frame
        motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1); // this is the speed of the robot in the y-axis of the world frame
        motion.c_z = -depth;
        motion.c_navyaw = yaw;
        motion.c_navroll = roll;
        motion.c_navpitch = pitch;
        motion.c_navyawvelocity = yaw_vel_filter_output;
        motion.c_acceleration_x = x_linear_acceleration;

        motion.e_x = target_point(0);
        motion.e_y = target_point(1);
        motion.e_z = -0.2;
        motion.e_navyaw = atan2(motion.e_y - robotsInfofromQuadrotor(1, self_id - 1), motion.e_x - robotsInfofromQuadrotor(0, self_id - 1)) * 57.3;

        cout << "self_id: " << self_id << endl;
        cout << "e_x: " << motion.e_x << "    "
             << "e_y:  " << motion.e_y << endl;
        cout << "c_x: " << motion.c_x << "    "
             << "c_vx: " << motion.c_speedx << endl;
        cout << "c_y: " << motion.c_y << "    "
             << "c_vy: " << motion.c_speedy << endl;
        cout << "c_z: " << motion.c_z << "    "
             << "c_yaw: " << motion.c_navyaw << endl;
        cout << "path_3d_exp_point: " << path_3d_exp_point.transpose() << endl;
    };
    break;

    case SINGLE_ROBOT_DEBUG: // hy:（重要）yaw和深度调节用
    {
        // motion.c_x = robotsInfofromQuadrotor(0, self_id - 1);
        // motion.c_y = robotsInfofromQuadrotor(1, self_id - 1);
        // motion.c_speedx = robotsInfofromQuadrotor(2, self_id - 1);
        // motion.c_speedy = robotsInfofromQuadrotor(3, self_id - 1);
        // motion.c_z = -depth/100000;     // 深度值去反 除100000

        motion.c_x = robot_pos(0, 0);
        motion.c_speedx = robot_pos(0, 1);
        motion.c_y = robot_pos(1, 0);
        motion.c_speedy = robot_pos(1, 1);
        motion.c_z = -depth; // hcy
        motion.c_speedz = robot_pos(2, 1);

        motion.c_calculatedz = -vz;
        motion.c_navyaw = yaw; // 取反 再发送，这样逆时针就增大了
        motion.c_navroll = roll;
        motion.c_navpitch = pitch;

        motion.e_z = -1.0;

#if MOTION_PUB_PRINT_SWITCH
        cout << "self_id: " << self_id << "    SINGLE_ROBOT_DEBUG" << endl;
        cout << "e_x: " << motion.e_x << "    " << "e_y:  " << motion.e_y << endl;
        cout << "e_z: " << motion.e_z << "    " << "e_yaw: " << motion.e_navyaw << endl;
        cout << "c_x: " << motion.c_x << "    " << "c_vx: " << motion.c_speedx << endl;
        cout << "c_y: " << motion.c_y << "    " << "c_vy: " << motion.c_speedy << endl;
        cout << "c_z: " << motion.c_z << "    " << "c_yaw: " << motion.c_navyaw << endl;
#endif
    };
    break;

    case DOCKING_MODE: // hcy
    {
        motion.cam_e_x = docking_exp_robot_pos(0); // hy:三组新全局坐标系下固定的规划点
        motion.cam_e_y = docking_exp_robot_pos(1);
        motion.cam_e_z = docking_exp_robot_pos(2);
        motion.cam_e_yaw = atan2(docking_exp_robot_pos(1) - docking_cur_robot_pos(1) // hy:根据固定的规划点和
                                 ,
                                 docking_exp_robot_pos(0) - docking_cur_robot_pos(0)) *
                           57.3;                   // 机器人在新全局坐标系下
                                                   // 的坐标算出来的yaw
                                                   // 机器人在规划点左侧时为负，
                                                   // 右侧时为正 //e_yaw
        motion.cam_c_x = docking_cur_robot_pos(0); // hy:即cam_pos_x，机器人在新全局坐标系下的x   //c_x
        motion.cam_c_y = docking_cur_robot_pos(1); // hy:即cam_pos_y，机器人在新全局坐标系下的y
        motion.cam_c_z = docking_cur_robot_pos(2); // hy:即cam_pos_z，机器人在新全局坐标系下的z
        motion.cam_c_yaw = cam_yaw;                // hy:即cam_yaw，视觉得出的yaw    //c_yaw

        motion.c_depthvalue = depth;
        motion.heli_traj_radius = doc_v2.circle_radius;

#if MOTION_PUB_PRINT_SWITCH
        cout << "self_id: " << self_id << "    DOCKING_MODE" << endl;
        cout << "e_x: " << motion.cam_e_x << "    " << "e_y:  " << motion.cam_e_y << endl;
        cout << "e_z: " << motion.cam_e_z << "    " << "e_yaw:  " << motion.cam_e_yaw << endl;
        cout << "c_x: " << motion.cam_c_x << "    " << "c_y:  " << motion.cam_c_y << endl;
        cout << "c_z: " << motion.cam_c_z << "    " << "c_yaw: " << motion.cam_c_yaw << endl;
#endif
    };
    break;

    case TRACKING_MODE: // hcy
    {
        motion.cam_e_x = cam_pos_x;
        motion.cam_e_y = cam_pos_y;
        motion.cam_e_z = cam_pos_z;
        motion.cam_e_yaw = cam_yaw;

        motion.cam_c_x = 0;
        motion.cam_c_y = 0;
        motion.cam_c_z = 0;
        motion.cam_c_yaw = 0;
#if MOTION_PUB_PRINT_SWITCH
        cout << "e_x: " << motion.cam_e_x << "    " << "e_y:  " << motion.cam_e_y << endl;
        cout << "e_z: " << motion.cam_e_z << "    " << "e_yaw:  " << motion.cam_e_yaw << endl;
        cout << "c_x: " << motion.cam_c_x << "    " << "c_y:  " << motion.cam_c_y << endl;
        cout << "c_z: " << motion.cam_c_z << "    " << "c_yaw: " << motion.cam_c_yaw << endl;
#endif
    };
    break;

    case TRACKING_ARUCO_MODE: // ljl
    {
        motion.cam_e_x = aruco_pos_x;
        motion.cam_e_y = aruco_pos_y;
        motion.cam_e_z = aruco_pos_z;
        motion.cam_e_yaw = aruco_yaw;
        // 采用世界坐标
        //  motion.cam_c_x = robot_pos(0, 0);
        //  motion.cam_c_y = robot_pos(1, 0);
        //  motion.cam_c_z = robot_pos(2, 0);
        //  motion.cam_c_yaw = yaw;
        // 采用相对坐标
        motion.cam_c_x = 0;
        motion.cam_c_y = 0;
        motion.cam_c_z = 0;
        motion.cam_c_yaw = 0;
        motion.c_x = robot_pos(0, 0); motion.c_y = robot_pos(1, 0); motion.c_z = robot_pos(2, 0);
        motion.c_navyaw = yaw;
        motion.c_depthvalue = -depth;
        motion.c_speedz = robot_pos(2, 1);
#if MOTION_PUB_PRINT_SWITCH
        cout << "e_x: " << motion.cam_e_x << "    " << "e_y:  " << motion.cam_e_y << endl;
        cout << "e_z: " << motion.cam_e_z << "    " << "e_yaw:  " << motion.cam_e_yaw << endl;
        cout << "c_x: " << motion.cam_c_x << "    " << "c_y:  " << motion.cam_c_y << endl;
        cout << "c_z: " << motion.cam_c_z << "    " << "c_yaw: " << motion.cam_c_yaw << endl;
#endif
    };
    break;

    default:
    {
        cout << " please set [robot mode] correctly " << endl;
    };
    break;
    }

    // 公共反馈始终填写；视觉字段只在显式有效时可用于控制。
    const double vision_age = (ros::Time::now() - aruco_stamp).toSec();
    motion.aruco_valid = aruco_valid && !aruco_stamp.isZero() && vision_age >= 0 && vision_age <= 0.5 &&
        (ros::SteadyTime::now() - aruco_received_wall).toSec() <= 0.5;
    motion.aruco_stamp = aruco_stamp.toSec();
    // 普通消息也携带最新视觉值；有效性字段保护首次 dock 的回调先后顺序。
    if (_mode == SINGLE_ROBOT_DEBUG || _mode == TRACKING_ARUCO_MODE)
    {
        motion.cam_e_x = aruco_pos_x; motion.cam_e_y = aruco_pos_y;
        motion.cam_e_z = aruco_pos_z; motion.cam_e_yaw = aruco_yaw;
    }
    const double feedback_age = (ros::Time::now() - feedback_stamp).toSec();
    motion.feedback_valid = feedback_valid && !feedback_stamp.isZero() &&
        feedback_age >= 0 && feedback_age <= 0.5 &&
        (ros::SteadyTime::now() - feedback_received_wall).toSec() <= 0.5;
    motion.feedback_stamp = feedback_stamp.toSec();
    motion.body_vx = body_vx; motion.body_vy = body_vy; motion.body_vz = body_vz;
    motion.body_yaw_deg = body_yaw_deg; motion.body_yaw_rate = body_yaw_rate;
    mc_pub.publish(motion);
}
/*@
 * * * * * * * * * * * * * *  把所有机器人的位置信息发布给每个机器人  * * * * * * * * * * * * * * * * * * *
 */
void multiRobotPosition_pub()
{
    multi_RObotPosition_msg.robot1_pos_x = robotsInfofromQuadrotor(0, 0);
    multi_RObotPosition_msg.robot1_pos_y = robotsInfofromQuadrotor(1, 0);
    multi_RObotPosition_msg.robot2_pos_x = robotsInfofromQuadrotor(0, 1);
    multi_RObotPosition_msg.robot2_pos_y = robotsInfofromQuadrotor(1, 1);
    multi_RObotPosition_msg.robot3_pos_x = robotsInfofromQuadrotor(0, 2);
    multi_RObotPosition_msg.robot3_pos_y = robotsInfofromQuadrotor(1, 2);
    muli_RobotPosition_pub.publish(multi_RObotPosition_msg);
}

/*@
 * * * * * * * * * * * * * *  把基于三次样条得到的轨迹信息发布出去  * * * * * * * * * * * * * * * * * * *
 */

void ref_trajectory_pub()
{

    mul_ref_trajectory_msg.path_length = trajectory_threeSpline.rows();
    vector<float> robot1_path_x, robot1_path_y, robot1_path_yaw, robot1_path_velocity, robot1_path_curvature;
    vector<float> robot2_path_x, robot2_path_y, robot2_path_yaw, robot2_path_velocity, robot2_path_curvature;
    vector<float> robot3_path_x, robot3_path_y, robot3_path_yaw, robot3_path_velocity, robot3_path_curvature;
    vector<float> virtureRobot_path_x, virtureRobot_path_y, virtureRobot_path_yaw, virtureRobot_path_velocity, virtureRobot_path_curvature;

    for (int i = 0; i < trajectory_threeSpline.rows(); i++)
    {

        // 虚拟机器人的参考轨迹
        virtureRobot_path_x.push_back(virture_ref_trajectory(i, 0));
        virtureRobot_path_y.push_back(virture_ref_trajectory(i, 1));
        virtureRobot_path_yaw.push_back(virture_ref_trajectory(i, 2));
        virtureRobot_path_velocity.push_back(virture_ref_trajectory(i, 3));
        virtureRobot_path_curvature.push_back(virture_ref_trajectory(i, 4));

        // 1 号机器人参考轨迹
        robot1_path_x.push_back(robot1_ref_trajectory(i, 0));
        robot1_path_y.push_back(robot1_ref_trajectory(i, 1));
        robot1_path_yaw.push_back(robot1_ref_trajectory(i, 2));
        robot1_path_velocity.push_back(robot1_ref_trajectory(i, 3));
        robot1_path_curvature.push_back(robot1_ref_trajectory(i, 4));

        // 2号机器人参考轨迹
        robot2_path_x.push_back(robot2_ref_trajectory(i, 0));
        robot2_path_y.push_back(robot2_ref_trajectory(i, 1));
        robot2_path_yaw.push_back(robot2_ref_trajectory(i, 2));
        robot2_path_velocity.push_back(robot2_ref_trajectory(i, 3));
        robot2_path_curvature.push_back(robot2_ref_trajectory(i, 4));
        // 3号机器人参考轨迹
        robot3_path_x.push_back(robot3_ref_trajectory(i, 0));
        robot3_path_y.push_back(robot3_ref_trajectory(i, 1));
        robot3_path_yaw.push_back(robot3_ref_trajectory(i, 2));
        robot3_path_velocity.push_back(robot3_ref_trajectory(i, 3));
        robot3_path_curvature.push_back(robot3_ref_trajectory(i, 4));
    }
    mul_ref_trajectory_msg.vs_reference_x = vs_reference_x;
    mul_ref_trajectory_msg.vs_reference_y = vs_reference_y;
    mul_ref_trajectory_msg.vs_reference_yaw = vs_curYaw;

    mul_ref_trajectory_msg.virtureRobot_path_x = virtureRobot_path_x;
    mul_ref_trajectory_msg.virtureRobot_path_y = virtureRobot_path_y;
    mul_ref_trajectory_msg.virtureRobot_path_yaw = virtureRobot_path_yaw;
    mul_ref_trajectory_msg.virtureRobot_path_velocity = virtureRobot_path_velocity;
    mul_ref_trajectory_msg.virtureRobot_path_curvature = virtureRobot_path_curvature;

    mul_ref_trajectory_msg.robot1_path_x = robot1_path_x;
    mul_ref_trajectory_msg.robot1_path_y = robot1_path_y;
    mul_ref_trajectory_msg.robot1_path_yaw = robot1_path_yaw;
    mul_ref_trajectory_msg.robot1_path_velocity = robot1_path_velocity;
    mul_ref_trajectory_msg.robot1_path_curvature = robot1_path_curvature;

    mul_ref_trajectory_msg.robot2_path_x = robot2_path_x;
    mul_ref_trajectory_msg.robot2_path_y = robot2_path_y;
    mul_ref_trajectory_msg.robot2_path_yaw = robot2_path_yaw;
    mul_ref_trajectory_msg.robot2_path_velocity = robot2_path_velocity;
    mul_ref_trajectory_msg.robot2_path_curvature = robot2_path_curvature;

    mul_ref_trajectory_msg.robot3_path_x = robot3_path_x;
    mul_ref_trajectory_msg.robot3_path_y = robot3_path_y;
    mul_ref_trajectory_msg.robot3_path_yaw = robot3_path_yaw;
    mul_ref_trajectory_msg.robot3_path_velocity = robot3_path_velocity;
    mul_ref_trajectory_msg.robot3_path_curvature = robot3_path_curvature;

    trajectory_pub.publish(mul_ref_trajectory_msg);
}

/*@
 * * * * * * * * * * * * * *  把机器人已到达的点数发布出去  * * * * * * * * * * * * * * * * * * *
 */
void achievedWaypoint_pub()
{
    waypoint_msg.self_id = self_id;
    waypoint_msg.robot1_waypoint = point_count;

    // 发布 waypoint_msg
    achievedWaypoint.publish(waypoint_msg);
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 消息订阅函数                                          */
/*                                                                                     */
/***************************************************************************************/
#if SUBSCRIBE_SWITCH

/*@
 * * * * * * * * * * * * * *  接收无人机发过来的路径  * * * * * * * * * * * * * * * * * * *
 */
void path_3dCallback(const asrobot::path_3d::ConstPtr &path_3d_)
{
    ROS_INFO("path_3dCallback!\n");
    imgScale = path_3d_->imgScale;
    vs_reference_x = path_3d_->vs_reference_x;
    vs_reference_y = path_3d_->vs_reference_y;
    vs_curYaw = path_3d_->vs_curYaw;

    // 将接收到path保存到eigen中
    Eigen::MatrixXf temp(3, path_3d_->path_length);
    for (int i = 0; i < path_3d_->path_length; i++)
    {
        temp(0, i) = path_3d_->path_x[i];
        temp(1, i) = path_3d_->path_y[i];
        temp(2, i) = path_3d_->path_z;
    }
    path_3d = temp; // path_3d内部装了路径
    cout << "received path 3d###################" << endl;
}
void path_threeSplineCallback(const asrobot::path_based_threeSpine::ConstPtr &path_threeSpline_)
{

    // Eigen::MatrixXf temp(path_threeSpline_->path_length, 5);
    // for (int i = 0; i < path_threeSpline_->path_length; i++)
    // {
    //     temp(i, 0) = path_threeSpline_->path_x[i];
    //     temp(i, 1) = path_threeSpline_->path_y[i];
    //     temp(i, 2) = path_threeSpline_->path_yaw[i];
    //     temp(i, 3) = path_threeSpline_->path_velocity[i];
    //     temp(i, 4) = path_threeSpline_->path_curvature[i];
    // }
    // trajectory_threeSpline = temp; //一行表示一个参考点的信息
    // update_mul_ref_Info(trajectory_threeSpline, formationShape);
    // cout << "+++++++++++++++++++++++++callback" << trajectory_threeSpline.cols() << "  " << trajectory_threeSpline.rows() << endl;
    // cout << "get the path base three order Spline from the quadrotor" << endl;

    Eigen::MatrixXf temp(3, path_threeSpline_->path_length);
    for (int i = 0; i < path_threeSpline_->path_length; i++)
    {
        temp(0, i) = path_threeSpline_->path_x[i];
        temp(1, i) = path_threeSpline_->path_y[i];
    }
    path_points = temp;
    cout << "################received path point###################" << endl;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*@
 * * * * * * * * * * * * * *  接收无人机发过来的各个机器人的位置  * * * * * * * * * * * * * * * * * * *
 */
// hcy
// void quadrotorcallback(const asrobot::quadrotor2robot::ConstPtr &msg)
// // void quadrotorcallback(const quad2robot::quadrotor2robot::ConstPtr &msg)
// {
//     /*          robot1  robot2  robot3  robot4  robot5
//      * pos_x
//      * pos_y
//      * vel_x, is the velcociy at the x axis of the world frame
//      * vel_y, is the velcociy at the y axis of the world frame
//      */
//     Eigen::MatrixXf robotsInfo_(4, 5);
//     robotsInfo_(0, 0) = msg->robot1_pos_x;
//     robotsInfo_(1, 0) = msg->robot1_pos_y;
//     robotsInfo_(2, 0) = msg->robot1_vel_x;
//     robotsInfo_(3, 0) = msg->robot1_vel_y;
//     robotsInfo_(0, 1) = msg->robot2_pos_x;
//     robotsInfo_(1, 1) = msg->robot2_pos_y;
//     robotsInfo_(2, 1) = msg->robot2_vel_x;
//     robotsInfo_(3, 1) = msg->robot2_vel_y;
//     robotsInfo_(0, 2) = msg->robot3_pos_x;
//     robotsInfo_(1, 2) = msg->robot3_pos_y;
//     robotsInfo_(2, 2) = msg->robot3_vel_x;
//     robotsInfo_(3, 2) = msg->robot3_vel_y;
//     robotsInfo_(0, 3) = msg->robot4_pos_x;
//     robotsInfo_(1, 3) = msg->robot4_pos_y;
//     robotsInfo_(2, 3) = msg->robot4_vel_x;
//     robotsInfo_(3, 3) = msg->robot4_vel_y;
//     robotsInfo_(0, 4) = msg->robot5_pos_x;
//     robotsInfo_(1, 4) = msg->robot5_pos_y;
//     robotsInfo_(2, 4) = msg->robot5_vel_x;
//     robotsInfo_(3, 4) = msg->robot5_vel_y;
//     robotsInfofromQuadrotor = robotsInfo_;
//     // cout<<"+++++++++++++++++ HHHHHHH"<<robotsInfo_(0, 2)<<endl;

//     // cout << " received from robotsInfofromQuadrotor " << endl;

//     // 将无人机检测到的坐标贴到位置模板上
//     all_robots_cur_pos(0, 0) = msg->robot1_pos_x;
//     all_robots_cur_pos(1, 0) = msg->robot1_pos_y;
//     all_robots_cur_pos(2, 0) = 0;
//     all_robots_cur_pos(0, 1) = msg->robot2_pos_x;
//     all_robots_cur_pos(1, 1) = msg->robot2_pos_y;
//     all_robots_cur_pos(2, 1) = 0;
//     all_robots_cur_pos(0, 2) = msg->robot3_pos_x;
//     all_robots_cur_pos(1, 2) = msg->robot3_pos_y;
//     all_robots_cur_pos(2, 2) = 0;

//     // cout << "robot1 current position: " << all_robots_cur_pos(0, 0) << endl;

//     /**********************************************/
//     Eigen::VectorXf quadInfo_temp(12, 1);
//     quadInfo_temp(0) = msg->quad_pos_x;
//     quadInfo_temp(1) = msg->quad_pos_y;
//     quadInfo_temp(2) = msg->quad_pos_z;
//     quadInfo_temp(3) = 0;
//     quadInfo_temp(4) = 0;
//     quadInfo_temp(5) = 0;
//     quadInfo_temp(6) = msg->quad_roll;
//     quadInfo_temp(7) = msg->quad_yaw;
//     quadInfo_temp(8) = msg->quad_pitch;
//     quadInfo_temp(9) = msg->quad_roll_vel;
//     quadInfo_temp(10) = msg->quad_yaw_vel;
//     quadInfo_temp(11) = msg->quad_pitch_vel;
//     quadInfo = quadInfo_temp;

//     //    ROS_INFO("get quadrotor2robot message,   robot1_pos_x=[%.2f], robot1_pos_y=[%.2f], "
//     //             "robot1_vel_x=[%d],robot1_vel_y=[%d],quad_roll=[%.2f], quad_yaw=[%.2f], "
//     //             "quad_pitch=[%.2f]", msg->robot1_pos_x, msg->robot1_pos_y, msg->robot1_vel_x, msg->robot1_vel_y,msg->quad_roll, msg->quad_yaw, msg->quad_pitch);
// }
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*@ljl
 * * * * * * * * * * * * * *  接收 gazebo 发过来的机器人的位置  * * * * * * * * * * * * * * * * * * *
 */

// 回调函数定义
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == robot_name) // 只处理自己这台
        {
            const geometry_msgs::Pose &pose = msg->pose[i];
            const geometry_msgs::Twist &twist = msg->twist[i];

            robot_pos(0, 0) = pose.position.x;
            robot_pos(1, 0) = pose.position.y;
            robot_pos(2, 0) = pose.position.z;

            robot_pos(0, 1) = twist.linear.x;
            robot_pos(1, 1) = twist.linear.y;
            robot_pos(2, 1) = twist.linear.z;
            Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                                 pose.orientation.y, pose.orientation.z);
            feedback_valid = q.coeffs().allFinite() && q.norm() > 1e-9 &&
                std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
                std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
                std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z) &&
                std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z);
            if (feedback_valid)
            {
                const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
                const Eigen::Vector3d vb = R.transpose() * Eigen::Vector3d(
                    twist.linear.x, twist.linear.y, twist.linear.z);
                const Eigen::Vector3d wb = R.transpose() * Eigen::Vector3d(
                    twist.angular.x, twist.angular.y, twist.angular.z);
                body_vx = vb.x(); body_vy = vb.y(); body_vz = vb.z();
                body_yaw_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / std::acos(-1.0);
                body_yaw_rate = wb.z();
                feedback_stamp = ros::Time::now();
                feedback_received_wall = ros::SteadyTime::now();
            }
            break;
        }
    }
}

/*@
 * * * * * * * * * * * * * *  接收无人机发过来的地图  * * * * * * * * * * * * * * * * * * *
 */
void mapCallback(const sensor_msgs::ImageConstPtr &map_)
{
    ROS_INFO("Recived!\n");
    try
    {
        cv::Mat img = cv_bridge::toCvShare(map_, "mono8")->image;
        img.copyTo(revMapImage);
        cv::imshow("MapShow", img);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", map_->encoding.c_str());
    }
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*@
 * * * * * * * * * * * * * *  接收各个机器人到达的路径点数  * * * * * * * * * * * * * * * * * * *
 */
void achievedWaypointCallback(const asrobot::achievedWaypoint::ConstPtr &waypoints)
{
    if (waypoints->self_id == 1)
    {
        achievedWaypointVec(0) = waypoints->robot1_waypoint;
    }
    else if (waypoints->self_id == 2)
    {
        achievedWaypointVec(1) = waypoints->robot2_waypoint;
    }
    else if (waypoints->self_id == 3)
    {
        achievedWaypointVec(2) = waypoints->robot3_waypoint;
    }

    cout << "achieved waypoint: " << achievedWaypointVec.transpose() << endl;
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*@
 * * * * * * * * * * * * * *  接收机器人本身的深度计的信息  * * * * * * * * * * * * * * * * * * *
 */
void depcallback(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    depth = (msg->fluid_pressure - pressureInAir) / pressurePerMeter;
    // 结构化节流日志见下方，避免每条压力消息刷屏。

    // 计算当前深度
    // double current_depth = (msg->fluid_pressure - pressureInAir) / pressurePerMeter;

    // 时间戳
    ros::Time current_time = msg->header.stamp;

    // 静态变量保存上一次数据
    static double last_depth = 0.0;
    static ros::Time last_time;
    static bool initialized = false;

    // 低通滤波参数 (alpha越小越平滑，取0~1)
    const double alpha = 0.3;
    static double filtered_vz = 0.0;

    vz = 0.0; // 垂直速度

    if (initialized)
    {
        double dt = (current_time - last_time).toSec();
        if (dt > 0.0)
        {
            double raw_vz = (depth - last_depth) / dt;                // 原始速度
            filtered_vz = alpha * raw_vz + (1 - alpha) * filtered_vz; // 一阶低通滤波
            vz = filtered_vz;
        }
    }
    else
    {
        initialized = true; // 第一次没有速度
        filtered_vz = 0.0;
    }

    // 更新上一次数据
    last_depth = depth;
    last_time = current_time;

    // 打印深度和滤波后的速度
    ROS_INFO_THROTTLE(1.0, "[DEPTH] depth=%.3f m velocity=%.3f m/s", depth, vz);
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*@
 * * * * * * * * * * * * * *  接收机器人本身的姿态的信息  * * * * * * * * * * * * * * * * * * *
 */
void imucallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double q0, q1, q2, q3;
    double ax, ay, az;

    q0 = msg->orientation.w;
    q1 = msg->orientation.y;
    q2 = msg->orientation.x;
    q3 = msg->orientation.z;
    yaw_velocity = msg->angular_velocity.z;

    // opt_yaw_velocity_value
    yaw_vel_filter_output = yaw_vel_filter.CalFilterOutput(yaw_velocity);

    x_linear_acceleration = msg->linear_acceleration.x;
    y_linear_acceleration = msg->linear_acceleration.y;
    pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                       // pitch
    roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;        // roll
    yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957; // yaw

    // x_linear_acceleration=linear_acceleration1*cos(yaw)+linear_acceleration2*sin(yaw);
    ROS_INFO_THROTTLE(1.0, "[ATTITUDE] pitch=%.2f roll=%.2f yaw=%.2f deg", pitch, roll, yaw);
    //  printf("yaw_velocity:%4f,  x_linear_acceleration:%4f\n", yaw_velocity, x_linear_acceleration);
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// hcy
/*@
 * * * * * * * * * * * * * *  接收双目的信息  * * * * * * * * * * * * * * * * * * *
 */
void binocularCallback(const asrobot::camera_data::ConstPtr &msg)
{
    cam_pos_x = msg->position_x;
    cam_pos_y = msg->position_y;
    cam_pos_z = msg->position_z;

    cam_roll = msg->cam_roll;
    cam_pitch = msg->cam_pitch;
    cam_yaw = msg->cam_yaw;

#if BIONOCULAR_SUB_PRINT_SWITCH
    cout << "#############  binocular camera data ##################" << endl;
    cout << "cam_pos_x: " << cam_pos_x << endl; // hy:机器人x
    cout << "cam_pos_y: " << cam_pos_y << endl; // hy:机器人y
    cout << "cam_pos_z: " << cam_pos_z << endl; // hy:机器人z

    cout << "cam_roll: " << cam_roll << endl;
    cout << "cam_pitch: " << cam_pitch << endl;
    cout << "cam_yaw: " << cam_yaw << endl; // hy:机器人yaw

#endif
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// ljl
/*@
 * * * * * * * * * * * * * *  接收aruco到机体位姿  * * * * * * * * * * * * * * * * * * *
 */
void downCamCallback(const asrobot::camera_data::ConstPtr &msg)
{
    aruco_pos_x = msg->position_x;
    aruco_pos_y = msg->position_y;
    aruco_pos_z = msg->position_z;

    aruco_roll = msg->cam_roll;
    aruco_pitch = msg->cam_pitch;
    aruco_yaw = msg->cam_yaw;
    aruco_valid = msg->aruco_valid && std::isfinite(msg->detection_stamp) && msg->detection_stamp > 0 &&
        std::isfinite(msg->position_x) && std::isfinite(msg->position_y) &&
        std::isfinite(msg->position_z) && std::isfinite(msg->cam_yaw);
    aruco_stamp = aruco_valid ? ros::Time(msg->detection_stamp) : ros::Time(0);
    aruco_received_wall = ros::SteadyTime::now();
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*@
 * * * * * * * * * * * * * *  接收机器人指令  * * * * * * * * * * * * * * * * * * *
 */
void cmdcallback(const std_msgs::String::ConstPtr &msg)
{
    string cmdstring;
    vector<string> cmd;
    // getline(cin,cmdstring);
    cmdstring = msg->data;
    stringstream cmdstream(cmdstring);
    string buf;
    while (cmdstream >> buf)
    {
        cmd.push_back(buf);
    }

    if (cmd[0] == "cxyz") // set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            robotsInfofromQuadrotor(0, self_id - 1) = atof(cmd[1].c_str());
            robotsInfofromQuadrotor(1, self_id - 1) = atof(cmd[2].c_str());
            robotsInfofromQuadrotor(2, self_id - 1) = atof(cmd[3].c_str());
        }
    }

    // if (cmd[0] == "xyz") // set pid parameter,for example:"pid 3 2 1"
    // {
    //     if (cmd.size() == 4)
    //     {
    //         robotsInfofromQuadrotor(0, self_id - 1) = robot_pos(0, 0);
    //         robotsInfofromQuadrotor(1, self_id - 1) = robot_pos(1, 0);
    //         robotsInfofromQuadrotor(2, self_id - 1) = robot_pos(2, 0);
    //     }
    // }

    if (cmd[0] == "pnt")
    {
        if (cmd.size() == 2)
        {
            point_count = atof(cmd[1].c_str());
        }
    }

    if (cmd[0] == "position")
    {
        motion.e_x = atof(cmd[1].c_str()); // x-y;y-x;axis
        motion.e_y = atof(cmd[2].c_str());
        cout << "go to position: " << motion.e_x << "," << motion.e_y << endl
             << endl;
    }

    // 编队变换在无人机部分给出
    if (cmd[0] == "fm")
    { // change formation shape "fm 0 1 2"
        cout << "*********************************************************chagne formation shape......" << endl;

        if (2 == cmd.size())
        {
            // 输入0 为三角型, 1 为横线
            formationShape = atof(cmd[1].c_str());
            cout << " ******** formation swiching !!! " << endl;
        }

        if (4 == cmd.size())
        {
            is_initial_multi_msg = false; // start to build new formation shape
            lineShapeLength = atof(cmd[1].c_str());
            triangleShapeSideLength = atof(cmd[2].c_str());
        }
    }

    if (cmd[0] == "docking") // hcy
    {
        if (cmd.size() == 1)
        {
            docking_flag = true;
        }
    }

    if (cmd[0] == "tracking")
    {
        if (cmd.size() == 1)
        {
            tracking_flag = true;
        }
    }

    if (cmd[0] == "dock") // LJL
    {
        if (cmd.size() == 1)
        {
            tracking_aruco_flag = true;
        }
    }

    if (cmd[0] == "st") // hcy
    {
        docking_flag = false;
        tracking_flag = false;
        tracking_aruco_flag = false;
    }
    cmd.clear();
}
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#endif

/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 定时器触发函数                                         */
/*                                                                                     */
/***************************************************************************************/
#if TIMER_SWITCH

void saveDataTimerCallback(const ros::TimerEvent &event)
{
    // 加入定时保存数据
}

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 decision主程序                                       */
/*                                                                                     */
/***************************************************************************************/

int main(int argc, char **argv)
{

    /*****************************    ROS初始化     ******************************/
    /*                                                                          */
    // std::cout<<"===================="<<std::endl;
    ros::init(argc, argv, "decision");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Rate loop_rate = ros::Rate(iDecisionLoopRate);
    ros::NodeHandle nh("~"); // 用于获取私有参数

    // 从参数服务器获取机器人名字
    nh.param<std::string>("robot_name", robot_name, std::string("gazebo_robot1"));
    /*                                                                          */
    /*****************************    --------     ******************************/

    /*****************************    发布话题       ******************************/
    /*                                                                          */
#if PUBLISH_SWITCH

    mc_pub = n.advertise<asrobot::mc>("mc_message", 10);
    //    multi_pub = n.advertise<asrobot::multirobot>("multi_message", 10);      // for eight camera based formation
    achievedWaypoint = n.advertise<asrobot::achievedWaypoint>("achivWaypoint", 10);
    trajectory_pub = n.advertise<asrobot::mul_ref_trajectory>("mul_ref_trajectory", 10);
    muli_RobotPosition_pub = n.advertise<asrobot::quadrotor2robot>("multi_RobotPosition", 10);
#endif
    /*                                                                          */
    /*****************************    --------     ******************************/

    /*****************************    订阅话题       ******************************/
    /*                                                                          */
#if SUBSCRIBE_SWITCH

    // sub_path_3d = n.subscribe("quad_path_3d", 10, path_3dCallback);
    sub_path_threeSpline = n.subscribe("quad_path_based_threeSpine", 10, path_threeSplineCallback);
    // quadrotor_sub = n.subscribe("quad_robotPos", 10, quadrotorcallback);
    sub_contour = it.subscribe("contour", 50, mapCallback);
    achiv_sub = n.subscribe("achivWaypoint", 10, achievedWaypointCallback);
    // --------------------------------------------------------------------------------

    dep_sub = n.subscribe("pressure", 10, depcallback);
    imu_sub = n.subscribe("imu", 10, imucallback);
    sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);
    biono_camera_sub = n.subscribe("cameradata", 10, binocularCallback);     // hcy
    down_camera_sub = n.subscribe("camera_aruco_data", 10, downCamCallback); // ljl
    model_states_sub = n.subscribe("/gazebo/model_states", 10, modelStatesCallback);

#endif
    /*                                                                          */
    /*****************************    --------     ******************************/

    /*****************************    定时器触发      ******************************/
    /*                                                                          */
#if TIMER_SWITCH

    ros::Timer timer_saveData = n.createTimer(ros::Duration(DURATION), saveDataTimerCallback);

#endif
    /*                                                                          */
    /*****************************     --------     ******************************/

    /*****************************    自定义初始化     ******************************/
    /*                                                                          */
    ROS_ERROR("self_id: %d", self_id); // 红字输出机器人id号
    achievedWaypointVec << 0, 0, 0;    // 初始化机器人接收到的路径点数
    /*                                                                          */
    /*****************************    --------     ******************************/

    /***************************************************************************************/
    /*                                 总循环                                               */
    /***************************************************************************************/

    while (ros::ok())
    {

        /*****************************    自定义函数   ******************************/
        /*                                                                         */

        // initialRobotPosAndPlanPath(FORMATION_MODE);    // 老版本路径规划
        //  initRobotsFormationAndPlanDynamicPath(); //************ 新版本多机路径规划

        /*//hcy
                if(init_flag == false)
                    {
                        for(int i=0; i<10; i++)
                        {   ros::spinOnce();  loop_rate.sleep(); }
                        init_flag = true;
                    }

                UpdateTargetPoint();
                cout << "===================End==================" << endl;
        */

#if DOCKING_SWITCH // hcy
        if (docking_flag == true)
        {
            dockingPlanPathProcess(); // docking 发布期望路径点
            // dockingPlanPathProcess_v2(); // hcy:docking 发布期望路径点
        }
#endif

        /*                                                                          */
        /*****************************    --------     ******************************/
        //  saveSingleRobotAllDatas(self_id, robotsInfofromQuadrotor(0,self_id-1), robotsInfofromQuadrotor(1,self_id-1),-depth, robotsInfofromQuadrotor(2,self_id-1), robotsInfofromQuadrotor(3,self_id-1),x_linear_acceleration,y_linear_acceleration,yaw, yaw_vel_filter_output);

        /*****************************    循环发布消息    ******************************/
        /*                                                                           */
        /// motion_control_pub(SINGLE_PATH_TRACK); // 发布期望路径点 以及机器人实时位置//hcy
        //        multiRobotInfo_pub();                    // 发布自己的位置信息
        // achievedWaypoint_pub();             // 发布自己已经到达了多少点
        // ref_trajectory_pub();               //*********发布参考路径
        // multiRobotPosition_pub();           //*********发布所有机器人的位置信息

#if DOCKING_SWITCH // hcy
        if (docking_flag == true)
        {
            motion_control_pub(DOCKING_MODE); // 发布期望路径点 以及机器人实时位置
        }
#endif

#if TRACKING_SWITCH // hcy
        if (tracking_flag == true)
        {
            motion_control_pub(TRACKING_MODE); // 发布期望路径点 以及机器人实时位置
        }
#endif

#if TRACKING_ARUCO_SWITCH // ljl
        if (tracking_aruco_flag == true)
        {
            motion_control_pub(TRACKING_ARUCO_MODE); // 发布期望路径点 以及机器人实时位置
        }
#endif

#if SINGLE_ROBOT_DEBUG_SWITCH                   // hcy
        if (!tracking_aruco_flag)
            motion_control_pub(SINGLE_ROBOT_DEBUG); // 对接时每轮只发布一次，避免重复积分
#endif

        /*                                                                          */
        /*****************************    --------     ******************************/
        // cout << "------------------------"<< init_flag << "---------------------------"<< endl;//hcy
        ros::spinOnce();
        loop_rate.sleep();
    }
    /*-------------------------------------------------------------------------------------*/

    // save_data.fout.close();
    return 0;
}
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/
