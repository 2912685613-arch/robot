/**
 * @file aruco_posture.cpp
 * @brief 【主流程／重要】将 ArUco 相机系位姿转为机体系位姿，发布给 decision。
 * 输入：/aruco/pose、/gazebo/model_states；输出：camera_aruco_data（asrobot::camera_data）。
 * 核心：Tba = Tbc * Tca；当前发布 Tba，世界系 Twa 仅计算，不用于输出。
 * 阅读优先级：aruco_pos_cb、Tbc 安装变换、Tba 和消息赋值最重要；cout 为辅助。
 */

#include "../../include/asrobot/f_multi_robot_plan.h"
#include "../../include/asrobot/f_single_robot_control.h"
#include <geometry_msgs/PoseStamped.h>
#include "asrobot/camera_data.h"

using namespace Eigen;
using namespace std;

// Twb的意思是body系在world系下的位姿，也是body系到world系的变换矩阵，Twb = Tb->w
// 【重要：命名】Txy 表示从 y 系到 x 系；w=世界，b=机体，c=相机，a=标记。
// 注意：这些变换没有显式初始化为 Identity，且没有等待首帧/超时判断；启动和丢失时需谨慎解释输出。
Eigen::Isometry3d Twa, Tca, Tba, Tbc, Twb, Twc; // 变换矩阵4*4

Eigen::Matrix3d rotation_matrix_bc, rotation_matrix_wb, rotation_matrix_ca; // 旋转矩阵

Eigen::Vector3d t_bc, t_wb, t_ca; // 平移向量

Quaterniond q_wb, q_ca; // 四元数

asrobot::camera_data msg; // define a camera message

// 都使用gazebo发布的model的位姿
/**
 * @brief 【扩展用途】遍历 Gazebo 模型，读取 gazebo_robot1 的世界位姿并更新 Twb。
 * @param msg 仿真中的模型名称及对应位姿；找不到指定模型时不更新。
 * @return 无；写入 t_wb、q_wb、rotation_matrix_wb 和 Twb。
 * 优先级：当前只用 Twb 计算未发布的 Twa；理解相对机体的输出可暂时跳过。
 */
void modelStatesCallback_(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    // 【次要：未使用变量】此局部标志当前没有参与任何判断。
    bool model_found_in_msg = false;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "gazebo_robot1")
        {
            const geometry_msgs::Pose &pose = msg->pose[i];
    
            t_wb << pose.position.x, pose.position.y, pose.position.z;
            q_wb.w() = pose.orientation.w;
            q_wb.x() = pose.orientation.x;
            q_wb.y() = pose.orientation.y;
            q_wb.z() = pose.orientation.z;
            rotation_matrix_wb = q_wb.toRotationMatrix(); // 四元数转为旋转矩阵

            //  得到机体系到世界系的变换矩阵R
            Twb.linear() = rotation_matrix_wb;
            //  得到机体系到世界系的变换矩阵T
            Twb.translation() = t_wb;
            break;
        }
    }
}

/**
 * @brief 【重要】接收标记相对相机的位姿，构建标记系到相机系的刚体变换 Tca。
 * @param msg /aruco/pose：平移单位米，姿态为四元数。
 * @return 无；保存最近一次 Tca，主循环据此转换坐标。
 * 注意：当前不校验 frame_id、时间戳和四元数，也不记录目标是否已经丢失。
 */
void aruco_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)//Tca从aruco到camera
{
    t_ca << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    q_ca = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    rotation_matrix_ca = q_ca.toRotationMatrix();
  
    //  得到aruco到相机系
    Tca.linear() = rotation_matrix_ca;
    Tca.translation() = t_ca;
}






/**
 * @brief 【核心入口】设置安装变换 Tbc，以 20 Hz 转换标记位姿并发布自定义消息。
 * @param argc/argv ROS 启动参数。
 * @return 正常退出返回 0。
 * 关键输出：机体系位置（米）和 roll/pitch/yaw（度）；position_z 另加 0.06 m。
 */
int main(int argc, char **argv)
{

    double position_x, position_y, position_z;
    double cam_pitch, cam_roll, cam_yaw;

    ros::init(argc, argv, "aruco_posture");
    ros::NodeHandle nh;

    ros::Subscriber position_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback_);
    ros::Subscriber aruco_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, aruco_pos_cb);

    ros::Publisher tba_pub = nh.advertise<asrobot::camera_data>("camera_aruco_data", 1000);

    // 【重要：安装关系】当前平移设为零，旋转使 (x_b,y_b,z_b)=(-y_c,-x_c,-z_c)。
    // 此矩阵依赖相机安装方向；更换安装方向时应重新核对。
    // Tbc相机系到机体系的变换
    t_bc << 0, 0, 0;
    rotation_matrix_bc << 0, -1, 0,
                         -1, 0, 0,
                         0, 0, -1;

    Tbc.linear() = rotation_matrix_bc;
    Tbc.translation() = t_bc;
    t_ca << 0, 0, 0;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate loop_rate = ros::Rate(20.0);

    while (ros::ok())
    {
        // 【次要：调试】cout 只输出观察信息，不参与下游计算。
        cout << "aruco码-位姿" << endl;

        // 【核心】先将标记从相机系变到机体系；Twa 额外结合仿真机体世界位姿。
        Tba = Tbc * Tca;
        Twa =Twb * Tbc * Tca;

        // 获取T中的平移向量
        // 【重要】变量名虽叫 t_wa，实际取自 Tba，表示相对机体的位置。
        Vector3d t_wa = Tba.translation();//按照世界坐标系的绝对坐标应该是Twa，采用相对用Tba
        position_x = t_wa[0];
        position_y = t_wa[1];
        position_z = t_wa[2];
        cout << "position_x:" << t_wa[0] << endl
             << "position_y:" << t_wa[1] << endl
             << "position_z:" << t_wa[2] << endl;
        // 获取R
        // 【重要】从同一个 Tba 提取旋转，保证姿态与发布位置使用同一参考系。
        Matrix3d R = Tba.rotation(); // 按照世界坐标系的绝对坐标应该是Twa，采用相对用Tba
        Quaterniond q(R);//转为四元数
        double q0, q1, q2, q3;
        q0 = q.w();
        q1 = q.x();
        q2 = q.y();
        q3 = q.z();
        // 转为欧拉角
        // 【重要：单位】反三角函数返回弧度，乘约 57.3 后转为度，下游需按度解释。
        cam_pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                               // pitch
        cam_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;    // roll
        cam_yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957; // yaw
       
       
        cout << "pitch:" << cam_pitch << endl
             << "roll:" << cam_roll << endl
             << "yaw:" << cam_yaw << endl;


        // 【核心输出】decision 的 downCamCallback 接收以下六个位置/姿态字段。
        // 统一发布 将变换后的数据发布出去
        msg.cam_pitch = cam_pitch;
        msg.cam_roll = cam_roll;
        // msg.cam_yaw = -cam_yaw;
        msg.cam_yaw = cam_yaw; 

        msg.position_x = position_x;
        msg.position_y = position_y;
        // msg.position_z = position_y;
        // 【重要：固定补偿】实际加的是 0.06 m；下面旧注释中的 0.03+0.01 与数值不一致。
        msg.position_z = position_z + 0.06; //相机和机体之间差0.03 再加0.01防止碰撞
        tba_pub.publish(msg);

        // 【重要：调度】本轮发布后再接收回调；未收到新位姿时仍会继续发布缓存值。
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}