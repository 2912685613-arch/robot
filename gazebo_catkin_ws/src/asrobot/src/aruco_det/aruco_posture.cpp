
#include "../../include/asrobot/f_multi_robot_plan.h"
#include "../../include/asrobot/f_single_robot_control.h"
#include <geometry_msgs/PoseStamped.h>
#include "asrobot/camera_data.h"

using namespace Eigen;
using namespace std;

// Twb的意思是body系在world系下的位姿，也是body系到world系的变换矩阵，Twb = Tb->w
Eigen::Isometry3d Twa, Tca, Tba, Tbc, Twb, Twc; // 变换矩阵4*4

Eigen::Matrix3d rotation_matrix_bc, rotation_matrix_wb, rotation_matrix_ca; // 旋转矩阵

Eigen::Vector3d t_bc, t_wb, t_ca; // 平移向量

Quaterniond q_wb, q_ca; // 四元数

asrobot::camera_data msg; // define a camera message

// 都使用gazebo发布的model的位姿
void modelStatesCallback_(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
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

void aruco_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)//Tca从aruco到camera
{
    t_ca << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    q_ca = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    rotation_matrix_ca = q_ca.toRotationMatrix();
  
    //  得到aruco到相机系
    Tca.linear() = rotation_matrix_ca;
    Tca.translation() = t_ca;
}






int main(int argc, char **argv)
{

    double position_x, position_y, position_z;
    double cam_pitch, cam_roll, cam_yaw;

    ros::init(argc, argv, "aruco_posture");
    ros::NodeHandle nh;

    ros::Subscriber position_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback_);
    ros::Subscriber aruco_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, aruco_pos_cb);

    ros::Publisher tba_pub = nh.advertise<asrobot::camera_data>("camera_aruco_data", 1000);

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
        cout << "aruco码-位姿" << endl;

        Tba = Tbc * Tca;
        Twa =Twb * Tbc * Tca;

        // 获取T中的平移向量
        Vector3d t_wa = Tba.translation();//按照世界坐标系的绝对坐标应该是Twa，采用相对用Tba
        position_x = t_wa[0];
        position_y = t_wa[1];
        position_z = t_wa[2];
        cout << "position_x:" << t_wa[0] << endl
             << "position_y:" << t_wa[1] << endl
             << "position_z:" << t_wa[2] << endl;
        // 获取R
        Matrix3d R = Tba.rotation(); // 按照世界坐标系的绝对坐标应该是Twa，采用相对用Tba
        Quaterniond q(R);//转为四元数
        double q0, q1, q2, q3;
        q0 = q.w();
        q1 = q.x();
        q2 = q.y();
        q3 = q.z();
        // 转为欧拉角
        cam_pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                               // pitch
        cam_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;    // roll
        cam_yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957; // yaw
       
       
        cout << "pitch:" << cam_pitch << endl
             << "roll:" << cam_roll << endl
             << "yaw:" << cam_yaw << endl;


        // 统一发布 将变换后的数据发布出去
        msg.cam_pitch = cam_pitch;
        msg.cam_roll = cam_roll;
        // msg.cam_yaw = -cam_yaw;
        msg.cam_yaw = cam_yaw; 

        msg.position_x = position_x;
        msg.position_y = position_y;
        // msg.position_z = position_y;
        msg.position_z = position_z + 0.06; //相机和机体之间差0.03 再加0.01防止碰撞
        tba_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}