/**
 * @file aruco_posture.cpp
 * @brief 【主流程／重要】将标记在相机光学坐标系下的位姿转换到机器人机体系。
 * 输入：/aruco/pose、/gazebo/model_states；输出：camera_aruco_data。
 * 核心：Tba = Tbc * Tca。当前发布相对机体的 Tba，Twa 仅供世界系扩展使用。
 */
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include "asrobot/camera_data.h"

// 【重要：命名】Txy 将 y 系坐标变到 x 系；w=世界、b=机体、c=光学相机、a=标记。
// 显式初始化，首帧之前禁止发布，避免将未初始化的数据交给决策模块。
Eigen::Isometry3d Twb = Eigen::Isometry3d::Identity();
Eigen::Isometry3d Tca = Eigen::Isometry3d::Identity();
std::string model_name, camera_frame_id;
ros::Time pose_stamp;
bool have_pose = false, have_world_pose = false;

/**
 * @brief 【扩展用途】读取指定 Gazebo 模型的世界位姿，构建机体到世界的 Twb。
 * @param msg Gazebo 模型名与位姿；未找到指定模型时保留上一结果。
 * @return 无；当前相对机体的输出不依赖此回调，可在学习主流程时后读。
 */
void modelStatesCallback_(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i)
    {
        if (msg->name[i] != model_name) continue;
        const auto &pose = msg->pose[i];
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-9 || !t.allFinite()) return;
        Twb.linear() = q.normalized().toRotationMatrix();
        Twb.translation() = t;
        have_world_pose = true;
        return;
    }
}

/**
 * @brief 【重要】验证检测结果的参考系和数值，缓存 Tca 及图像采集时间。
 * @param msg 标记原点在相机光学系的位置（米）和标记系到光学系的四元数。
 * @return 无；参考系不匹配、数值无效时拒绝接收，main 根据时间戳停止过期输出。
 */
void aruco_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (msg->header.frame_id != camera_frame_id)
    {
        ROS_WARN_THROTTLE(2.0, "Ignoring ArUco pose in frame '%s'; expected '%s'.",
                          msg->header.frame_id.c_str(), camera_frame_id.c_str());
        return;
    }
    const auto &p = msg->pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-9 || !t.allFinite()) return;
    Tca.linear() = q.normalized().toRotationMatrix();
    Tca.translation() = t;
    pose_stamp = msg->header.stamp;
    have_pose = true;
}

/**
 * @brief 【核心入口】配置安装变换，以 20 Hz 发布仍在有效时间内的标记相对机体位姿。
 * @param argc/argv ROS 启动参数。camera_x/y/z 是相机光心在机体系的位置，单位米。
 * @return 参数无效时返回 1，正常退出返回 0。
 * 位置输出以米为单位，姿态角以度为单位；position_z_offset 是可选的独立补偿。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_posture");
    ros::NodeHandle nh, private_nh("~");
    double camera_x, camera_y, camera_z, z_offset, pose_timeout;
    private_nh.param<std::string>("model_name", model_name, "gazebo_robot1");
    private_nh.param<std::string>("camera_frame_id", camera_frame_id,
                                 "gazebo_robot1/camera_bottom_optical_frame");
    private_nh.param("camera_x", camera_x, 0.0);
    private_nh.param("camera_y", camera_y, 0.0);
    private_nh.param("camera_z", camera_z, -0.12);
    private_nh.param("position_z_offset", z_offset, 0.0);
    private_nh.param("pose_timeout", pose_timeout, 0.5);
    if (!std::isfinite(camera_x) || !std::isfinite(camera_y) || !std::isfinite(camera_z) ||
        !std::isfinite(z_offset) || !std::isfinite(pose_timeout) || pose_timeout <= 0)
    {
        ROS_FATAL("Invalid camera translation, position offset or pose timeout.");
        return 1;
    }

    // 【重要：安装关系】与 bottom_camera 宏中的两个固定关节一致。
    // 相机光学系 (x,y,z) 到机体系的旋转结果为 (-y,-x,-z)，再加光心安装偏移。
    Eigen::Isometry3d Tbc = Eigen::Isometry3d::Identity();
    Tbc.linear() << 0, -1, 0, -1, 0, 0, 0, 0, -1;
    Tbc.translation() = Eigen::Vector3d(camera_x, camera_y, camera_z);

    auto position_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback_);
    auto aruco_pose_sub = nh.subscribe("/aruco/pose", 1, aruco_pos_cb);
    auto data_pub = nh.advertise<asrobot::camera_data>("camera_aruco_data", 1);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        const double age = (ros::Time::now() - pose_stamp).toSec();
        const bool pose_fresh = have_pose && age >= 0 && age <= pose_timeout;
        if (!pose_fresh)
        {
            ROS_WARN_THROTTLE(2.0, "Waiting for a fresh ArUco pose (timeout %.2f s).", pose_timeout);
            asrobot::camera_data invalid;
            invalid.aruco_valid = false;
            invalid.detection_stamp = pose_stamp.toSec();
            data_pub.publish(invalid);
            loop_rate.sleep();
            continue;
        }

        // 【核心】当前输出 Tba，表示标记相对机体的位置和姿态。
        const Eigen::Isometry3d Tba = Tbc * Tca;
        if (have_world_pose)
        {
            // 【次要：扩展】这是结合 Gazebo 真值计算的标记世界位姿，不是机器人视觉定位结果。
            const Eigen::Isometry3d Twa = Twb * Tba;
            ROS_DEBUG_THROTTLE(2.0, "Marker world position: %.3f %.3f %.3f",
                              Twa.translation().x(), Twa.translation().y(), Twa.translation().z());
        }
        Eigen::Quaterniond q(Tba.rotation());
        q.normalize();
        const double to_degrees = 180.0 / std::acos(-1.0);
        const double sin_pitch = 2 * (q.w() * q.y() - q.x() * q.z());
        asrobot::camera_data data;
        data.cam_pitch = std::asin(std::max(-1.0, std::min(1.0, sin_pitch))) * to_degrees;
        data.cam_roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                                  1 - 2 * (q.x() * q.x() + q.y() * q.y())) * to_degrees;
        data.cam_yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                                 1 - 2 * (q.y() * q.y() + q.z() * q.z())) * to_degrees;
        data.position_x = Tba.translation().x();
        data.position_y = Tba.translation().y();
        data.position_z = Tba.translation().z() + z_offset;
        data.aruco_valid = true;
        data.detection_stamp = pose_stamp.toSec();
        data_pub.publish(data);
        loop_rate.sleep();
    }
    return 0;
}
