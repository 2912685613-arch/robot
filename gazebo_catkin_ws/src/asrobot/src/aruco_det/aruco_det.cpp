/**
 * @file aruco_det.cpp
 * @brief 【主流程／重要】大小双标记检测节点，当前由 CMakeLists.txt 编译、aruco_det.launch 启动。
 * 数据流：底部相机 Image -> 识别 ID/角点 -> 估计标记相对相机的位姿 -> /aruco/pose。
 * 必须理解：相机标定、标记实际边长、目标切换、位姿计算和发布。
 * 次要辅助：日志、画框/画坐标轴、imshow 窗口、aruco_det_image 可视化输出。
 * “次要”仅指阅读优先级；图像话题可能被其他工具订阅，不代表可以直接删除。
 */
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/CameraInfo.h>
#include <cmath>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/aruco.hpp>
#include <geometry_msgs/PoseStamped.h>

// For tf2 conversions
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Includes the above and conversion functions

#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <algorithm>

// 【重要：共享数据】图像回调保存最新一帧，主循环读取；spinOnce 在本线程处理回调。
cv::Mat frame; // Global frame variable
cv::Mat cameraMatrix, distCoeffs;
std_msgs::Header image_header;
bool new_frame = false;

/**
 * @brief 【重要】读取相机发布的 K 和 D，保持内参与实际图像匹配。
 * @param msg CameraInfo；仅接受本节点支持的针孔相机和 OpenCV 畸变模型。
 * @return 无；有效时更新 cameraMatrix/distCoeffs，无效时继续等待。
 */
void camera_infoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if (msg->K[0] <= 0 || msg->K[4] <= 0 ||
        (!msg->distortion_model.empty() && msg->distortion_model != "plumb_bob" &&
         msg->distortion_model != "rational_polynomial"))
    {
        ROS_WARN_THROTTLE(2.0, "Waiting for valid pinhole camera calibration.");
        return;
    }
    for (double value : msg->K)
        if (!std::isfinite(value)) return;
    const auto count = msg->D.size();
    if (count != 0 && count != 4 && count != 5 && count != 8 && count != 12 && count != 14)
        return;
    for (double value : msg->D)
        if (!std::isfinite(value)) return;
    cameraMatrix = cv::Mat(3, 3, CV_64F);
    std::copy(msg->K.begin(), msg->K.end(), cameraMatrix.ptr<double>());
    distCoeffs = count == 0 ? cv::Mat::zeros(5, 1, CV_64F) : cv::Mat(msg->D, true);
}

/**
 * @brief 【重要】接收相机图像，转成 OpenCV BGR8 格式，复制到全局 frame。
 * @param msg ROS 相机图像；转换失败时记录错误，保留此前的 frame。
 * @return 无；仅更新图像缓存，不在回调里执行检测，也不发布位姿。
 * 主循环仅在 new_frame 为真时处理图像，没有新图像时不重复发布。
 */
void cam_imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image.clone(); // 保存图像及其采集时间，避免重复检测旧帧。
        image_header = msg->header;
        new_frame = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/**
 * @brief 【核心入口】配置 ROS、加载标定，然后以 10 Hz 执行检测、切换目标和发布。
 * @param argc/argv ROS 命令行参数，可用于覆盖私有参数和重映射话题。
 * @return 参数或离线相机配置无效时返回 1，正常退出返回 0。
 * 阅读顺序：参数 -> 相机矩阵 -> 大小码状态机 -> 目标位姿 -> ROS 发布。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_det_node");
    ros::NodeHandle n;
    ros::NodeHandle nh1("~"); // Private NodeHandle for parameters

    // 【重要】私有参数属于当前节点；字典须匹配图案，边长须匹配实物，否则识别或距离会错。
    // --- Parameters for ArUco detection ---
    int dictionary_id;
    std::string camera_param_path;
    std::string sub_image_topic;
    std::string camera_frame_id;
    std::string camera_info_topic;
    bool use_camera_info, show_debug_image;

    // Parameters for the outer (larger) ArUco marker
    int aruco_id_outer;
    float aruco_length_outer;

    // Parameters for the inner (smaller) ArUco marker
    int aruco_id_inner;
    float aruco_length_inner;

    // Parameters for Z distance switching thresholds
    float switch_depth_threshold_to_inner;  // Renamed from switch_depth_threshold
    float switch_depth_threshold_to_outer;  // New parameter
    int inner_marker_lost_threshold_frames; // New parameter

    nh1.param<int>("dictionary_id", dictionary_id, 10); // e.g., DICT_6X6_250

    nh1.param<int>("aruco_id_outer", aruco_id_outer, 5);
    nh1.param<float>("aruco_length_outer", aruco_length_outer, 0.1f);

    nh1.param<int>("aruco_id_inner", aruco_id_inner, 18);
    nh1.param<float>("aruco_length_inner", aruco_length_inner, 0.02f);

    nh1.param<float>("switch_depth_threshold_to_inner", switch_depth_threshold_to_inner, 0.5f);
    nh1.param<float>("switch_depth_threshold_to_outer", switch_depth_threshold_to_outer, 0.7f);  // 大于内切阈值，形成回差
    nh1.param<int>("inner_marker_lost_threshold_frames", inner_marker_lost_threshold_frames, 100); // e.g., 10 frames

    // 【重要】默认订阅 CameraInfo；离线标定模式由 launch 提供与机器无关的 YAML 路径。
    nh1.param<std::string>("camera_param_path", camera_param_path, "");
    nh1.param<std::string>("sub_image_topic", sub_image_topic, "/gazebo_robot1/camera_bottom/image_raw");
    nh1.param<std::string>("camera_frame_id", camera_frame_id, "gazebo_robot1/camera_bottom_optical_frame");
    nh1.param<std::string>("camera_info_topic", camera_info_topic, "/gazebo_robot1/camera_bottom/camera_info");
    nh1.param<bool>("use_camera_info", use_camera_info, true);
    nh1.param<bool>("show_debug_image", show_debug_image, false);
    if (aruco_id_outer == aruco_id_inner || aruco_length_outer <= 0 || aruco_length_inner <= 0 ||
        switch_depth_threshold_to_inner <= 0 ||
        switch_depth_threshold_to_outer <= switch_depth_threshold_to_inner ||
        inner_marker_lost_threshold_frames <= 0)
    {
        ROS_FATAL("Invalid marker parameters: IDs must differ, lengths/counts must be positive, outer threshold must exceed inner threshold.");
        return 1;
    }

    // 【次要：诊断】启动时打印配置，便于核对；不参与位姿计算。
    ROS_INFO("--- ArUco Detection Parameters ---");
    ROS_INFO("Dictionary ID: %d", dictionary_id);
    ROS_INFO("Outer ArUco ID: %d, Length: %.3f m", aruco_id_outer, aruco_length_outer);
    ROS_INFO("Inner ArUco ID: %d, Length: %.3f m", aruco_id_inner, aruco_length_inner);
    ROS_INFO("Switch to Inner Threshold (Z_outer < %.3f m)", switch_depth_threshold_to_inner);
    ROS_INFO("Switch to Outer Threshold (Z_inner > %.3f m OR lost for %d frames)", switch_depth_threshold_to_outer, inner_marker_lost_threshold_frames);
    ROS_INFO("Camera Parameters Path: %s", camera_param_path.c_str());
    ROS_INFO("Subscribed Image Topic: %s", sub_image_topic.c_str());
    ROS_INFO("Camera Frame ID for Pose: %s", camera_frame_id.c_str());
    ROS_INFO("---------------------------------");

    // 【接口】图像输出供观察；/aruco/pose 是下游 aruco_posture 使用的核心输出。
    ros::Publisher aruco_det_image_pub = n.advertise<sensor_msgs::Image>("aruco_det_image", 10);
    ros::Subscriber cam_sub = n.subscribe(sub_image_topic, 1, cam_imageCallback);
    ros::Publisher aruco_det_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/aruco/pose", 10);

    ros::Subscriber camera_info_sub;
    if (use_camera_info)
        camera_info_sub = n.subscribe(camera_info_topic, 1, camera_infoCallback);

    cv_bridge::CvImage cv_image;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // 【重要：标定】优先使用 CameraInfo；离线模式保留 YAML，解析异常也在此捕获。
    if (!use_camera_info)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(camera_param_path);
            double fx = config["fx"].as<double>();
            double fy = config["fy"].as<double>();
            double cx = config["x0"].as<double>();
            double cy = config["y0"].as<double>();
            cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            distCoeffs = (cv::Mat_<double>(5, 1) << config["k1"].as<double>(),
                config["k2"].as<double>(), config["p1"].as<double>(),
                config["p2"].as<double>(), config["k3"].as<double>());
            if (fx <= 0 || fy <= 0 || !cv::checkRange(cameraMatrix) || !cv::checkRange(distCoeffs))
            {
                ROS_FATAL("Invalid camera calibration in YAML.");
                return 1;
            }
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Failed to load camera parameters from %s: %s", camera_param_path.c_str(), e.what());
            return 1;
        }
    }

    // 【重要：状态】current_target_id/length 必须配套切换，边长参与米制距离估算。
    // --- State variables for target switching ---
    int current_target_id = aruco_id_outer; // Initial target
    float current_target_length = aruco_length_outer;
    int inner_marker_lost_count = 0; // Counter for how many consecutive frames inner marker is lost

    ROS_INFO("Initially targeting ArUco ID %d (Outer)", current_target_id);

    ros::Rate loop_rate(10); // Loop rate e.g., 10 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        if (cameraMatrix.empty())
            ROS_WARN_THROTTLE(2.0, "Waiting for camera calibration on %s", camera_info_topic.c_str());
        if (new_frame && !frame.empty() && !cameraMatrix.empty())
        {
            new_frame = false;
            cv::Mat current_frame = frame.clone(); // Work on a copy of the frame
            cv::Mat gray;
            cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            // 【核心步骤 1】markerIds 与 markerCorners 按索引一一对应；此时还没有米制位姿。
            cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

            if (markerIds.size() > 0)
            {
                // 【次要：可视化】只往显示图像上画框，后续位姿使用之前提取的角点。
                cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);

                // 【核心步骤 2】分别按大小码的实际边长估计 Z，用来决定当前跟踪哪一个。
                // --- Information gathering for both markers (if present)-获取当前码 ---
                bool outer_marker_visible_this_frame = false;
                double z_distance_to_outer = -1.0; // Invalid initial value
                auto it_outer = std::find(markerIds.begin(), markerIds.end(), aruco_id_outer);
                if (it_outer != markerIds.end())
                {
                    outer_marker_visible_this_frame = true;
                    int index_outer = std::distance(markerIds.begin(), it_outer);
                    std::vector<cv::Vec3d> rvec_outer_check, tvec_outer_check;
                    std::vector<std::vector<cv::Point2f>> outer_marker_corners_subset = {markerCorners[index_outer]};
                    cv::aruco::estimatePoseSingleMarkers(outer_marker_corners_subset, aruco_length_outer, cameraMatrix, distCoeffs, rvec_outer_check, tvec_outer_check);
                    if (!tvec_outer_check.empty())
                    {
                        z_distance_to_outer = tvec_outer_check[0][2];
                    }
                }

                bool inner_marker_visible_this_frame = false;
                double z_distance_to_inner = -1.0; // Invalid initial value
                auto it_inner = std::find(markerIds.begin(), markerIds.end(), aruco_id_inner);
                if (it_inner != markerIds.end())
                {
                    inner_marker_visible_this_frame = true;
                    int index_inner = std::distance(markerIds.begin(), it_inner);
                    std::vector<cv::Vec3d> rvec_inner_check, tvec_inner_check;
                    std::vector<std::vector<cv::Point2f>> inner_marker_corners_subset = {markerCorners[index_inner]};
                    cv::aruco::estimatePoseSingleMarkers(inner_marker_corners_subset, aruco_length_inner, cameraMatrix, distCoeffs, rvec_inner_check, tvec_inner_check);
                    if (!tvec_inner_check.empty())
                    {
                        z_distance_to_inner = tvec_inner_check[0][2];
                    }
                }

                // 【核心步骤 3】默认大码近于 0.5 m 且小码可见时转小码；小码远于 0.7 m 或丢失过久时尝试转大码。
                // 注意：这里比较相机 Z 分量，并非三维欧氏距离。
                // 外切阈值大于内切阈值，形成回差；仅处理回调更新的帧，避免旧图重复计入丢失。
                // --- Dual Switching Logic-切换逻辑 ---
                if (current_target_id == aruco_id_outer)
                {
                    // Try to switch to inner
                    if (outer_marker_visible_this_frame && z_distance_to_outer > 0 && abs(z_distance_to_outer) < switch_depth_threshold_to_inner)
                    {
                        // Prefer inner if outer is close enough AND inner is also visible
                        if (inner_marker_visible_this_frame)
                        {
                            current_target_id = aruco_id_inner;
                            current_target_length = aruco_length_inner;
                            inner_marker_lost_count = 0; // Reset counter
                            ROS_INFO("Switching to Inner ArUco ID %d (Z_outer = %.3f m < %.3f m, Inner Visible)",
                                     current_target_id, z_distance_to_outer, switch_depth_threshold_to_inner);
                        }
                        else
                        {
                            ROS_WARN_THROTTLE(1.0, "Outer marker close (Z=%.3f m), but inner marker ID %d not visible. Sticking to outer.", z_distance_to_outer, aruco_id_inner);
                        }
                    }
                }
                else // current_target_id == aruco_id_inner
                {
                    bool should_switch_to_outer = false;
                    if (!inner_marker_visible_this_frame)
                    {
                        inner_marker_lost_count++;
                        ROS_WARN_THROTTLE(1.0, "Inner ArUco ID %d lost for %d frame(s).", aruco_id_inner, inner_marker_lost_count);
                        if (inner_marker_lost_count >= inner_marker_lost_threshold_frames)
                        {
                            should_switch_to_outer = true;
                            ROS_WARN("Inner ArUco ID %d lost for %d frames. Attempting to switch to outer.", aruco_id_inner, inner_marker_lost_count);
                        }
                    }
                    else // Inner marker is visible
                    {
                        inner_marker_lost_count = 0; // Reset counter
                        if (z_distance_to_inner > 0 && abs(z_distance_to_inner) > switch_depth_threshold_to_outer)
                        {
                            should_switch_to_outer = true;
                            ROS_INFO("Inner ArUco ID %d is too far (Z_inner = %.3f m > %.3f m). Attempting to switch to outer.",
                                     aruco_id_inner, z_distance_to_inner, switch_depth_threshold_to_outer);
                        }
                    }

                    if (should_switch_to_outer)
                    {
                        if (outer_marker_visible_this_frame)
                        {
                            current_target_id = aruco_id_outer;
                            current_target_length = aruco_length_outer;
                            ROS_INFO("Switched back to Outer ArUco ID %d.", current_target_id);
                        }
                        else
                        {
                            ROS_WARN_THROTTLE(1.0, "Wanted to switch to outer marker, but it's not visible. Sticking to inner (even if lost).");
                            // If inner is lost and outer is also lost, we are in a difficult state.
                            // For now, we keep current_target_id as inner, hoping it reappears.
                        }
                    }
                }

                // 【核心步骤 4】只发布当前选中 ID 的位姿；看到其他码并不等于当前目标有效。
                // --- Process the current_target_id ---
                auto it_target = std::find(markerIds.begin(), markerIds.end(), current_target_id);
                if (it_target != markerIds.end())
                {
                    int index_target = std::distance(markerIds.begin(), it_target);
                    std::vector<cv::Vec3d> rvec_target_vec, tvec_target_vec;
                    std::vector<std::vector<cv::Point2f>> target_marker_corners_subset = {markerCorners[index_target]};
                    cv::aruco::estimatePoseSingleMarkers(target_marker_corners_subset, current_target_length, cameraMatrix, distCoeffs, rvec_target_vec, tvec_target_vec);

                    if (!rvec_target_vec.empty() && !tvec_target_vec.empty())
                    {
                        cv::Vec3d rvec_target = rvec_target_vec[0];
                        cv::Vec3d tvec_target = tvec_target_vec[0];

                        // 【次要：可视化】画坐标轴，不改变估算得到的 rvec/tvec。
                        cv::aruco::drawAxis(current_frame, cameraMatrix, distCoeffs, rvec_target, tvec_target, current_target_length * 0.5f);

                        // 【重要：输出约定】位置为标记原点在相机系中的坐标（米），姿态为标记系到相机系的旋转。
                        geometry_msgs::PoseStamped aruco_det_pose;
                        aruco_det_pose.header.stamp = image_header.stamp;
                        aruco_det_pose.header.frame_id = image_header.frame_id.empty() ? camera_frame_id : image_header.frame_id; // Use parameterized frame_id

                        aruco_det_pose.pose.position.x = tvec_target[0];
                        aruco_det_pose.pose.position.y = tvec_target[1];
                        aruco_det_pose.pose.position.z = tvec_target[2];

                        // 【重要】rvec 是旋转向量，不能直接填入四元数；先 Rodrigues 转矩阵，再转单位四元数。
                        cv::Mat rotation_matrix;
                        cv::Rodrigues(rvec_target, rotation_matrix);

                        // Using tf2 for robust quaternion conversion
                        tf2::Matrix3x3 tf_rot_matrix(
                            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
                        tf2::Quaternion tf_quat;
                        tf_rot_matrix.getRotation(tf_quat);
                        tf_quat.normalize(); // Ensure it's a unit quaternion

                        aruco_det_pose.pose.orientation = tf2::toMsg(tf_quat); // Convert tf2::Quaternion to geometry_msgs::Quaternion

                        aruco_det_pose_pub.publish(aruco_det_pose);
                        ROS_INFO_THROTTLE(1.0, "Targeting ArUco ID %d. Pose published. Z = %.3f m", current_target_id, tvec_target[2]);
                    }
                    else
                    {
                        ROS_WARN_THROTTLE(1.0, "Could not estimate pose for target ArUco ID %d, though detected.", current_target_id);
                    }
                }
                else // Current target not found in this frame
                {
                    ROS_WARN_THROTTLE(1.0, "Current target ArUco ID %d not found in the frame.", current_target_id);
                    // If current target is inner and it's lost, the switching logic above already handles incrementing lost_count.
                    // If current target is outer and it's lost, we just report it.
                    // Consider if we should default to outer if inner is lost and outer is visible (already handled by switching logic)
                }
            }
            // 【重要：失效行为】没有有效目标时不发布新位姿；没有额外发布“丢失”状态。
            else // No markers detected at all
            {
                ROS_WARN_THROTTLE(1.0, "No ArUco markers detected in the frame.");
                if (current_target_id == aruco_id_inner)
                {
                    inner_marker_lost_count++;
                    ROS_WARN_THROTTLE(1.0, "No markers, and current target is inner. Lost count: %d", inner_marker_lost_count);
                    if (inner_marker_lost_count >= inner_marker_lost_threshold_frames)
                    {
                        current_target_id = aruco_id_outer; // Default back to outer if inner is lost for too long and no markers are seen
                        current_target_length = aruco_length_outer;
                        ROS_WARN("No markers and inner target lost for %d frames. Defaulting to Outer ArUco ID %d.", inner_marker_lost_count, current_target_id);
                    }
                }
            }
            // 【次要：调试显示】窗口和标注图像便于观察检测效果，计算位姿本身不依赖窗口。
            if (show_debug_image)
            {
                cv::imshow("ArUco Detection", current_frame);
                cv::waitKey(1);
            }

            // Publish the image with detections
            cv_image.header = image_header;
            if (cv_image.header.frame_id.empty()) cv_image.header.frame_id = camera_frame_id; // Or a specific image frame if different
            cv_image.encoding = "bgr8";
            cv_image.image = current_frame;
            aruco_det_image_pub.publish(cv_image.toImageMsg());
        }

        // 【重要：调度】每轮开头处理回调；此处限制循环频率。
        loop_rate.sleep();
    }
    if (show_debug_image) cv::destroyAllWindows();
    return 0;
}