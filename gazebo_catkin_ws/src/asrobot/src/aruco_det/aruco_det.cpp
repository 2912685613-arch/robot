#include "ros/ros.h"
#include "sensor_msgs/Image.h"
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

cv::Mat frame; // Global frame variable

void cam_imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image.clone(); // Use clone to ensure data is copied if cv_ptr goes out of scope
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_det_node");
    ros::NodeHandle n;
    ros::NodeHandle nh1("~"); // Private NodeHandle for parameters

    // --- Parameters for ArUco detection ---
    int dictionary_id;
    std::string camera_param_path;
    std::string sub_image_topic;
    std::string camera_frame_id; // New parameter for camera frame_id

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
    nh1.param<float>("switch_depth_threshold_to_outer", switch_depth_threshold_to_outer, 0.4f);  // e.g., if inner is further than 0.25m
    nh1.param<int>("inner_marker_lost_threshold_frames", inner_marker_lost_threshold_frames, 100); // e.g., 10 frames

    nh1.param<std::string>("camera_param_path", camera_param_path, "/home/jll/gazebo_catkin_ws/src/asrobot/config/camera_gazebo.yaml");
    nh1.param<std::string>("sub_image_topic", sub_image_topic, "/gazebo_robot1/camera_bottom/camera_bottom/image_raw");
    nh1.param<std::string>("camera_frame_id", camera_frame_id, "camera_link"); // Default camera frame

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

    ros::Publisher aruco_det_image_pub = n.advertise<sensor_msgs::Image>("aruco_det_image", 10);
    ros::Subscriber cam_sub = n.subscribe(sub_image_topic, 1, cam_imageCallback);
    ros::Publisher aruco_det_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/aruco/pose", 10);

    cv_bridge::CvImage cv_image;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(camera_param_path);
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("Failed to load camera parameters from %s: %s", camera_param_path.c_str(), e.what());
        return 1; // Exit if camera params can't be loaded
    }

    double fx = config["fx"].as<double>();
    double fy = config["fy"].as<double>();
    double cx = config["x0"].as<double>();
    double cy = config["y0"].as<double>();
    double k1 = config["k1"].as<double>();
    double k2 = config["k2"].as<double>();
    double p1 = config["p1"].as<double>();
    double p2 = config["p2"].as<double>();
    double k3 = config["k3"].as<double>();

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                            0, fy, cy,
                            0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

    // --- State variables for target switching ---
    int current_target_id = aruco_id_outer; // Initial target
    float current_target_length = aruco_length_outer;
    int inner_marker_lost_count = 0; // Counter for how many consecutive frames inner marker is lost

    ROS_INFO("Initially targeting ArUco ID %d (Outer)", current_target_id);

    ros::Rate loop_rate(10); // Loop rate e.g., 10 Hz

    while (ros::ok())
    {
        if (!frame.empty())
        {
            cv::Mat current_frame = frame.clone(); // Work on a copy of the frame
            cv::Mat gray;
            cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

            if (markerIds.size() > 0)
            {
                cv::aruco::drawDetectedMarkers(current_frame, markerCorners, markerIds);

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

                        cv::aruco::drawAxis(current_frame, cameraMatrix, distCoeffs, rvec_target, tvec_target, current_target_length * 0.5f);

                        geometry_msgs::PoseStamped aruco_det_pose;
                        aruco_det_pose.header.stamp = ros::Time::now();
                        aruco_det_pose.header.frame_id = camera_frame_id; // Use parameterized frame_id

                        aruco_det_pose.pose.position.x = tvec_target[0];
                        aruco_det_pose.pose.position.y = tvec_target[1];
                        aruco_det_pose.pose.position.z = tvec_target[2];

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
            cv::imshow("ArUco Detection", current_frame);
            cv::waitKey(1);

            // Publish the image with detections
            cv_image.header.stamp = ros::Time::now();
            cv_image.header.frame_id = camera_frame_id; // Or a specific image frame if different
            cv_image.encoding = "bgr8";
            cv_image.image = current_frame;
            aruco_det_image_pub.publish(cv_image.toImageMsg());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}