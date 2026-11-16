cmake_minimum_required(VERSION 2.8.3)
project(asrobot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)
find_package(osqp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(qpOASES_INCLUDE_DIRS /usr/local/include/)
set(qpOASES_LIBS /usr/local/lib/libqpOASES.a)

add_message_files(
  FILES
  camera_data.msg multirobot.msg multi_robot8.msg quadrotor2robot.msg
  camera82robot.msg stereo2_pub.msg servo.msg path_3d.msg path_2d.msg
  path_based_threeSpine.msg mul_ref_trajectory.msg mc.msg pwm.msg
  achievedWaypoint.msg status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime roscpp std_msgs std_srvs cv_bridge image_transport sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIRS}
)

set(MSG_GEN_TARGET ${PROJECT_NAME}_generate_messages_cpp)

add_executable(joint_control src/gazebo_commu/joint_control.cpp)
add_dependencies(joint_control ${MSG_GEN_TARGET})
target_link_libraries(joint_control ${catkin_LIBRARIES})

add_executable(motion
  src/motion_control/motion.cpp src/motion_control/f_single_robot_control.cpp
  src/motion_control/c_controller_group.cpp src/motion_control/c_fuzzy_pid.cpp
  src/motion_control/c_gaitcontrol.cpp
  src/motion_control/c_legmodel.cpp src/motion_control/c_sim_3d_ctrl.cpp
  src/motion_control/c_vectoredthrustcontrol.cpp src/decision_center/c_filter_group.cpp
  src/motion_control/generate_trajectory.cpp src/motion_control/MPCController.cpp
  src/motion_control/mpc_formation.cpp src/motion_control/OSQP_solver.cpp
  src/motion_control/qpOASES_solver.cpp src/motion_control/c_save_data.cpp
)
add_dependencies(motion ${MSG_GEN_TARGET})
target_link_libraries(motion PRIVATE osqp::osqp ${catkin_LIBRARIES} ${qpOASES_LIBS})

add_executable(mancontrol src/man_control/mancontrol.cpp)
add_dependencies(mancontrol ${MSG_GEN_TARGET})
target_link_libraries(mancontrol ${catkin_LIBRARIES})

add_executable(decision
  src/decision_center/decision.cpp src/decision_center/f_multi_robot_plan.cpp
  src/decision_center/c_multi_agent_algorithm.cpp src/decision_center/c_save_data.cpp
  src/decision_center/c_filter_group.cpp src/decision_center/c_docking.cpp
)
add_dependencies(decision ${MSG_GEN_TARGET})
target_link_libraries(decision ${catkin_LIBRARIES})

add_executable(aruco_det src/aruco_det/aruco_det.cpp)
add_dependencies(aruco_det ${MSG_GEN_TARGET})
target_link_libraries(aruco_det ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp ${aruco_LIBRARIES})

add_executable(aruco_posture src/aruco_det/aruco_posture.cpp)
add_dependencies(aruco_posture ${MSG_GEN_TARGET})
target_link_libraries(aruco_posture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBRARIES})

add_executable(thruster_control src/gazebo_commu/thruster_control.cpp)
add_dependencies(thruster_control ${MSG_GEN_TARGET})
target_link_libraries(thruster_control ${catkin_LIBRARIES})

# add_executable(StereoVision_pub
#   src/vision_module_tracking/c_filter_group.cpp src/vision_module_tracking/fhog.cpp
#   src/vision_module_tracking/kcftracker.cpp src/vision_module_tracking/stereocamera.cpp
#   src/vision_module_tracking/StereoVision_pub.cpp
# )
# add_dependencies(StereoVision_pub ${MSG_GEN_TARGET})
# target_link_libraries(StereoVision_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# 只订阅和记录的 ArUco 诊断工具，不参与控制回路。
catkin_install_python(PROGRAMS scripts/aruco_pose_logger.py scripts/tuning_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 浏览器调参台的辅助模块和离线静态资源。
install(FILES scripts/tuning_dashboard_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY web config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
