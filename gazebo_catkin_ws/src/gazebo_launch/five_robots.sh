#!/bin/bash
# 启动水下世界、五台机器人以及各自的关节位置控制器。
# 机器人之间通过 ROS namespace 隔离；URDF 中的执行关节名保持不带 namespace。

set -u

WS="${HOME}/robot/gazebo_catkin_ws"
LOG_DIR="${WS}/logs"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

source "${WS}/devel/setup.bash"
CONTROL_CONFIG_DIR="$(rospack find gazebo_control)/config"

launch_terminal() {
    local title="$1"
    local log_file="$2"
    shift 2
    local command="$*"

    dbus-launch gnome-terminal --title="${title}" -- bash -c "
        source '${WS}/devel/setup.bash'
        set -o pipefail
        ${command} 2>&1 | tee '${log_file}'
        status=\${PIPESTATUS[0]}
        printf '\\nroslaunch exit status: %s\\n' \"\${status}\"
        exec bash
    " &
}

mkdir -p "${LOG_DIR}"

# Gazebo world
launch_terminal "world" \
    "${LOG_DIR}/world_${TIMESTAMP}.log" \
    roslaunch gazebo_world underwater_world_noshadow.launch

# Wait for Gazebo's spawn services before loading the robots.
sleep 3

# Robot 1: original ASRobot model
launch_terminal "model1" \
    "${LOG_DIR}/model1_${TIMESTAMP}.log" \
    roslaunch gazebo_model gazebo_model.launch mode:=water namespace:=gazebo_robot1 x:=0 y:=0 z:=-0.1

sleep 2

# Robot 2: original ASRobot model
launch_terminal "model2" \
    "${LOG_DIR}/model2_${TIMESTAMP}.log" \
    roslaunch gazebo_model gazebo_model2.launch mode:=water namespace:=gazebo_robot2 x:=-1 y:=0 z:=-0.1

sleep 2

# Robot 3: same model as robot 2, with a different namespace and pose.
# There is no gazebo_model3.launch in this package, so reuse gazebo_model2.launch.
launch_terminal "model3" \
    "${LOG_DIR}/model3_${TIMESTAMP}.log" \
    roslaunch gazebo_model gazebo_model2.launch mode:=water namespace:=gazebo_robot3 x:=1 y:=0 z:=-0.1

sleep 2

# Robot 4: eight-thruster camera model 1
launch_terminal "model4" \
    "${LOG_DIR}/model4_${TIMESTAMP}.log" \
    roslaunch gazebo_model gazebo_eight1.launch mode:=water namespace:=gazebo_eight_camera1 x:=0 y:=-1 z:=-0.1

sleep 2

# Robot 5: eight-thruster camera model 2
launch_terminal "model5" \
    "${LOG_DIR}/model5_${TIMESTAMP}.log" \
    roslaunch gazebo_model gazebo_eight2.launch mode:=water namespace:=gazebo_eight_camera2 x:=0 y:=1 z:=-0.1

# Give the five gazebo_ros_control plugins time to register their controller managers.
sleep 4

# Each controller manager is namespaced. The joint names in the YAML files are
# deliberately unqualified: H1/H1_joint, M1/M1_joint, ..., V4/V4_joint.
launch_terminal "control1" \
    "${LOG_DIR}/control1_${TIMESTAMP}.log" \
    roslaunch --disable-title gazebo_control gazebo_control.launch \
        mode:=water namespace:=gazebo_robot1 \
        config:="${CONTROL_CONFIG_DIR}/gazebo_control_water.yaml"

launch_terminal "control2" \
    "${LOG_DIR}/control2_${TIMESTAMP}.log" \
    roslaunch --disable-title gazebo_control gazebo_control.launch \
        mode:=water namespace:=gazebo_robot2 \
        config:="${CONTROL_CONFIG_DIR}/gazebo_control_water.yaml"

launch_terminal "control3" \
    "${LOG_DIR}/control3_${TIMESTAMP}.log" \
    roslaunch --disable-title gazebo_control gazebo_control.launch \
        mode:=water namespace:=gazebo_robot3 \
        config:="${CONTROL_CONFIG_DIR}/gazebo_control_water.yaml"

launch_terminal "control4" \
    "${LOG_DIR}/control4_${TIMESTAMP}.log" \
    roslaunch --disable-title gazebo_control gazebo_control.launch \
        mode:=water namespace:=gazebo_eight_camera1 \
        config:="${CONTROL_CONFIG_DIR}/gazebo_control_eight_water.yaml"

launch_terminal "control5" \
    "${LOG_DIR}/control5_${TIMESTAMP}.log" \
    roslaunch --disable-title gazebo_control gazebo_control.launch \
        mode:=water namespace:=gazebo_eight_camera2 \
        config:="${CONTROL_CONFIG_DIR}/gazebo_control_eight2_water.yaml"
