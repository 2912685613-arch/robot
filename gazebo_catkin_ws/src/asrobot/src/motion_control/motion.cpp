
#include "../../include/asrobot/f_single_robot_control.h"
#include <sstream>
#include <cmath>
#include "asrobot/aruco_docking_control.h"

// ===================================================================================
//    全局发布变量
// ===================================================================================
ros::Publisher pub_pwm;
ros::Publisher pub_status;

ros::NodeHandle *extern_n;
ros::Publisher *extern_pub_pwm;
ros::Publisher *extern_pub_status;
ros::Publisher mc_pub_debug;

// ===================================================================================
//    全局消息变量（发布）
// ===================================================================================
asrobot::pwm pwm_msg; // 输出pwm信息
asrobot::status key;  // 输出舵机开关
asrobot::mc motion_debug;
// ===================================================================================
//    全局订阅变量
// ===================================================================================
ros::Subscriber sub_depth;
ros::Subscriber sub_cmd;
ros::Subscriber sub_trajectory;
ros::Subscriber sub_multiRobotPosition;

// ===================================================================================
// 1. 状态机定义 (核心)
// ===================================================================================
enum class RobotMode
{
    IDLE,
    GAIT_CONTROL,
    MANUAL_YAW,        // 手动航向-H型,只有Yaw特殊，因为底层的运动yaw是计算出来的，所以单独控制yaw需要改debug_mode
    MANUAL_DEPTH,      // 手动深度-H型
    MANUAL_DEPTH_YAW,  // 航向、深度-H型
    MANUAL_XYZ_TARGET, // XYZ-H型
    DOCKING_ARUCO,     // 对接-识别aruco并运动
    GRASP,             // 以对腿抓取
    DEBUGVZ,           // 调节双环内环Z
    DEBUGPZ            // 调节双环外环Z
};

// ===================================================================================
// 日志辅助函数
// ===================================================================================
void print_header(const std::string &title, bool is_state_change = false)
{
    const int total_width = 60;
    std::string border(total_width, '=');
    std::string state_prefix = is_state_change ? "STATE CHANGE: " : "";
    std::string full_title = state_prefix + title;

    int padding_total = total_width - 2 - full_title.length();
    if (padding_total < 0)
        padding_total = 0; // 防止标题过长
    int padding_left = padding_total / 2;
    int padding_right = padding_total - padding_left;

    std::stringstream ss;
    ss << "\n"
       << border << "\n"
       << "= " << std::string(padding_left, ' ') << full_title << std::string(padding_right, ' ') << " =\n"
       << border;

    ROS_INFO_STREAM(ss.str());
}

// 全局变量，保存当前状态
RobotMode g_current_state = RobotMode::IDLE;
bool fuzzy_pid_enabled = true;

// ===================================================================================
// 2. 逻辑处理函数 (从 mccallback 和 main 循环中拆分)
// ===================================================================================

void handle_idle()
{
    // 在此状态下，确保电机停止
    // 你可以取消下面这行的注释，以确保在IDLE状态下电机总是停止的
    vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
}

void handle_gait_control()
{
    gaitControl();
}

void handle_manual_yaw()
{
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "--- YAW CONTROL ---\n"
                                 << "  [Target]: " << exp_yaw << "\n"
                                 << "  [Actual]: " << cur_yaw);
    combine3DPidCtrl.setDebugMode(DEBUG_YAW_ONLY, exp_yaw);
    PidYawControl_H(exp_yaw, cur_yaw);
}

void handle_manual_depth()
{
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "--- DEPTH CONTROL ---\n"
                                 << "  [Target]: " << exp_p_z << "\n"
                                 << "  [Actual]: " << cur_p_z);
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    PidDepthControl_X(exp_p_z, cur_p_z);
}

void handle_manual_depthyaw()
{
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "--- DEPTH CONTROL ---\n"
                                 << "  [Target]: " << exp_p_z << "\n"
                                 << "  [Actual]: " << cur_p_z << "\n"
                                 << "--- YAW CONTROL ---\n"
                                 << "  [Target]: " << exp_yaw << "\n"
                                 << "  [Actual]: " << cur_yaw);
    combine3DPidCtrl.setDebugMode(DEBUG_DEPYAW_ONLY, exp_yaw);
    PidDepthYawControl_H(exp_p_z, exp_v_z, exp_yaw, cur_p_z, cur_v_z, cur_yaw);
}

void handle_xyz_target()
{
    combine3DPidCtrl.setXYZPIDLogEnabled(true);
    PidXYZControl_X(exp_p_x, exp_p_y, exp_p_z, cur_p_x, cur_p_y, cur_p_z, cur_yaw);
    combine3DPidCtrl.setXYZPIDLogEnabled(false);
}

void xyzLogTimerCallback(const ros::WallTimerEvent &)
{
    if (g_current_state != RobotMode::MANUAL_XYZ_TARGET)
        return;

    const std::string &snapshot = combine3DPidCtrl.getXYZPIDSnapshot();
    const double age = snapshot.empty() ? 0.0 :
        (ros::WallTime::now() - combine3DPidCtrl.getXYZPIDSnapshotTime()).toSec();
    ROS_INFO_STREAM("--- XYZ TARGET CONTROL ---\n"
                    << "  [Target] X: " << exp_p_x << ", Y: " << exp_p_y << ", Z: " << exp_p_z << "\n"
                    << "  [Actual] X: " << cur_p_x << ", Y: " << cur_p_y << ", Z: " << cur_p_z << "\n"
                    << "  [Computed Target Yaw]: " << exp_yaw << ", [Actual Yaw]: " << cur_yaw << "\n"
                    << "  PID snapshot age (s): " << age << "\n"
                    << (snapshot.empty() ? "  Waiting for position feedback" : snapshot));
}

void handle_grasp_target()
{
    ADRCPosition3DControl_G(exp_p_x, exp_p_y, exp_yaw, exp_p_z, cur_p_x, cur_p_y, cur_yaw, cur_p_z);
}

// 对接使用独立控制器，保持已有手动PID模式和旧分配路径不变。
aruco_docking::Config docking_config;
aruco_docking::Controller docking_controller;
aruco_docking::Input docking_input;
ros::WallTime docking_last_mc, docking_steering_start;
aruco_docking::State docking_reported_state = aruco_docking::State::Idle;

void reportDockingState(aruco_docking::State state)
{
    if (state != docking_reported_state)
    {
        if (state == aruco_docking::State::Lost)
            ROS_WARN_STREAM("[DOCK STATE] " << aruco_docking::name(docking_reported_state)
                            << " -> LOST_STOP: invalid/stale feedback or transport/clock interruption; dock required to restart.");
        else
            ROS_INFO_STREAM("[DOCK STATE] " << aruco_docking::name(docking_reported_state)
                            << " -> " << aruco_docking::name(state));
        docking_reported_state = state;
    }
}

void handle_docking_aruco(const asrobot::mc::ConstPtr &msg)
{
    docking_input.now = ros::Time::now().toSec();
    docking_input.wall = ros::WallTime::now().toSec();
    docking_input.vision_valid = msg->aruco_valid;
    docking_input.feedback_valid = msg->feedback_valid;
    docking_input.vision_stamp = msg->aruco_stamp;
    docking_input.feedback_stamp = msg->feedback_stamp;
    docking_input.x = msg->cam_e_x; docking_input.y = msg->cam_e_y; docking_input.z = msg->cam_e_z;
    docking_input.vx = msg->body_vx; docking_input.vy = msg->body_vy; docking_input.vz = msg->body_vz;
    docking_input.yaw = msg->body_yaw_deg; docking_input.yaw_rate = msg->body_yaw_rate;
    // 水平转腿期间电机关闭；不拿转腿瞬态作为位置保持稳定证据。
    if ((ros::WallTime::now() - docking_steering_start).toSec() < 1.0)
    {
        handle_idle();
        return;
    }
    const auto output = docking_controller.update(docking_input);
    reportDockingState(output.state);
    if (!output.active)
    {
        handle_idle();
        return;
    }
    combine3DPidCtrl.vec_ctrl.dockingControl(output.fx, output.fy, output.fz, output.mz);
    ROS_INFO_STREAM_THROTTLE(0.5, "[DOCK CONTROL] state=" << aruco_docking::name(output.state)
        << " source_stamp=" << docking_input.vision_stamp
        << " age=" << docking_input.now-docking_input.vision_stamp
        << " xyz=" << docking_input.x << "," << docking_input.y << "," << docking_input.z
        << " xy_threshold=" << output.xy_threshold << " z_error=" << output.z_error
        << " target_vz=" << output.target_vz << " body_vz=" << docking_input.vz
        << " yaw_target=" << output.yaw_target << " yaw=" << docking_input.yaw
        << " yaw_error=" << output.yaw_error << " Fz=" << output.fz << " Mz=" << output.mz);
}

void dockingWatchdog(const ros::WallTimerEvent &)
{
    if (g_current_state != RobotMode::DOCKING_ARUCO) return;
    const double silence = (ros::WallTime::now()-docking_last_mc).toSec();
    if (silence > docking_config.timeout)
    {
        // 断掉 mc_message 或停止 /clock 时也会执行，不依赖订阅回调触发停机。
        reportDockingState(docking_controller.lost().state);
        handle_idle();
    }
}

// =================================================================
// ===                        PID双环调参执行函数                  ===
// =================================================================
void handle_debug_vz()
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    combine3DPidCtrl.setHeaveInnerOnly(true);
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   0, 0, 0, exp_p_z, 0, 0, 0,
                                   cur_p_x, cur_p_y, cur_yaw, cur_p_z, cur_v_x, cur_v_y, cur_v_z);
}
void handle_debug_pz()
{
    combine3DPidCtrl.setDebugMode(NAV_NORMAL);
    combine3DPidCtrl.setHeaveInnerOnly(false);
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   0, 0, 0, exp_p_z, 0, 0, 0,
                                   0, 0, cur_yaw, cur_p_z, cur_v_x, cur_v_y, cur_v_z);
}
// ===================================================================================
// 3. 回调函数
// ===================================================================================

void mccallback(const asrobot::mc::ConstPtr &msg)
{
    // 统一更新所有当前状态变量
    cur_p_x = msg->c_x;
    cur_v_x = msg->c_speedx;
    cur_p_y = msg->c_y;
    cur_v_y = msg->c_speedy;
    cur_p_z = msg->c_z;
    cur_v_z = msg->c_speedz;
    cur_cla_vz = msg->c_calculatedz;

    cur_yaw = msg->c_navyaw;
    cur_roll = msg->c_navroll;
    cur_pitch = msg->c_navpitch;

    cam_exp_p_x = msg->cam_e_x;
    cam_exp_p_y = msg->cam_e_y;
    cam_exp_p_z = msg->cam_e_z;
    cam_cur_p_x = msg->cam_c_x;
    cam_cur_p_y = msg->cam_c_y;
    cam_cur_p_z = msg->cam_c_z;
    cam_cur_yaw = msg->cam_c_yaw;
    cam_exp_yaw = msg->cam_e_yaw;

    // 总开关仅作用于手动 PID 命令，不改变自主对接及 ADRC/步态算法。
    const bool manual_pid = g_current_state == RobotMode::MANUAL_YAW ||
        g_current_state == RobotMode::MANUAL_DEPTH || g_current_state == RobotMode::MANUAL_DEPTH_YAW ||
        g_current_state == RobotMode::MANUAL_XYZ_TARGET || g_current_state == RobotMode::DEBUGVZ ||
        g_current_state == RobotMode::DEBUGPZ;
    combine3DPidCtrl.setManualPIDActive(manual_pid);
    combine3DPidCtrl.setFuzzyPIDEnabled(manual_pid && fuzzy_pid_enabled);
    combine3DPidCtrl.setControlMode(g_current_state == RobotMode::DEBUGVZ ||
        g_current_state == RobotMode::DEBUGPZ ? C_Combine3DPidControl::DUAL_LOOP : C_Combine3DPidControl::SINGLE_LOOP);
    docking_last_mc = ros::WallTime::now();
    if (g_current_state != RobotMode::DOCKING_ARUCO)
    {
        // dock命令捕获最新有效反馈航向，避免强制转向零度或随小码姿态跳变。
        docking_input.now = ros::Time::now().toSec();
        docking_input.feedback_valid = msg->feedback_valid;
        docking_input.feedback_stamp = msg->feedback_stamp;
        docking_input.yaw = msg->body_yaw_deg;
    }

    // 根据当前状态，调用对应的处理函数
    // 注意：这里的逻辑现在是事件驱动的，每次收到mc_message都会执行
    switch (g_current_state)
    {
    case RobotMode::IDLE:
        handle_idle();
        break;
    case RobotMode::GAIT_CONTROL:
        handle_gait_control();
        break;
    case RobotMode::MANUAL_YAW:
        handle_manual_yaw();
        break;
    case RobotMode::MANUAL_DEPTH:
        handle_manual_depth();
        break;
    case RobotMode::MANUAL_DEPTH_YAW:
        handle_manual_depthyaw();
        break;
    case RobotMode::MANUAL_XYZ_TARGET:
        handle_xyz_target();
        break;
    case RobotMode::DOCKING_ARUCO:
        handle_docking_aruco(msg);
        break;
    case RobotMode::GRASP:
        handle_grasp_target();
        break;
    case RobotMode::DEBUGVZ:
        handle_debug_vz();
        break;
    case RobotMode::DEBUGPZ:
        handle_debug_pz();
        break;
    default:
        ROS_WARN_THROTTLE(5.0, "In unhandled or default state!");
        handle_idle();
        break;
    }
}

void cmdcallback(const std_msgs::String::ConstPtr &msg)
{
    std::stringstream ss(msg->data);
    std::string buf;
    std::vector<std::string> cmd;
    while (ss >> buf)
    {
        cmd.push_back(buf);
    }

    if (cmd.empty())
        return;

    const RobotMode previous_state = g_current_state;

    // --- 状态切换命令 ---
    if (cmd[0] == "gait" && cmd.size() == 2)
    {
        g_current_state = RobotMode::GAIT_CONTROL;
        gait_R = std::stod(cmd[1]);
        print_header("-> GAIT CONTROL", true);
    }
    else if (cmd[0] == "yaw" && cmd.size() == 2)
    {
        g_current_state = RobotMode::MANUAL_YAW;
        exp_yaw = std::stod(cmd[1]);
        print_header("-> MANUAL YAW", true);
    }
    else if (cmd[0] == "dep" && cmd.size() == 2)
    {
        g_current_state = RobotMode::MANUAL_DEPTH;
        exp_p_z = std::stod(cmd[1]);
        print_header("-> MANUAL DEPTH", true);
    }
    // else if (cmd[0] == "dep" && cmd.size() == 2)
    // {
    //     g_current_state = RobotMode::MANUAL_DEPTH;
    //     exp_v_z = std::stod(cmd[1]);
    //     print_header("-> MANUAL DEPTH", true);
    // }
    else if (cmd[0] == "depyaw" && cmd.size() == 3)
    {
        g_current_state = RobotMode::MANUAL_DEPTH_YAW;
        exp_p_z = std::stod(cmd[1]);
        exp_yaw = std::stod(cmd[2]);
        print_header("-> MANUAL DEPTH", true);
    }
    else if (cmd[0] == "xyz" && cmd.size() == 4)
    {
        g_current_state = RobotMode::MANUAL_XYZ_TARGET;
        exp_p_x = std::stod(cmd[1]);
        exp_p_y = std::stod(cmd[2]);
        exp_p_z = std::stod(cmd[3]);
        print_header("-> XYZ TARGET", true);
    }
    else if (cmd[0] == "fuzzypid" && cmd.size() == 2 && (cmd[1] == "on" || cmd[1] == "off"))
    {
        const bool enabled = cmd[1] == "on";
        if (enabled != fuzzy_pid_enabled)
            combine3DPidCtrl.resetManualPIDs();
        fuzzy_pid_enabled = enabled;
        ROS_INFO("Manual fuzzy PID %s (yaw/dep/depyaw/xyz/vz/pz)", enabled ? "on" : "off");
    }
    else if (cmd[0] == "fuzzyxyz")
    {
        ROS_WARN("Use fuzzypid on/off to switch all manual PID controllers.");
    }
    else if (cmd[0] == "dock") // docking-aruco
    {                          // trackaruco
        g_current_state = RobotMode::DOCKING_ARUCO;
        const double now = ros::Time::now().toSec();
        const bool yaw_fresh = docking_input.feedback_valid && std::isfinite(docking_input.yaw) &&
            now-docking_input.feedback_stamp >= 0 && now-docking_input.feedback_stamp <= docking_config.timeout &&
            (ros::WallTime::now()-docking_last_mc).toSec() <= docking_config.timeout;
        docking_controller.start(ros::WallTime::now().toSec(), docking_input.yaw, yaw_fresh);
        docking_steering_start = ros::WallTime::now();
        docking_last_mc = ros::WallTime::now();
        docking_reported_state = aruco_docking::State::Idle;
        reportDockingState(docking_controller.state());
        handle_idle();
        combine3DPidCtrl.vec_ctrl.prepareDockingLegs();
    }
    else if (cmd[0] == "st")
    {
        g_current_state = RobotMode::IDLE;
        vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
        clearADRCSpeedControllerVariables();
        gait_stop();
        legSetAngle(0, 0, 0);
        print_header("-> IDLE (SYSTEM STOPPED)", true);
    }
    else if (cmd[0] == "grasp") // grasp x y z
    {
        g_current_state = RobotMode::GRASP;
        exp_p_x = std::stod(cmd[1]);
        exp_p_y = std::stod(cmd[2]);
        exp_p_z = std::stod(cmd[3]);
        exp_yaw = atan2(exp_p_y - cur_p_y, exp_p_x - cur_p_x) * 57.3;
        print_header("-> XYZ TARGET GRASP", true);
    }
    else if (cmd[0] == "pidspeed") // set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            setSpeedPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set speed-pid parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }
    else if (cmd[0] == "pidyaw") // set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            setYawPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set yaw-pid parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }
    else if (cmd[0] == "piddep") // set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            setDepthPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set dep-pid parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }
    else if (cmd[0] == "pidx") // set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            setXPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set dep-X parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }
    else if (cmd[0] == "pidy") // set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            setYPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set dep-Y parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }
    else if (cmd[0] == "a") // leg angle
    {
        float A, B, C;
        A = atof(cmd[1].c_str());
        B = atof(cmd[2].c_str());
        C = atof(cmd[3].c_str());
        printf("set angle\n");
        legSetAngle(A, B, C);
    }
    // =================================================================
    // ===                        PID双环调参cmd                      ===
    // =================================================================
    else if (cmd[0] == "vz") // z的速度，内环调参，提前在函数中设置好了期望
    {
        print_header("-> DEBUG-VZ", true);
        g_current_state = RobotMode::DEBUGVZ;
    }
    else if (cmd[0] == "pidvz") // 调节z的内环pid
    {
        if (cmd.size() == 4)
        {
            setVelDepPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set vel-dep-pid parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }
    else if (cmd[0] == "pz" && cmd.size() == 2) // z的位置，外环调参，手动给期望位置
    {
        exp_p_z = std::stod(cmd[1]);
        print_header("-> DEBUG-PZ", true);
        g_current_state = RobotMode::DEBUGPZ;
    }
    else if (cmd[0] == "pidpz") // 调节z的外环pid
    {
        if (cmd.size() == 4)
        {
            setPosDepPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            cout << "set pos-dep-pid parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
        }
    }

    if (g_current_state != previous_state)
    {
        combine3DPidCtrl.resetManualPIDs();
        if (g_current_state != RobotMode::DOCKING_ARUCO) docking_controller.stop();
    }
    else if (cmd[0] == "xyz" || cmd[0] == "yaw" || cmd[0] == "dep" ||
             cmd[0] == "depyaw" || cmd[0] == "pz" || cmd[0] == "vz")
        combine3DPidCtrl.resetManualPIDHistory();

    // ... 其他命令 ...
}

// 其他回调函数和工具函数保持不变
void Spine_pathCallback(const asrobot::mul_ref_trajectory::ConstPtr &msg) { /* ... */ }
void multiRobotPosition_Callback(const asrobot::quadrotor2robot::ConstPtr &msg) { /* ... */ }

void reference_trajectory_to_file(MatrixXd r1, MatrixXd r2, MatrixXd r3, MatrixXd vr) { /* ... */ }
void saveDataTimerCallback(const ros::TimerEvent &event) { /* ... */ }

// ===================================================================================
// 4. 主函数 (main)
// ===================================================================================
int main(int argc, char **argv)
{

    ros::init(argc, argv, "motion");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    private_nh.param("docking/target_z", docking_config.target_z, docking_config.target_z);
    private_nh.param("docking/timeout", docking_config.timeout, docking_config.timeout);
    private_nh.param("docking/wait_timeout", docking_config.wait_timeout, docking_config.wait_timeout);
    private_nh.param("docking/max_speed", docking_config.max_speed, docking_config.max_speed);
    private_nh.param("docking/near_speed", docking_config.near_speed, docking_config.near_speed);
    private_nh.param("docking/near_distance", docking_config.near_distance, docking_config.near_distance);
    private_nh.param("docking/acceleration", docking_config.acceleration, docking_config.acceleration);
    private_nh.param("docking/position_kp", docking_config.position_kp, docking_config.position_kp);
    private_nh.param("docking/velocity_kp", docking_config.velocity_kp, docking_config.velocity_kp);
    private_nh.param("docking/velocity_ki", docking_config.velocity_ki, docking_config.velocity_ki);
    private_nh.param("docking/z_force_limit", docking_config.z_force_limit, docking_config.z_force_limit);
    private_nh.param("docking/xy_kp", docking_config.xy_kp, docking_config.xy_kp);
    private_nh.param("docking/xy_kd", docking_config.xy_kd, docking_config.xy_kd);
    private_nh.param("docking/xy_force_limit", docking_config.xy_force_limit, docking_config.xy_force_limit);
    private_nh.param("docking/yaw_kp", docking_config.yaw_kp, docking_config.yaw_kp);
    private_nh.param("docking/yaw_kd", docking_config.yaw_kd, docking_config.yaw_kd);
    private_nh.param("docking/yaw_limit", docking_config.yaw_limit, docking_config.yaw_limit);
    private_nh.param("docking/xy_tolerance", docking_config.xy_tolerance, docking_config.xy_tolerance);
    private_nh.param("docking/xy_hysteresis", docking_config.xy_hysteresis, docking_config.xy_hysteresis);
    private_nh.param("docking/z_tolerance", docking_config.z_tolerance, docking_config.z_tolerance);
    private_nh.param("docking/velocity_tolerance", docking_config.velocity_tolerance, docking_config.velocity_tolerance);
    private_nh.param("docking/yaw_tolerance", docking_config.yaw_tolerance, docking_config.yaw_tolerance);
    private_nh.param("docking/settle_time", docking_config.settle_time, docking_config.settle_time);
    if (!docking_config.valid())
    {
        ROS_FATAL("Invalid docking configuration; check docking_control.yaml.");
        return 1;
    }
    docking_controller = aruco_docking::Controller(docking_config);
    extern_n = &n; // 保持对旧代码的兼容
    // /clock暂停时仍处理墙钟看门狗，不能在ros::Rate::sleep中阻塞停机回调。
    ros::WallRate loop_rate(iMotionLoopRate);

    // initMpcOsqp();

    // 发布者
    pub_pwm = n.advertise<asrobot::pwm>("pwm_message", 10);
    pub_status = n.advertise<asrobot::status>("status_message", 10);
    mc_pub_debug = n.advertise<asrobot::mc>("motion_debug_topic", 10);
    extern_pub_pwm = &pub_pwm;
    extern_pub_status = &pub_status;

    // 订阅者
    sub_depth = n.subscribe("mc_message", 100, mccallback); // 所有要执行的程序在这里
    sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);  // 所有的命令在这个函数里面
    // sub_trajectory = n.subscribe("mul_ref_trajectory", 10, Spine_pathCallback);
    // sub_multiRobotPosition = n.subscribe("multi_RobotPosition", 10, multiRobotPosition_Callback);

    // ros::Duration(1.0).sleep();
    //  定时器
    ros::Timer timer_saveData = n.createTimer(ros::Duration(0.1), saveDataTimerCallback); // 假设 DURATION 是 0.1
    ros::WallTimer timer_docking = n.createWallTimer(ros::WallDuration(0.05), dockingWatchdog);
    ros::WallTimer timer_xyzLog = n.createWallTimer(ros::WallDuration(1.0 / 30.0), xyzLogTimerCallback);

    // 控制器参数初始化
    simMotorForce(0, 0, 0, 0, 0, 0, 0);
    /// PID 控制器参数设置
    // setYawPIDPmt(0.1, 0.005, 0.01);
    setYawPIDPmt(0.025, 0.011, 0.06); // 航向
    setDepthPIDPmt(1.6, 0.02, 0.1);
    setPitchPIDPmt(0.5, 0.04, 0.15);
    setSpeedPIDPmt(0.8, 0, 0.2); // H型前进
    setXPIDPmt(0.6, 0, 0.5);
    setYPIDPmt(0.6, 0, 0.5);

    setPxADRCPmt(1.5, 0.1, 27, 0);
    setPyADRCPmt(1.5, 0.1, 27, 0);
    setPzADRCPmt(0, 0, 27, 0);
    setYawADRCPmt(0.1, 0.0001, 25, 0.5);

    LegShape_X();

    ROS_INFO("Motion control node started. Current state: IDLE");

    while (ros::ok())
    {

        combine3DPidCtrl.debug_msg(mc_pub_debug); // 仿真调参用

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
