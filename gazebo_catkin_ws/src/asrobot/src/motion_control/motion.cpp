
#include "../../include/asrobot/f_single_robot_control.h"
#include <sstream>

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
bool fuzzy_xyz_enabled = true;

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
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "--- XYZ TARGET CONTROL ---\n"
                                 << "  [Target] X: " << exp_p_x << ", Y: " << exp_p_y << ", Z: " << exp_p_z << "\n"
                                 << "  [Actual] X: " << cur_p_x << ", Y: " << cur_p_y << ", Z: " << cur_p_z << "\n"
                                 << "  [Computed Target Yaw]: " << exp_yaw << ", [Actual Yaw]: " << cur_yaw);

    combine3DPidCtrl.setFuzzyXYZEnabled(fuzzy_xyz_enabled);
    PidXYZControl_X(exp_p_x, exp_p_y, exp_p_z, cur_p_x, cur_p_y, cur_p_z, cur_yaw);
    combine3DPidCtrl.setFuzzyXYZEnabled(false);
}

void handle_grasp_target()
{
    ADRCPosition3DControl_G(exp_p_x, exp_p_y, exp_yaw, exp_p_z, cur_p_x, cur_p_y, cur_yaw, cur_p_z);
}

void handle_docking_aruco(const asrobot::mc::ConstPtr &msg) // 对接
{
    // ------------------------------纯X型运动----------------------------------
    // cam_cur_p_z = cam_cur_p_z + 0.1;
    // PidXYZControl_X(cam_exp_p_x, cam_exp_p_y, cam_exp_p_z, cam_cur_p_x, cam_cur_p_y, cam_cur_p_z, cam_cur_yaw);

    // ------------------------------H-X型运动----------------------------------
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "--- DOCKING : STAGE 1 (XY ALIGN) ---\n"
                                 << "  [Target] X: " << cam_exp_p_x << ", Y: " << cam_exp_p_y << ", Z: " << cam_exp_p_z << "\n"
                                 << "  [Actual] X: " << cam_cur_p_x << ", Y: " << cam_cur_p_y << ", Z: " << cam_cur_p_z);
    // PidXYZControl_H(cam_exp_p_x, cam_exp_p_y, 0, cam_cur_p_x, cam_cur_p_y, 0, cam_cur_yaw);

    bool flag_xy = true;
    bool flag_z = false;

    if (flag_xy == true)
    {
        ROS_INFO_STREAM_THROTTLE(1.0,
                                 "--- DOCKING : STAGE 1 (XY ALIGN) ---\n"
                                     << "  [Target] X: " << cam_exp_p_x << ", Y: " << cam_exp_p_y << ", Z: " << cam_exp_p_z << "\n"
                                     << "  [Actual] X: " << cam_cur_p_x << ", Y: " << cam_cur_p_y << ", Z: " << cam_cur_p_z);
        PidXYZControl_X(cam_exp_p_x, cam_exp_p_y, 0, cam_cur_p_x, cam_cur_p_y, 0, cam_cur_yaw);

        if (abs(cam_exp_p_x - cam_cur_p_x) < 0.05 && abs(cam_exp_p_y - cam_cur_p_y) < 0.05)
        {
            ROS_WARN(">>> CONDITION MET! Switching to Stage 2 (YAW ALIGN)...");
            flag_xy = false;
            flag_z = true;
            combine3DPidCtrl.vec_ctrl.mcSpeedPID.pidReset();
        }
    }
    if (flag_z == true)
    {
        print_header("-> DOCKING : STAGE 2 (YAW ALIGN)", true);
        ROS_INFO_STREAM_THROTTLE(1.0,
                                 "--- DEPTH CONTROL ---\n"
                                     << "  [Target]: " << cam_exp_p_z << "\n"
                                     << "  [Actual]: " << cam_cur_p_z << "\n"
                                     << "--- YAW CONTROL ---\n"
                                     << "  [Target]: " << cam_exp_yaw << "\n"
                                     << "  [Actual]: " << cam_cur_yaw);
        cam_cur_p_z = cam_cur_p_z ;
        // 单环
        // PidXYZControl_X(cam_exp_p_x, cam_exp_p_y, cam_exp_p_z, cam_cur_p_x, cam_cur_p_y, cam_cur_p_z, cam_cur_yaw);
        // 双环
        PidXYZControl_X(cam_exp_p_x, cam_exp_p_y, cam_exp_p_z, cam_cur_p_x, cam_cur_p_y, cam_cur_p_z, cam_cur_yaw, cur_v_z);
    }
}

// =================================================================
// ===                        PID双环调参执行函数                  ===
// =================================================================
void handle_debug_vz()
{
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   0, 0, 0, 0, 0, 0, 0,
                                   cur_p_x, cur_p_y, cur_yaw, cur_p_z, cur_v_x, cur_v_y, cur_v_z);
    combine3DPidCtrl.updateControl(SHAPE_X,
                                   0, 0, 0, exp_p_z, 0, 0, 0,
                                   cur_p_x, cur_p_y, cur_yaw, cur_p_z, cur_v_x, cur_v_y, cur_v_z);
}
void handle_debug_pz()
{
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
        combine3DPidCtrl.resetFuzzyXYZ();
        g_current_state = RobotMode::MANUAL_XYZ_TARGET;
        exp_p_x = std::stod(cmd[1]);
        exp_p_y = std::stod(cmd[2]);
        exp_p_z = std::stod(cmd[3]);
        print_header("-> XYZ TARGET", true);
    }
    else if (cmd[0] == "fuzzyxyz" && cmd.size() == 2 && (cmd[1] == "on" || cmd[1] == "off"))
    {
        fuzzy_xyz_enabled = cmd[1] == "on";
        combine3DPidCtrl.resetFuzzyXYZ();
        combine3DPidCtrl.vec_ctrl.mcXPID.pidReset();
        combine3DPidCtrl.vec_ctrl.mcYPID.pidReset();
        combine3DPidCtrl.vec_ctrl.mcDepthPID.pidReset();
        ROS_INFO("XYZ fuzzy PID %s", fuzzy_xyz_enabled ? "on" : "off");
    }
    else if (cmd[0] == "dock") // docking-aruco
    {                          // trackaruco
        g_current_state = RobotMode::DOCKING_ARUCO;
        ROS_INFO_STREAM_THROTTLE(1.0,
                                 "--- BEDUGGGGGGGGGGGGGG ---\n");
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

    if (previous_state == RobotMode::MANUAL_XYZ_TARGET && g_current_state != previous_state)
        combine3DPidCtrl.resetFuzzyXYZ();

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
    extern_n = &n; // 保持对旧代码的兼容
    ros::Rate loop_rate(iMotionLoopRate);

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

    // 控制器参数初始化
    simMotorForce(0, 0, 0, 0, 0, 0, 0);
    /// PID 控制器参数设置
    // setYawPIDPmt(0.1, 0.005, 0.01);
    setYawPIDPmt(0.025, 0.011, 0.06); // 航向
    setDepthPIDPmt(1.25, 0.006, 0.02);
    setPitchPIDPmt(0.5, 0.04, 0.15);
    setSpeedPIDPmt(0.8, 0, 0.2); // H型前进
    setXPIDPmt(0.5, 0, 0.2);
    setYPIDPmt(0.5, 0, 0.2);

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
