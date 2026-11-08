#ifndef C_SIM_3D_CTRL_H
#define C_SIM_3D_CTRL_H

#include <math.h>
#include <string>

#include "c_controller_group.h"
#include "c_fuzzy_pid.h"
#include "c_vectoredthrustcontrol.h"

using namespace std;

using namespace ControllerGroup;
using namespace VectoredThrustControl;

extern bool stage3_slow_final_descend;

namespace CombineCtrl
{

    // 常量 控制过程中的各种标志位
    const int Px_CONTROL_MODE = 1;
    const int Vx_CONTROL_MODE = 2;

    const int Py_CONTROL_MODE = 3;
    const int Vy_CONTROL_MODE = 4;

    const int Pz_CONTROL_MODE = 5;

    const int YAW_CONTROL_MODE = 6;

    const int Px_Vx_CONTROL_MODE = 7;
    const int Py_Vy_CONTROL_MODE = 8;

    const int ALL_CONTROL_MODE_SINGLE_INTEG = 9;
    const int ALL_CONTROL_MODE_DOUBLE_INTEG = 10;

    const int ADJUST_PARAMETERS_MODE = 11; // In this mode, the robot can swim forward in "H" mode
                                           // with depth control and yaw control. it is adjust control parameters underwater.

    const int MPC_MODE = 12;
    const int SHAPE_H = 1;
    const int SHAPE_H_T = 2;
    const int SHAPE_X = 3;
    const int SHAPE_G = 4;

    const int ROBOT_REGION_1 = 1; // 基于机器人坐标系下的4个区域 -45 - 0 0 - 45
    const int ROBOT_REGION_2 = 2; // 45 - 135
    const int ROBOT_REGION_3 = 3; // 135 - 180 -180 - -135
    const int ROBOT_REGION_4 = 4; // -45 - -135

    const float decrease_speed_radius = 0.8; // 减速半径
    const float DECREASE_SPEED_RADIUS = 0.8;
    const float finish_radius = 0.20; // 结束运动的邻域半径
    const float FINISH_RADIUS = 0.20;

    // 3维位置控制, 输入的是期望的位置和速度
    class C_Combine3DControl
    {
    public:
        bool is_judge_direction; // 会否已经设置好方向,每次只调节1次
        bool is_running;         // 机器人运行标志位 初始时启动机器人运行
        int shape;               // 接收形状的方向号
        int region;              // 点所在的实际区域

        // 机器人坐标系下需要控制的变量
        float robot_whole_distance;
        float robot_x_distance;
        float robot_y_distance;
        float robot_x_speed;
        float robot_y_speed;

        // 机器人坐标系下期望的速度变量
        float robot_exp_x_speed;
        float robot_exp_y_speed;

        // 期望变量
        float exp_p_x;
        float exp_v_x;

        float exp_p_y;
        float exp_v_y;

        float exp_p_z;

        float exp_yaw;
        float exp_roll;
        float exp_pitch;

        // 反馈(实时采集)量
        float cur_p_x;
        float cur_v_x;

        float cur_p_y;
        float cur_v_y;

        float cur_p_z;

        float cur_yaw;
        float cur_roll;
        float cur_pitch;

        // 控制器输出量
        float controller_p_x;
        float controller_v_x;

        float controller_p_y;
        float controller_v_y;

        float controller_p_z;

        float controller_yaw;

        // 调试指示变量
        int debug_mode;

        // adrc 控制器实例化
        C_adrc adrc_Px_ctrl, adrc_Vx_ctrl;
        C_adrc adrc_Py_ctrl, adrc_Vy_ctrl;
        C_adrc adrc_Pz_ctrl;
        C_adrc adrc_yaw_ctrl;

        // s面控制器实例化
        C_SsurfaceController_v2 ss_Px_ctrl, ss_Vx_ctrl;
        C_SsurfaceController_v2 ss_Py_ctrl, ss_Vy_ctrl;
        C_SsurfaceController_v2 ss_Pz_ctrl;
        C_SsurfaceController_v2 ss_yaw_ctrl;

        // 矢量控制实例化
        C_VectoredThrustControl vec_ctrl;

        // 成员函数声明
    public:
        C_Combine3DControl();

        // 根据点判断方向, 进而改变形状
        void judge_Direction(float _exp_yaw, float _cur_yaw); // 均为角度制
        // 根据控制模式码 施加各种各样的控制
        void adrcSelectControllerAndOutput(int _mode, float _exp_p_x, float _exp_v_x,
                                           float _exp_p_y, float _exp_v_y,
                                           float _exp_p_z, float _exp_yaw,

                                           float _cur_p_x, float _cur_v_x,
                                           float _cur_p_y, float _cur_v_y,
                                           float _cur_p_z, float _cur_yaw);
        // 速度调节
        //        void adrcSelectSpeedControllerAndOutput(int _mode,  float _exp_v_x,
        //                                                float _exp_v_y,
        //                                                float _cur_v_x,
        //                                                float _cur_v_y,
        //                                                float _cur_yaw);

        void ssSelectControllerAndOutput(int _mode, float _exp_p_x, float _exp_v_x,
                                         float _exp_p_y, float _exp_v_y,
                                         float _exp_p_z, float _exp_yaw,

                                         float _cur_p_x, float _cur_v_x,
                                         float _cur_p_y, float _cur_v_y,
                                         float _cur_p_z, float _cur_yaw);
    };

    // 3维速度控制   输入的是期望和实际的速度
    class C_Combine3DSpeedControl
    {
    public:
        int shape; // 接收形状的方向号

        // 机器人的实际速度
        float robot_x_speed;
        float robot_y_speed;
        float robot_yaw_speed;
        float robot_z_speed;
        float robot_yaw;

        // 机器人的期望速度
        float robot_exp_x_speed;
        float robot_exp_y_speed;
        float robot_exp_yaw_speed;
        float robot_exp_z_speed;
        float robot_exp_yaw;

        // 控制器输出量
        float controller_x_speed;
        float controller_y_speed;
        float controller_yaw_speed;
        float controller_z_speed;
        float controller_yaw;

        // adrc 控制器实例化
        C_adrc_speed adrc_speed_x_ctrl;
        C_adrc_speed adrc_speed_y_ctrl;
        C_adrc_speed adrc_speed_yaw_ctrl;
        C_adrc_speed adrc_speed_z_ctrl; // C_adrc_speed 速度控制
        C_adrc adrc_yaw_ctrl;           // C-adrc       位置控制

        // 矢量控制实例化
        C_VectoredThrustControl speed_vec_ctrl;

        // 成员函数声明
    public:
        C_Combine3DSpeedControl();

        // 根据形状 期望和实际速度施加控制
        void adrcSelectSpeedControllerAndOutput(int _shape, float _exp_v_x,
                                                float _exp_v_y,
                                                float _exp_v_yaw,
                                                float _exp_v_z,

                                                float _cur_v_x,
                                                float _cur_v_y,
                                                float _cur_v_yaw,
                                                float _cur_v_z);

        void adrcSelectSpeedControllerAndOutput(int _shape,
                                                float _exp_v_x, float _exp_v_y,
                                                float _exp_v_yaw, float _exp_v_z,
                                                float _exp_yaw,

                                                float _cur_v_x, float _cur_v_y,
                                                float _cur_v_yaw, float _cur_v_z,
                                                float _cur_yaw);
    };

    // hcy
    //  3维位置控制   输入的是实际的位置和期望的位置，但这个控制类不能变形
    class C_Combine3DPositionControl
    {
    public:
        int shape; // 接收形状的方向号

        // 机器人的实际位置
        float robot_x_position;
        float robot_y_position;
        float robot_yaw;
        float robot_z_position;

        // 机器人的期望位置
        float robot_exp_x_position;
        float robot_exp_y_position;
        float robot_exp_yaw;
        float robot_exp_z_position;

        // 控制器输出量
        float controller_x_position;
        float controller_y_position;
        float controller_yaw;
        float controller_z_position;

        // adrc 控制器实例化
        C_adrc_position adrc_x_ctrl;
        C_adrc_position adrc_y_ctrl;
        C_adrc_position adrc_yaw_ctrl;
        C_adrc_position adrc_z_ctrl;

        // 机器人坐标系下 目标点的坐标
        float robot_x_distance;
        float robot_y_distance;

        // 矢量控制实例化
        C_VectoredThrustControl vec_ctrl;

        // 成员函数声明
    public:
        C_Combine3DPositionControl();

        // 根据形状 期望位置 和 实际位置 进行位置控制
        void adrcSelectPositionControllerAndOutput(int _shape, float _exp_p_x,
                                                   float _exp_p_y,
                                                   float _exp_yaw,
                                                   float _exp_p_z,

                                                   float _cur_p_x,
                                                   float _cur_p_y,
                                                   float _cur_p_yaw,
                                                   float _cur_p_z);
    };

    // hcy
    //  3维位置控制   输入的是实际的位置和期望的位置，但这个控制类不能变形
    //  该控制将所有坐标统一到了旋转后的全局坐标系上
    class C_Combine3DPositionControl_v2
    {
    public:
        int shape; // 接收形状的方向号

        // 在旋转后的世界坐标系下机器人的实际位置
        float robot_x_position;
        float robot_y_position;
        float robot_yaw;
        float robot_z_position;

        // 在旋转后的世界坐标系下机器人的期望位置
        float robot_exp_x_position;
        float robot_exp_y_position;
        float robot_exp_yaw;
        float robot_exp_z_position;

        // 控制器输出量
        float controller_x_position;
        float controller_y_position;
        float controller_yaw;
        float controller_z_position;

        // adrc 控制器实例化
        C_adrc_position_v2 adrc_x_ctrl;
        C_adrc_position_v2 adrc_y_ctrl;
        C_adrc_position_v2 adrc_yaw_ctrl;
        C_adrc_position_v2 adrc_z_ctrl;

        C_adrc adrc_Px_ctrl, adrc_Vx_ctrl; // hcy
        C_adrc adrc_Py_ctrl, adrc_Vy_ctrl;
        C_adrc adrc_Pz_ctrl;
        C_adrc adrc_Pyaw_ctrl;

        // 矢量控制实例化
        C_VectoredThrustControl vec_ctrl;

        // 成员函数声明
    public:
        C_Combine3DPositionControl_v2();

        // 根据形状 期望位置 和 实际位置 进行位置控制
        void adrcSelectPositionControllerAndOutput(int _shape, float _exp_p_x,
                                                   float _exp_p_y,
                                                   float _exp_yaw,
                                                   float _exp_p_z,

                                                   float _cur_p_x,
                                                   float _cur_p_y,
                                                   float _cur_p_yaw,
                                                   float _cur_p_z);

        void pidSelectPositionControllerAndOutput(int _shape, float _exp_p_x,
                                                  float _exp_p_y,
                                                  float _exp_yaw,
                                                  float _exp_p_z,

                                                  float _cur_p_x,
                                                  float _cur_p_y,
                                                  float _cur_p_yaw,
                                                  float _cur_p_z);
        void update_and_control(int _shape,
                                float _exp_p_x, float _exp_p_y, float _exp_yaw, float _exp_p_z,
                                float _cur_p_x, float _cur_p_y, float _cur_yaw, float _cur_p_z);
        void debug_msg(ros::Publisher &pub);
    };

    // PID控制的状态基
    enum ControlDebugMode
    {
        NAV_NORMAL,     // 正常导航模式
        DEBUG_Z_ONLY,   // 只调试深度
        DEBUG_YAW_ONLY, // 只调试偏航
        DEBUG_X_ONLY,   // 只调试前进 (需要先调好Yaw)
        DEBUG_DEPYAW_ONLY
    };

    // PID
    class C_Combine3DPidControl
    {
    public:
        // 表示控制模式
        enum ControlMode
        {
            SINGLE_LOOP, // 单环控制 (位置 -> 力)
            DUAL_LOOP    // 双环/级联控制 (位置 -> 速度 -> 力)
        };

        // 构造函数
        C_Combine3DPidControl();

        void initialize_all_PIDs(); // 初始化pid参数

        // 外部设置pid参数接口
        void setHeaveInnerLoopParams(float _kp, float _ki, float _kd);
        void setHeaveOuterLoopParams(float _kp, float _ki, float _kd);

        // 用于在外部设置控制模式
        void setControlMode(ControlMode mode);
        void setFuzzyPIDEnabled(bool enabled) { fuzzy_pid_enabled_ = enabled; }
        void setManualPIDActive(bool active) { manual_pid_active_ = active; }
        void setHeaveInnerOnly(bool inner_only) { heave_inner_only_ = inner_only; }
        void setXYZPIDLogEnabled(bool enabled) { log_xyz_pid_ = enabled; }
        void resetManualPIDs();
        void resetManualPIDHistory();
        const std::string& getXYZPIDSnapshot() const { return xyz_pid_snapshot_; }
        const ros::WallTime& getXYZPIDSnapshotTime() const { return xyz_pid_snapshot_time_; }

        /**
         * @brief      单环原来用的这个
         * @details    这是主控制逻辑的调度中心。它接收目标位姿和当前位姿，
         *             根据不同的机器人形态（SHAPE_X, SHAPE_H等），调用相应的
         *             PID控制器和底层控制分配函数。
         *
         * @param[in]  shape      当前的机器人形态 (e.g., SHAPE_X, SHAPE_H).
         * @param[in]  exp_p_x    目标点的世界坐标X.
         * @param[in]  exp_p_y    目标点的世界坐标Y.
         * @param[in]  exp_yaw    目标姿态的偏航角 (单位: 度).
         * @param[in]  exp_p_z    目标深度Z.
         * @param[in]  cur_p_x    当前点的世界坐标X.
         * @param[in]  cur_p_y    当前点的世界坐标Y.
         * @param[in]  cur_yaw    当前姿态的偏航角 (单位: 度).
         * @param[in]  cur_p_z    当前深度Z.
         *
         * @note       此函数内部不返回值，而是通过调用底层的 `space3DControl`
         *             函数来间接驱动执行器。
         */
        void updateControl(int _shape,
                           float _exp_p_x, float _exp_p_y, float _exp_yaw, float _exp_p_z,
                           float _cur_p_x, float _cur_p_y, float _cur_yaw, float _cur_p_z);

        // 双环多了些参数
        void updateControl(
            int shape,
            float exp_p_x, float exp_p_y, float exp_yaw, float exp_p_z,
            float exp_v_x, float exp_v_y, float exp_v_z,
            float cur_p_x, float cur_p_y, float cur_yaw, float cur_p_z,
            float cur_v_x, float cur_v_y, float cur_v_z);

        /**
         * @brief      发布机器人当前和期望位姿，调式用
         *
         * @note       在主函数中调用
         */
        void debug_msg(ros::Publisher &pub);
        /**
         * @brief 设置控制器的调试模式.
         * @param mode 要设置的调试模式 (NAV_NORMAL, DEBUG_Z_ONLY, etc.).
         * @param debug_target 如果是调试模式，需要提供一个固定的目标值.
         *                     - 对于 DEBUG_Z_ONLY, 这是目标深度.
         *                     - 对于 DEBUG_YAW_ONLY, 这是目标航向.
         */
        void setDebugMode(ControlDebugMode mode, float debug_target = 0.0f);

        // 矢量控制实例化
        C_VectoredThrustControl vec_ctrl;

    private:
        bool fuzzy_pid_enabled_ = false;
        bool log_xyz_pid_ = false;
        ControllerGroup::FuzzyPID fuzzy_x_, fuzzy_y_, fuzzy_z_;
        ControllerGroup::FuzzyPID fuzzy_yaw_, fuzzy_forward_, fuzzy_heave_position_, fuzzy_heave_velocity_;
        bool manual_pid_active_ = false;
        bool heave_inner_only_ = false;
        float calculateManualPID(ControllerGroup::PID &base, ControllerGroup::FuzzyPID &fuzzy,
                                 float error, const char *axis, bool depth = false);
        float calculateHeave(bool inner_only);
        std::string xyz_pid_snapshot_;
        ros::WallTime xyz_pid_snapshot_time_;
        // 存储当前控制模式
        ControlMode current_mode_;

        // --- 新增：用于存储调试模式和目标的私有成员变量 ---
        ControlDebugMode current_debug_mode_;
        float debug_target_value_;
        /**
         * @brief      内部控制逻辑函数
         * @details    这是内部控制，
         *             根据不同的机器人形态（SHAPE_X, SHAPE_H等），通过PID计算相应的误差
         *             调用space3DControl解算
         */
        void updatePidControl();

        // --- 新增的私有成员函数，用于重构 ---
        void handleShapeH_DualLoop_Control();
        void handleShapeX_DualLoop_Control();

        void handleShapeX_SingleLoop_Control();
        void handleShapeH_SingleLoop_Control();
        void printDebugInfo();

        // 共享的数据
        float cur_p_x_, cur_p_y_, cur_p_z_, cur_yaw_;
        float cur_v_x_, cur_v_y_, cur_v_z_;
        float exp_p_x_, exp_p_y_, exp_p_z_, exp_yaw_;
        float exp_v_x_, exp_v_y_, exp_v_z_;
        float filtered_cur_v_z_;
        int shape_;
    };
}

#endif // C_SIM_3D_CTRL_H
