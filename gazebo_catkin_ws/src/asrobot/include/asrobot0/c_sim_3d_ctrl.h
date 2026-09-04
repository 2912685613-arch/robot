#ifndef C_SIM_3D_CTRL_H
#define C_SIM_3D_CTRL_H

#include <math.h>

#include "c_controller_group.h"
#include "c_vectoredthrustcontrol.h"


using namespace std;

using namespace ControllerGroup;
using namespace VectoredThrustControl;

extern bool stage3_slow_final_descend;

namespace CombineCtrl{

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

    const int ADJUST_PARAMETERS_MODE = 11;    // In this mode, the robot can swim forward in "H" mode
                                            // with depth control and yaw control. it is adjust control parameters underwater.

    const int MPC_MODE = 12;
    const int SHAPE_H = 1;
    const int SHAPE_H_T = 2;
    const int SHAPE_X = 3;

    const int ROBOT_REGION_1 = 1;      // 基于机器人坐标系下的4个区域 -45 - 0 0 - 45
    const int ROBOT_REGION_2 = 2;      // 45 - 135
    const int ROBOT_REGION_3 = 3;      // 135 - 180 -180 - -135
    const int ROBOT_REGION_4 = 4;      // -45 - -135

    const float decrease_speed_radius = 0.8; // 减速半径
    const float DECREASE_SPEED_RADIUS = 0.8;
    const float finish_radius = 0.20;  // 结束运动的邻域半径
    const float FINISH_RADIUS = 0.20;


    // 3维位置控制, 输入的是期望的位置和速度
    class C_Combine3DControl
    {
    public:
        bool is_judge_direction; // 会否已经设置好方向,每次只调节1次
        bool is_running;         // 机器人运行标志位 初始时启动机器人运行
        int shape;  // 接收形状的方向号
        int region; // 点所在的实际区域

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
        void judge_Direction(float _exp_yaw, float _cur_yaw ); // 均为角度制
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
        int shape;               // 接收形状的方向号

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
        C_adrc_speed adrc_speed_z_ctrl;    // C_adrc_speed 速度控制
        C_adrc adrc_yaw_ctrl;              // C-adrc       位置控制

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


    //hcy
    // 3维位置控制   输入的是实际的位置和期望的位置，但这个控制类不能变形
    class C_Combine3DPositionControl
    {
    public:
        int shape;               // 接收形状的方向号

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


    //hcy
    // 3维位置控制   输入的是实际的位置和期望的位置，但这个控制类不能变形
    // 该控制将所有坐标统一到了旋转后的全局坐标系上
    class C_Combine3DPositionControl_v2
    {
    public:
        int shape;               // 接收形状的方向号

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
        
        C_adrc adrc_Px_ctrl, adrc_Vx_ctrl;//hcy
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


    };


}



#endif // C_SIM_3D_CTRL_H
