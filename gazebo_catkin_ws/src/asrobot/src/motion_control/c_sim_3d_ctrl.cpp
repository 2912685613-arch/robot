#include "../../include/asrobot/c_sim_3d_ctrl.h"
#include "asrobot/mc.h"

// #define pi 3.1415926535897932384626

namespace CombineCtrl
{
    /* !
     *  组合三维控制函数实现
     */

    /* @
     * 函数功能: 对组合控制器内部的各种变量进行初始化
     *
     * 输入: 无
     *
     * 输出: 无
     *
     */
    C_Combine3DControl::C_Combine3DControl()
    {
        is_judge_direction = false; // 会否已经设置好方向,每次只调节1次
                                    //         is_set_exp_yaw = false;/************************************************************************/
        is_running = true;          // 机器人运行标志位 初始时启动机器人运行

        shape = 0;  // 腿部变形表示
        region = 0; // 目标点所在的实际区域

        // 机器人坐标系下 期望变量
        robot_whole_distance = 0; // 斜向距离
        robot_x_distance = 0;     // 机器人坐标系下,目标点的x值
        robot_y_distance = 0;     // 机器人坐标系下,目标点的y值
        robot_x_speed = 0;        // 机器人坐标系下, x方向的速度
        robot_y_speed = 0;        // 机器人坐标系下, y方向的速度

        // 期望变量 这些量在全局坐标系下
        exp_p_x = 0; // 全局坐标系下 目标的x坐标
        exp_v_x = 0; // 全局坐标系下 期望的x方向速度 (这个量一般不直接给)

        exp_p_y = 0; // 全局坐标系下 目标的y坐标
        exp_v_y = 0; // 全局坐标系下 期望的y方向速度 (这个量一般不直接给)

        exp_p_z = 0; // 全局坐标系下 目标的z坐标

        exp_yaw = 0; // 期望航线角度
        exp_roll = 0;
        exp_pitch = 0;

        // 反馈(实时采集)量
        cur_p_x = 0; // 全局坐标系下 当前的x坐标
        cur_v_x = 0; // 全局坐标系下 机器人x方向前进速度

        cur_p_y = 0; // 全局坐标系下 当前的y坐标
        cur_v_y = 0; // 全局坐标系下 机器人y方向前进速度

        cur_p_z = 0; // 全局坐标系下 当前的z坐标

        cur_yaw = 0; // 当前航向角
        cur_roll = 0;
        cur_pitch = 0;

        // 控制器输出量
        controller_p_x = 0; // 机器人坐标系下 沿着x方向位置控制量
        controller_v_x = 0; // 机器人坐标系下 沿着x方向速度控制量

        controller_p_y = 0; // 机器人坐标系下 沿着y方向位置控制量
        controller_v_y = 0; // 机器人坐标系下 沿着y方向速度控制量

        controller_p_z = 0; // 机器人坐标系下 沿着z方向的位置控制量

        controller_yaw = 0;

        // 调试指示变量
        debug_mode = 0; // 调试模式指示变量
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /* @
     *
     * 功能：　判断区域，调节腿部形状
     *
     * 输入：　期望的航向角，　当前ＩＭＵ输出的航向角
     *
     * 输出：　无
     *
     */
    // 判断方向 4个区域
    // REGION_315_45
    // REGION_45_135
    // REGION_135_225
    // REGION_225_315
    void C_Combine3DControl::judge_Direction(float _exp_yaw, float _cur_yaw) // 均为角度制
    {
        if (is_judge_direction == false)
        {
            // 航向角在 0 - 45 上

            if ((_cur_yaw > 0) && (_cur_yaw < 45))
            {
                if ((_exp_yaw > (-45 + _cur_yaw)) && (_exp_yaw < (45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_1;
                }
                else if ((_exp_yaw > (45 + _cur_yaw)) && (_exp_yaw < (135 + _cur_yaw)))
                {
                    region = ROBOT_REGION_2;
                }
                else if (((_exp_yaw > (135 + _cur_yaw)) && (_exp_yaw < 180)) ||
                         ((_exp_yaw > -180) && (_exp_yaw < (-135 + _cur_yaw))))
                {
                    region = ROBOT_REGION_3;
                }
                else if ((_exp_yaw > (-135 + cur_yaw)) && (_exp_yaw < (-45 + cur_yaw)))
                {
                    region = ROBOT_REGION_4;
                }
            }
            else if ((_cur_yaw > 45) && (_cur_yaw < 135))
            {
                if ((_exp_yaw > (-45 + _cur_yaw)) && (_exp_yaw > (45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_1;
                }
                else if (((_exp_yaw > (45 + _cur_yaw)) && (_exp_yaw < 180)) ||
                         ((_exp_yaw > -180) && (_exp_yaw < (135 + _cur_yaw - 360))))
                {
                    region = ROBOT_REGION_2;
                }
                else if ((_exp_yaw > (135 + _cur_yaw - 360)) && (_exp_yaw < (-135 + _cur_yaw)))
                {
                    region = ROBOT_REGION_3;
                }
                else if ((_exp_yaw > (-135 + _cur_yaw)) && (_exp_yaw < (-45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_4;
                }
            }
            else if ((_cur_yaw > 135) && (_cur_yaw < 180))
            {
                if (((_exp_yaw > (-45 + _cur_yaw)) && (_exp_yaw < 180)) ||
                    ((_exp_yaw > -180) && (_exp_yaw < (45 + _cur_yaw - 360))))
                {
                    region = ROBOT_REGION_1;
                }
                else if ((_exp_yaw > (45 + _cur_yaw - 360)) && (_exp_yaw < (135 + _cur_yaw - 360)))
                {
                    region = ROBOT_REGION_2;
                }
                else if ((_exp_yaw > (135 + _cur_yaw - 360)) && (_exp_yaw < (-135 + _cur_yaw)))
                {
                    region = ROBOT_REGION_3;
                }
                else if ((_exp_yaw > (-135 + _cur_yaw)) && (_exp_yaw < (-45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_4;
                }
            }
            else if ((_cur_yaw > -45) && (_cur_yaw < 0))
            {
                if ((_exp_yaw > (-45 + _cur_yaw)) && (_exp_yaw < (45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_1;
                }
                else if ((_exp_yaw > (45 + _cur_yaw)) && (_exp_yaw < (135 + _cur_yaw)))
                {
                    region = ROBOT_REGION_2;
                }
                else if (((_exp_yaw > (135 + _cur_yaw)) && (_exp_yaw < 180)) ||
                         ((_exp_yaw > -180) && (_exp_yaw < (-135 + _cur_yaw))))
                {
                    region = ROBOT_REGION_3;
                }
                else if ((_exp_yaw > (-135 + _cur_yaw)) && (_exp_yaw < (-45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_4;
                }
            }
            else if ((_cur_yaw > -135) && (_cur_yaw < -45))
            {
                if ((_exp_yaw > (-45 + _cur_yaw)) && (_exp_yaw < (45 + _cur_yaw)))
                {
                    region = ROBOT_REGION_1;
                }
                else if (((_exp_yaw) > (45 + _cur_yaw)) && (_exp_yaw < (135 + _cur_yaw)))
                {
                    region = ROBOT_REGION_2;
                }
                else if ((_exp_yaw > (135 + _cur_yaw)) && (_exp_yaw < (-135 + _cur_yaw + 360)))
                {
                    region = ROBOT_REGION_3;
                }
                else if (((_exp_yaw > (-135 + _cur_yaw + 360)) && (_exp_yaw < 180)) ||
                         ((_exp_yaw > -180) && (_exp_yaw < (-45 + _cur_yaw))))
                {
                    region = ROBOT_REGION_4;
                }
            }
            else if ((_cur_yaw > -180) && (_cur_yaw < -135))
            {
                if (((_exp_yaw > (-45 + _cur_yaw + 360)) && (_exp_yaw < 180)) ||
                    ((_exp_yaw > -180) && (_exp_yaw < (45 + _cur_yaw))))
                {
                    region = ROBOT_REGION_1;
                }
                else if ((_exp_yaw > (45 + _cur_yaw)) && (_exp_yaw < (135 + _cur_yaw)))
                {
                    region = ROBOT_REGION_2;
                }
                else if ((_exp_yaw > (135 + _cur_yaw)) && (_exp_yaw < (-135 + _cur_yaw + 360)))
                {
                    region = ROBOT_REGION_3;
                }
                else if ((_exp_yaw > (-135 + _cur_yaw + 360)) && (_exp_yaw < (-45 + _cur_yaw + 360)))
                {
                    region = ROBOT_REGION_4;
                }
            }

            // 调整形状
            if ((region == ROBOT_REGION_1) || (region == ROBOT_REGION_3))
            {
                shape = SHAPE_H;
            }
            else
            {
                shape = SHAPE_H_T;
            }

            is_running = true; // 打开机器人运行开关

            is_judge_direction = true; // 表示已经判断过标志位了
        }
        cout << "region : " << region << endl;
        cout << "shape :" << shape << endl;
        cout << "_exp_yaw:  " << _exp_yaw << endl;
        cout << "_cur_yaw:  " << _cur_yaw << endl;
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    // 选择对应控制器进行调节
    /* @
     *
     * 功能：　adrc控制器，带入任意控制码进行调节
     *
     * 输入：　控制模式码，　期望的位置姿态，　实际的位置姿态
     *
     * 输出：　无
     *
     */
    // hcy:gazebo原程序p使用的
    void C_Combine3DControl::adrcSelectControllerAndOutput(int _mode, float _exp_p_x, float _exp_v_x,
                                                           float _exp_p_y, float _exp_v_y,
                                                           float _exp_p_z, float _exp_yaw,

                                                           float _cur_p_x, float _cur_v_x,
                                                           float _cur_p_y, float _cur_v_y,
                                                           float _cur_p_z, float _cur_yaw)

    {
        float cur_yaw_control = _cur_yaw;
        //         float delta_yaw = _exp_yaw - _cur_yaw;
        //         if ( delta_yaw > 180 )
        //             _cur_yaw += 360;
        //         else if( delta_yaw <= -180 )
        //              _cur_yaw += -360;

        //         if (delta_yaw > 180)
        //             _cur_yaw += -360;
        //         else if(delta_yaw <= -180)
        //              _cur_yaw += 360;

        switch (_mode)
        {
        case 0:
        {
            cout << " now it has stop !!! " << endl;
        };
        break;
        case Px_CONTROL_MODE: // 控x方向位置
        {
            robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180 * pi) * (_exp_p_y - _cur_p_y);
            controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
            vec_ctrl.motorforce(controller_p_x, 0, 0, 0, cur_roll, _cur_yaw, cur_pitch);
            vec_ctrl.PxControlWithPxForce(controller_p_x);

            cout << " controller_p_x:  " << controller_p_x << endl;
            cout << "robot_x_distance" << robot_x_distance << endl;
            cout << "_cur_yaw" << _cur_yaw << endl;
        };
        break;
        case Vx_CONTROL_MODE: // 控x方向速度
        {
            controller_v_x = adrc_Vx_ctrl.adrcOutput(_exp_v_x, _cur_v_x);
            vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.VxControlWithVxForce(controller_v_x);
            cout << " controller_v_x:  " << controller_v_x << endl;
        };
        break;

        case Py_CONTROL_MODE: // 控y方向位置
        {

            controller_p_y = adrc_Py_ctrl.adrcOutput(_exp_p_y, _cur_p_y);
            vec_ctrl.motorforce(0, controller_p_y, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.PyControlWithPyForce(controller_p_y);
            cout << " controller_p_y:  " << controller_p_y << endl;
        };
        break;
        case Vy_CONTROL_MODE: // 控y方向速度
        {
            controller_v_y = adrc_Vy_ctrl.adrcOutput(_exp_v_y, _cur_v_y);
            vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.VyControlWithVyForce(controller_v_y);
            cout << " controller_v_y:  " << controller_v_y << endl;
        };
        break;

        case Pz_CONTROL_MODE: // 控z方向位置  0.7 为抵消浮力的参考量
        {
            controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2;
            vec_ctrl.motorforce(0, 0, controller_p_z, 0, 0, _cur_yaw, 0);
            vec_ctrl.depthControlWithPzForce(controller_p_z);
            cout << " controller_p_z:  " << controller_p_z << endl;
        };
        break;
        case YAW_CONTROL_MODE: // 控航向角
        {
            controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw);
            vec_ctrl.motorforce(0, 0, 0, controller_yaw, 0, _cur_yaw, 0);
            vec_ctrl.yawControlWithYawForce(controller_yaw);
            cout << " controller_yaw:  " << controller_yaw << endl;
        };
        break;
        case Px_Vx_CONTROL_MODE: // 双环控制x
        {
            robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
            robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);

            robot_x_speed = cos(_cur_yaw / 180.0 * pi) * _cur_v_x + sin(_cur_yaw / 180.0 * pi) * _cur_v_y;
            robot_y_speed = (-sin(_cur_yaw / 180.0 * pi)) * _cur_v_x + cos(_cur_yaw / 180.0 * pi) * _cur_v_y;

            controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
            controller_v_x = adrc_Vx_ctrl.adrcOutput(controller_p_x, robot_x_speed);
            vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.doubleClosedLoopControlPx(controller_v_x);
            cout << " controller_p_x:  " << controller_p_x << endl;
            cout << " controller_v_x:  " << controller_v_x << endl;
        };
        break;
        case Py_Vy_CONTROL_MODE: // 双环控制y
        {
            controller_p_y = adrc_Py_ctrl.adrcOutput(_exp_p_y, _cur_p_y);
            controller_v_y = adrc_Vx_ctrl.adrcOutput(controller_p_y, _cur_v_y);
            vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.doubleClosedLoopControlPy(controller_v_y);
            cout << " controller_p_y:  " << controller_p_y << endl;
            cout << " controller_v_y:  " << controller_v_y << endl;
        };
        break;

        case ALL_CONTROL_MODE_SINGLE_INTEG: // 环全方位控制
        {
            // 根据实际情况切换形状 /////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) > DECREASE_SPEED_RADIUS)
            {
                shape = shape; // 以前是哪种形状,现在还用哪种形状
            }
            else
            {
                shape = SHAPE_X; // 转换为x形状
            }

            // 根据形状施加控制 //////////////////////////////////
            if (shape == SHAPE_H_T) // y方向加控制量 位置单环控制
            {
                // 根据区域重新分配航向角,考虑到 2号区域和4号区域需要转向的问题
                _exp_yaw -= 90; // 跟正常航向角差90度
                if (region == ROBOT_REGION_2)
                {
                    _exp_yaw = _exp_yaw; // 2号区域不用动
                }
                else if (region == ROBOT_REGION_4)
                {
                    _exp_yaw += 180; // 四号区域 加180 加到2号区域那边
                }
                if (_exp_yaw > 180) // 限制在-180 到 180内
                    _exp_yaw -= 360;
                else if (_exp_yaw < -180)
                    _exp_yaw += 360;

                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                controller_p_x = 0;                                            // x方向没有控制量
                controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0); // 斜线位移

                // 斜线速度
                /// DEBUG
                /// controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                controller_p_z = 0;
                controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw); // 航向角调节

                cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

                // 解算前的方法
                // vec_ctrl.yawLocomMode_LR();
                // vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);// 质点控制法
                // 解算后的方法
                /// DEBUG 航向角权重设置为5
                vec_ctrl.space3DControl(SHAPE_H_T, controller_p_x, controller_p_y, controller_p_z, 5 * controller_yaw);
                /// vec_ctrl.space3DControl(SHAPE_H_T, 0, 0, 0, controller_yaw);
                ///
            }
            else if (shape == SHAPE_H) // x方向加控制量 位置单环控制
            {

                if (region == ROBOT_REGION_1)
                    _exp_yaw = _exp_yaw; // 如果在1号区域,不用动,因为机器人的头就在这个区域
                else if (region == ROBOT_REGION_3)
                {
                    _exp_yaw += 180; // 加到1号区域那边,跟1号区域共用期望角度
                }
                if (_exp_yaw > 180) // 加过辙了 往下扣360
                    _exp_yaw -= 360;
                else if (_exp_yaw < -180)
                    _exp_yaw += 360;
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                // 双环x控制量
                controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                // 斜线速度
                controller_p_y = 0;
                /// DEBUG
                /// controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                controller_p_z = 0;
                ///
                controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw); // 航向角调节

                cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
                // 解算前的方法
                // vec_ctrl.yawLocomMode_FB();
                // vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                // 解算后的方法
                /// DEBUG  航向角权重设置为5
                vec_ctrl.space3DControl(SHAPE_H, controller_p_x, controller_p_y, controller_p_z, 5 * controller_yaw);
                // vec_ctrl.space3DControl(SHAPE_H, 0, 0, 0, controller_yaw);
                ///
            }

            else if (shape == SHAPE_X) // 位置单环控制
            {
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                // x y 都施加双环控制
                controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);

                controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0);
                /// DEBUG
                /// controller_p_x = adrc_Px_ctrl.adrcOutput(1, 0);
                /// controller_p_y = adrc_Py_ctrl.adrcOutput(1, 0);
                ///
                // z正常控制
                // controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2; // 控制深度
                /// DEBUG
                controller_p_z = 0;
                ///
                controller_yaw = 0; // x型模式无法调节航向角

                cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;
                // 解算前的方法
                // vec_ctrl.yawLocomMode_X();
                // vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                // 解算后的方法
                vec_ctrl.space3DControl(SHAPE_X, controller_p_x, controller_p_y, controller_p_z, controller_yaw);
            }

            // 判断终止条件 /////////////////////////////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) < finish_radius)
            {
                vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0); // 全关
                is_judge_direction = false;
                is_running = false;
                debug_mode = 0; // 强制让调试模式为0 终止运行
                shape = 0;      // 强制将形状取消, 不接受控制
            }
            cout << " controller_p_x:  " << controller_p_x << endl;
            cout << " controller_p_y:  " << controller_p_y << endl;
            cout << " controller_p_z:  " << controller_p_z << endl;
            cout << " controller_yaw:  " << controller_yaw << endl;
            cout << "robot_x_distance  " << robot_x_distance << endl;
            cout << "robot_y_distance  " << robot_y_distance << endl;
            cout << "exp_p_x:  " << _exp_p_x << "  " << "cur_p_x:  " << _cur_p_x << endl;
            cout << "exp_p_y:  " << _exp_p_y << "  " << "cur_p_y:  " << _cur_p_y << endl;
            cout << "exp_yaw:  " << _exp_yaw << "  " << "cur_yaw:  " << _cur_yaw << endl;
        };
        break;

        case ALL_CONTROL_MODE_DOUBLE_INTEG: // 双环全方位控制
        {
            // 根据实际情况切换形状 /////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) > DECREASE_SPEED_RADIUS)
            {
                shape = shape; // 以前是哪种形状,现在还用哪种形状
            }
            else
            {
                shape = SHAPE_X; // 转换为x形状
            }

            // 根据形状施加控制 //////////////////////////////////
            if (shape == SHAPE_H_T) // y方向加控制量 双环
            {
                // 根据区域重新分配航向角,考虑到 2号区域和4号区域需要转向的问题
                _exp_yaw -= 90; // 跟正常航向角差90度
                if (region == ROBOT_REGION_2)
                {
                    _exp_yaw = _exp_yaw; // 2号区域不用动
                }
                else if (region == ROBOT_REGION_4)
                {
                    _exp_yaw += 180; // 四号区域 加180 加到2号区域那边
                }
                if (_exp_yaw > 180) // 限制在-180 到 180内
                    _exp_yaw -= 360;
                else if (_exp_yaw < -180)
                    _exp_yaw += 360;

                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_y_distance = -sin(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                robot_y_speed = (-sin(_cur_yaw / 180.0 * pi)) * _cur_v_x + cos(_cur_yaw / 180.0 * pi) * _cur_v_y;

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                controller_v_x = 0;                                            // x方向没有控制量
                controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0); // 斜线位移

                // 斜线速度
                controller_v_y = adrc_Vy_ctrl.adrcOutput(controller_p_y, robot_y_speed);

                controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2; // 控制深度
                controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw);      // 航向角调节

                cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;
                // 解算前的方法
                // vec_ctrl.yawLocomMode_LR();
                // vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);// 质点控制法
                // 解算后的方法
                vec_ctrl.space3DControl(SHAPE_H_T, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
            }
            else if (shape == SHAPE_H) // x方向加控制量 双环
            {
                if (region == ROBOT_REGION_1)
                    _exp_yaw = _exp_yaw; // 如果在1号区域,不用动,因为机器人的头就在这个区域
                else if (region == ROBOT_REGION_3)
                {
                    _exp_yaw += 180; // 加到1号区域那边,跟1号区域共用期望角度
                }
                if (_exp_yaw > 180) // 加过辙了 往下扣360
                    _exp_yaw -= 360;
                else if (_exp_yaw < -180)
                    _exp_yaw += 360;
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                robot_x_speed = cos(_cur_yaw / 180.0 * pi) * _cur_v_x + sin(_cur_yaw / 180.0 * pi) * _cur_v_y;

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                // 双环x控制量
                controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                controller_v_x = adrc_Vx_ctrl.adrcOutput(controller_p_x, robot_x_speed);
                // 斜线速度
                controller_v_y = 0;

                controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2; // 控制深度
                controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw);      // 航向角调节

                cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
                // 解算前的方法
                // vec_ctrl.yawLocomMode_FB();
                // vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                // 解算后的方法
                vec_ctrl.space3DControl(SHAPE_H, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
            }
            else if (shape == SHAPE_X)
            {
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                robot_x_speed = cos(_cur_yaw / 180.0 * pi) * _cur_v_x + sin(_cur_yaw / 180.0 * pi) * _cur_v_y;
                robot_y_speed = (-sin(_cur_yaw / 180.0 * pi)) * _cur_v_x + cos(_cur_yaw / 180.0 * pi) * _cur_v_y;
                // x y 都施加双环控制
                controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                controller_v_x = adrc_Vx_ctrl.adrcOutput(controller_p_x, robot_x_speed);

                controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0);
                controller_v_y = adrc_Vy_ctrl.adrcOutput(controller_p_y, robot_y_speed);

                // z正常控制
                controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2; // 控制深度
                controller_yaw = 0;                                                 // x型模式无法调节航向角

                cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;
                // 解算前的方法
                // vec_ctrl.yawLocomMode_X();
                // vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                // 解算后的方法
                vec_ctrl.space3DControl(SHAPE_X, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
            }

            // 判断终止条件 /////////////////////////////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) <= finish_radius)
            {
                // vec_ctrl.motorforce(0,0,0,0,0,0,0);
                vec_ctrl.thrusterforce2speed(0, 0, 0, 0); // 全关

                is_judge_direction = false;
                is_running = false;
                debug_mode = 0; // 强制让调试模式为0 终止运行
                shape = 0;      // 强制将形状取消, 不接受控制
            }
            cout << "****double_integ####" << endl;
            cout << " controller_v_x:  " << controller_v_x << endl;
            cout << " controller_v_y:  " << controller_v_y << endl;
            cout << " controller_p_z:  " << controller_p_z << endl;
            cout << " controller_yaw:  " << controller_yaw << endl;
            cout << "robot_x_distance  " << robot_x_distance << endl;
            cout << "robot_y_distance  " << robot_y_distance << endl;
            cout << "exp_p_x:  " << _exp_p_x << "  " << "cur_p_x:  " << _cur_p_x << endl;
            cout << "exp_p_y:  " << _exp_p_y << "  " << "cur_p_y:  " << _cur_p_y << endl;
            cout << "exp_yaw:  " << _exp_yaw << "  " << "cur_yaw:  " << _cur_yaw << endl;
        };
        break;
        case ADJUST_PARAMETERS_MODE:
        {
            //                 if(-90 <= _exp_yaw <= 90) {}
            //                 else if (-180 <= _exp_yaw < -90) {
            //                     _exp_yaw += 180;
            //                     _exp_p_x = -_exp_p_x;
            //                 } else if (90 < _exp_yaw <=180) {
            //                     _exp_yaw -= 180;
            //                     _exp_p_x = -_exp_p_x;
            //                 } else {
            //                     cout<<"Yaw Input Error!!"<<endl;
            //                 }

            // +-180 boundary process
            float delta_yaw = _exp_yaw - cur_yaw_control;
            if (delta_yaw > 180)
                cur_yaw_control += 360;
            else if (delta_yaw <= -180)
                cur_yaw_control += -360;

            controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2;   // 控制深度
            controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, cur_yaw_control); // 航向角调节

            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
            cout << "  controller_yaw " << controller_yaw << endl;
            vec_ctrl.yawLocomMode_FB();
            vec_ctrl.motorforce(_exp_p_x, 0, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
            cout << " exp_depth:  " << _exp_p_z << ", _cur_depth:  " << _cur_p_z << endl;
            cout << " exp_yaw:  " << _exp_yaw << ", _cur_yaw:  " << _cur_yaw << ", cur_yaw_control: " << cur_yaw_control << endl;
            cout << " Detpth controller output:  " << controller_p_z << endl;
            cout << " Yaw controller output:  " << controller_yaw << endl;
        };
        break;
        default:
        {
            cout << "there is no this control mode !!! " << endl;
        };
        break;
        }
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    // 选择对应控制器进行调节
    /* @
     *
     * 功能：　ｓ面控制器，带入任意控制码进行调节
     *
     * 输入：　控制模式码，　期望的位置姿态，　实际的位置姿态
     *
     * 输出：　无
     *
     */
    void C_Combine3DControl::ssSelectControllerAndOutput(int _mode, float _exp_p_x, float _exp_v_x,
                                                         float _exp_p_y, float _exp_v_y,
                                                         float _exp_p_z, float _exp_yaw,

                                                         float _cur_p_x, float _cur_v_x,
                                                         float _cur_p_y, float _cur_v_y,
                                                         float _cur_p_z, float _cur_yaw)
    {
        switch (_mode)
        {
        case 0:
        {
            cout << " now it has stop !!! " << endl;
        };
        break;
        case Px_CONTROL_MODE: // 控x方向位置
        {
            robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180 * pi) * (_exp_p_y - _cur_p_y);
            controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
            vec_ctrl.motorforce(controller_p_x, 0, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.PxControlWithPxForce(controller_p_x);
            cout << " controller_p_x:  " << controller_p_x << endl;
            cout << "robot_x_distance" << robot_x_distance << endl;
            cout << "_cur_yaw" << _cur_yaw << endl;
        };
        break;
        case Vx_CONTROL_MODE: // 控x方向速度
        {
            controller_v_x = ss_Vx_ctrl.CalSSCv2output(_exp_v_x, _cur_v_x);
            vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.VxControlWithVxForce(controller_v_x);
            cout << " controller_v_x:  " << controller_v_x << endl;
        };
        break;

        case Py_CONTROL_MODE: // 控y方向位置
        {

            controller_p_y = ss_Py_ctrl.CalSSCv2output(_exp_p_y, _cur_p_y);
            vec_ctrl.motorforce(0, controller_p_y, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.PyControlWithPyForce(controller_p_y);
            cout << " controller_p_y:  " << controller_p_y << endl;
        };
        break;
        case Vy_CONTROL_MODE: // 控y方向速度
        {
            controller_v_y = ss_Vy_ctrl.CalSSCv2output(_exp_v_y, _cur_v_y);
            vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.VyControlWithVyForce(controller_v_y);
            cout << " controller_v_y:  " << controller_v_y << endl;
        };
        break;

        case Pz_CONTROL_MODE: // 控z方向位置  0.7 为抵消浮力的参考量
        {
            controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z) - 0.7;
            vec_ctrl.motorforce(0, 0, controller_p_z, 0, 0, _cur_yaw, 0);
            vec_ctrl.depthControlWithPzForce(controller_p_z);
            cout << " controller_p_z:  " << controller_p_z << endl;
        };
        break;
        case YAW_CONTROL_MODE: // 控航向角
        {
            controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw);
            vec_ctrl.motorforce(0, 0, 0, controller_yaw, 0, _cur_yaw, 0);
            vec_ctrl.yawControlWithYawForce(controller_yaw);
            cout << " controller_yaw:  " << controller_yaw << endl;
        };
        break;
        case Px_Vx_CONTROL_MODE: // 双环控制x
        {
            robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
            robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);

            robot_x_speed = cos(_cur_yaw / 180.0 * pi) * _cur_v_x + sin(_cur_yaw / 180.0 * pi) * _cur_v_y;
            robot_y_speed = (-sin(_cur_yaw / 180.0 * pi)) * _cur_v_x + cos(_cur_yaw / 180.0 * pi) * _cur_v_y;

            controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
            controller_v_x = ss_Vx_ctrl.CalSSCv2output(controller_p_x, robot_x_speed);
            vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.doubleClosedLoopControlPx(controller_v_x);
            cout << " controller_p_x:  " << controller_p_x << endl;
            cout << " controller_v_x:  " << controller_v_x << endl;
        };
        break;
        case Py_Vy_CONTROL_MODE: // 双环控制y
        {
            controller_p_y = ss_Py_ctrl.CalSSCv2output(_exp_p_y, _cur_p_y);
            controller_v_y = ss_Vx_ctrl.CalSSCv2output(controller_p_y, _cur_v_y);
            vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
            vec_ctrl.doubleClosedLoopControlPy(controller_v_y);
            cout << " controller_p_y:  " << controller_p_y << endl;
            cout << " controller_v_y:  " << controller_v_y << endl;
        };
        break;

        case ALL_CONTROL_MODE_SINGLE_INTEG: // 环全方位控制
        {
            // 根据实际情况切换形状 /////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) > DECREASE_SPEED_RADIUS)
            {
                shape = shape; // 以前是哪种形状,现在还用哪种形状
            }
            else
            {
                shape = SHAPE_X; // 转换为x形状
            }

            // 根据形状施加控制 //////////////////////////////////
            if (shape == SHAPE_H_T) // y方向加控制量 位置单环控制
            {
                _exp_yaw -= 90;
                if (_exp_yaw < -180)
                {
                    _exp_yaw += 360;
                }
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                controller_p_x = 0;                                              // x方向没有控制量
                controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0); // 斜线位移

                // 斜线速度
                controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z);  // 控制深度
                controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

                vec_ctrl.yawLocomMode_LR();
                vec_ctrl.motorforce(controller_p_x, controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                vec_ctrl.space3DControl(SHAPE_H_T, controller_p_x, controller_p_y, controller_p_z, controller_yaw);
            }
            else if (shape == SHAPE_H) // x方向加控制量 位置单环控制
            {
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                // 双环x控制量
                controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                // 斜线速度
                controller_p_y = 0;

                controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z);  // 控制深度
                controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

                vec_ctrl.yawLocomMode_FB();
                vec_ctrl.motorforce(controller_p_x, controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                vec_ctrl.space3DControl(SHAPE_H, controller_p_x, controller_p_y, controller_p_z, controller_yaw);
            }
            else if (shape == SHAPE_X) // 位置单环控制
            {
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                // x y 都施加双环控制
                controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);

                controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0);

                // z正常控制
                controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z); // 控制深度
                controller_yaw = 0;                                             // x型模式无法调节航向角

                cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

                vec_ctrl.yawLocomMode_X();
                vec_ctrl.motorforce(controller_p_x, controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                vec_ctrl.space3DControl(SHAPE_X, controller_p_x, controller_p_y, controller_p_z, controller_yaw);
            }

            // 判断终止条件 /////////////////////////////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) < finish_radius)
            {
                vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0); // 全关
                is_judge_direction = false;
                is_running = false;
                debug_mode = 0; // 强制让调试模式为0 终止运行
                shape = 0;      // 强制将形状取消, 不接受控制
            }

            cout << " controller_v_x:  " << controller_v_x << endl;
            cout << " controller_v_y:  " << controller_v_y << endl;
            cout << " controller_p_z:  " << controller_p_z << endl;
            cout << " controller_yaw:  " << controller_yaw << endl;
            cout << "robot_x_distance  " << robot_x_distance << endl;
            cout << "robot_y_distance  " << robot_y_distance << endl;
            cout << "cur_p_x:  " << _cur_p_x << endl;
            cout << "cur_p_y:  " << _cur_p_y << endl;
        };
        break;

        case ALL_CONTROL_MODE_DOUBLE_INTEG: // 双环全方位控制
        {
            // 根据实际情况切换形状 /////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) > DECREASE_SPEED_RADIUS)
            {
                shape = shape; // 以前是哪种形状,现在还用哪种形状
            }
            else
            {
                shape = SHAPE_X; // 转换为x形状
            }

            // 根据形状施加控制 //////////////////////////////////
            if (shape == SHAPE_H_T) // y方向加控制量 双环
            {
                _exp_yaw -= 90;
                if (_exp_yaw < -180)
                {
                    _exp_yaw += 360;
                }
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_y_distance = -sin(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                robot_y_speed = (-sin(_cur_yaw / 180.0 * pi)) * _cur_v_x + cos(_cur_yaw / 180.0 * pi) * _cur_v_y;

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                controller_v_x = 0;                                              // x方向没有控制量
                controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0); // 斜线位移

                // 斜线速度
                controller_v_y = ss_Vy_ctrl.CalSSCv2output(controller_p_y, robot_y_speed);

                controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z);  // 控制深度
                controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

                vec_ctrl.yawLocomMode_LR();
                vec_ctrl.motorforce(controller_v_x, controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                vec_ctrl.space3DControl(SHAPE_H_T, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
            }
            else if (shape == SHAPE_H) // x方向加控制量 双环
            {
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                robot_x_speed = cos(_cur_yaw / 180.0 * pi) * _cur_v_x + sin(_cur_yaw / 180.0 * pi) * _cur_v_y;

                // 暂时不用全距离
                //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                // 双环x控制量
                controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                controller_v_x = ss_Vx_ctrl.CalSSCv2output(controller_p_x, robot_x_speed);
                // 斜线速度
                controller_v_y = 0;

                controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z);  // 控制深度
                controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

                vec_ctrl.yawLocomMode_FB();
                vec_ctrl.motorforce(controller_v_x, controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                vec_ctrl.space3DControl(SHAPE_H, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
            }
            else if (shape == SHAPE_X)
            {
                // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                robot_x_distance = cos(_cur_yaw / 180.0 * pi) * (_exp_p_x - _cur_p_x) + sin(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                robot_y_distance = (-sin(_cur_yaw / 180.0 * pi)) * (_exp_p_x - _cur_p_x) + cos(_cur_yaw / 180.0 * pi) * (_exp_p_y - _cur_p_y);
                // 速度是个向量 不平移 直接旋转
                robot_x_speed = cos(_cur_yaw / 180.0 * pi) * _cur_v_x + sin(_cur_yaw / 180.0 * pi) * _cur_v_y;
                robot_y_speed = (-sin(_cur_yaw / 180.0 * pi)) * _cur_v_x + cos(_cur_yaw / 180.0 * pi) * _cur_v_y;
                // x y 都施加双环控制
                controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                controller_v_x = ss_Vx_ctrl.CalSSCv2output(controller_p_x, robot_x_speed);

                controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0);
                controller_v_y = ss_Vy_ctrl.CalSSCv2output(controller_p_y, robot_y_speed);

                // z正常控制
                controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z); // 控制深度
                controller_yaw = 0;                                             // x型模式无法调节航向角

                cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

                vec_ctrl.yawLocomMode_X();
                vec_ctrl.motorforce(controller_v_x, controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                vec_ctrl.space3DControl(SHAPE_X, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
            }

            // 判断终止条件 /////////////////////////////////////////////////////
            if (sqrt((_exp_p_x - _cur_p_x) * (_exp_p_x - _cur_p_x) +
                     (_exp_p_y - _cur_p_y) * (_exp_p_y - _cur_p_y)) <= finish_radius)
            {
                vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0); // 全关
                is_judge_direction = false;
                is_running = false;
                debug_mode = 0; // 强制让调试模式为0 终止运行
                shape = 0;      // 强制将形状取消, 不接受控制
            }

            cout << " controller_v_x:  " << controller_v_x << endl;
            cout << " controller_v_y:  " << controller_v_y << endl;
            cout << " controller_p_z:  " << controller_p_z << endl;
            cout << " controller_yaw:  " << controller_yaw << endl;
            cout << "robot_x_distance  " << robot_x_distance << endl;
            cout << "robot_y_distance  " << robot_y_distance << endl;
            cout << "exp_p_x:  " << _exp_p_x << "  " << "cur_p_x:  " << _cur_p_x << endl;
            cout << "exp_p_y:  " << _exp_p_y << "  " << "cur_p_y:  " << _cur_p_y << endl;
            cout << "exp_yaw:  " << _exp_yaw << "  " << "cur_yaw:  " << _cur_yaw << endl;
        };
        break;
        default:
        {
            cout << "there is no this control mode !!! " << endl;
        };
        break;
        }
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /*@ 组合三维速度控制实现
     * 追踪期望速度 只要期望速度追上了, 期望的位置也就追上了
     *
     */
    C_Combine3DSpeedControl::C_Combine3DSpeedControl()
    {
        shape = 0; // 初始化时不接受形状指示

        // 实际值全部初始化为0
        robot_x_speed = 0;
        robot_y_speed = 0;
        robot_yaw_speed = 0;
        robot_z_speed = 0;

        // 期望值全部初始化为0
        robot_exp_x_speed = 0;
        robot_exp_y_speed = 0;
        robot_exp_yaw_speed = 0;
        robot_exp_z_speed = 0;

        // 控制器输出量全部初始化为0
        controller_x_speed = 0;
        controller_y_speed = 0;
        controller_yaw_speed = 0;
        controller_z_speed = 0;
    }

    // hcy:gazebo原程序使用
    //  注意: 输入的期望速度,是机器人坐标系下的,非全局坐标系下的
    void C_Combine3DSpeedControl::adrcSelectSpeedControllerAndOutput(int _shape,
                                                                     float _exp_v_x, float _exp_v_y,
                                                                     float _exp_v_yaw, float _exp_v_z,

                                                                     float _cur_v_x, float _cur_v_y,
                                                                     float _cur_v_yaw, float _cur_v_z)
    {
        // 变量赋值不要放在 switch的下方 否则这段不执行
        robot_exp_x_speed = _exp_v_x;
        robot_x_speed = _cur_v_x;

        robot_exp_y_speed = _exp_v_y;
        robot_y_speed = _cur_v_y;

        robot_exp_yaw_speed = _exp_v_yaw;
        robot_yaw_speed = _cur_v_yaw;

        robot_exp_z_speed = _exp_v_z;
        robot_z_speed = _cur_v_z;

        switch (_shape)
        {

        case SHAPE_H:
        {
            // H 型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            /// DEBUG
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            // controller_x_speed = 0;
            //  速度y控制量 H型模式下 机器人的y方向无法推进
            controller_y_speed = 0;

            controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed, robot_yaw_speed); // 航向角调节
                                                                                                         //            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度
                                                                                                         //  controller_yaw_speed = 0;
            controller_z_speed = 0;
            ///
            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

            speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
        };
        break;
        case SHAPE_H_T:
        {
            // 工型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量 工型模式下 机器人的x方向无法推进
            controller_x_speed = 0;
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);

            controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed, robot_yaw_speed); // 航向角调节
            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

            speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
        };
        break;
        case SHAPE_X:
        {
            // X型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);
            // 航向角速度为0
            controller_yaw_speed = 0;                                                            // 航向角调节
            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed); // 控制深度

            cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            // speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
            //                               controller_z_speed, controller_yaw_speed);
            speed_vec_ctrl.space3DControl(SHAPE_X, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
        };
        break;
        }

        cout << " controller_x_speed:  " << controller_x_speed << endl;
        cout << " controller_y_speed:  " << controller_y_speed << endl;
        cout << " controller_yaw_speed:  " << controller_yaw_speed << endl;
        cout << " controller_z_speed:  " << controller_z_speed << endl;
        cout << "_exp_v_x:  " << robot_exp_x_speed << "  " << "_cur_v_x:  " << robot_x_speed << endl;
        cout << "_exp_v_y:  " << robot_exp_y_speed << "  " << "_cur_v_y:  " << robot_y_speed << endl;
        cout << "_exp_v_yaw:  " << robot_exp_yaw_speed << "  " << "_cur_v_yaw:  " << robot_yaw_speed << endl;
        cout << "_exp_v_z:  " << robot_exp_z_speed << "  " << "_cur_v_z:  " << robot_z_speed << endl;
    }

    // hcy:gazebo原程序使用
    void C_Combine3DSpeedControl::adrcSelectSpeedControllerAndOutput(int _shape,
                                                                     float _exp_v_x, float _exp_v_y,
                                                                     float _exp_v_yaw, float _exp_v_z,
                                                                     float _exp_yaw,

                                                                     float _cur_v_x, float _cur_v_y,
                                                                     float _cur_v_yaw, float _cur_v_z,
                                                                     float _cur_yaw)
    {

        // 变量赋值不要放在 switch的下方 否则这段不执行
        robot_exp_x_speed = _exp_v_x;
        robot_x_speed = _cur_v_x;

        robot_exp_y_speed = _exp_v_y;
        robot_y_speed = _cur_v_y;

        robot_exp_yaw_speed = _exp_v_yaw;
        robot_yaw_speed = _cur_v_yaw;

        robot_exp_z_speed = _exp_v_z;
        robot_z_speed = _cur_v_z;

        robot_exp_yaw = _exp_yaw;
        robot_yaw = _cur_yaw;

        bool __control_v_yaw_flag = true;

        // 期望的航向角是0 开启角度控制
        if ((_exp_v_yaw < 0.000001) && (_exp_v_yaw > -0.000001))
        {
            __control_v_yaw_flag = false;
        }

        cout << "__control_v_yaw_flag: " << __control_v_yaw_flag << endl;

        switch (_shape)
        {

        case SHAPE_H:
        {
            // H 型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            /// DEBUG
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            //            controller_x_speed = 0;
            // 速度y控制量 H型模式下 机器人的y方向无法推进
            controller_y_speed = 0;

            if (__control_v_yaw_flag == true)
            {
                controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed, robot_yaw_speed); // 航向角调节
            }
            else
            {
                controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);
            }
            //            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度
            //            controller_yaw_speed = 0;
            controller_z_speed = 0;
            ///
            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

            // 解算的输入有变化
            if (__control_v_yaw_flag == true)
            {
                speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw_speed);
            }
            else
            {
                speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw);
            }
        };
        break;
        case SHAPE_H_T:
        {
            // 工型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量 工型模式下 机器人的x方向无法推进
            controller_x_speed = 0;
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);

            if (__control_v_yaw_flag == true)
            {
                controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed, robot_yaw_speed); // 航向角调节
            }
            else
            {
                controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);
            }

            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed); // 控制深度

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

            // 解算的输入有变化
            if (__control_v_yaw_flag == true)
            {
                speed_vec_ctrl.space3DControl(SHAPE_H_T, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw_speed);
            }
            else
            {
                speed_vec_ctrl.space3DControl(SHAPE_H_T, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw);
            }
        };
        break;
        case SHAPE_X:
        {
            // X型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);
            // 航向角速度为0
            controller_yaw_speed = 0;                                                            // 航向角调节
            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed); // 控制深度

            cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
        };
        break;
        }

        cout << " controller_x_speed:  " << controller_x_speed << endl;
        cout << " controller_y_speed:  " << controller_y_speed << endl;
        cout << " controller_yaw_speed:  " << controller_yaw_speed << endl;
        cout << " controller_z_speed:  " << controller_z_speed << endl;
        cout << " controller_yaw:  " << controller_yaw << endl;
        cout << "_exp_v_x:  " << robot_exp_x_speed << "  " << "_cur_v_x:  " << robot_x_speed << endl;
        cout << "_exp_v_y:  " << robot_exp_y_speed << "  " << "_cur_v_y:  " << robot_y_speed << endl;
        cout << "_exp_v_yaw:  " << robot_exp_yaw_speed << "  " << "_cur_v_yaw:  " << robot_yaw_speed << endl;
        cout << "_exp_v_z:  " << robot_exp_z_speed << "  " << "_cur_v_z:  " << robot_z_speed << endl;
        cout << "_exp_yaw:  " << robot_exp_yaw << "  " << "_cur_yaw:  " << robot_yaw << endl;
    }

    // hcy
    C_Combine3DPositionControl::C_Combine3DPositionControl()
    {
        shape = 0;
        robot_x_position = 0.0;
        robot_y_position = 0.0;
        robot_yaw = 0.0;
        robot_z_position = 0.0;

        robot_exp_x_position = 0.0;
        robot_exp_y_position = 0.0;
        robot_exp_yaw = 0.0;
        robot_exp_z_position = 0.0;

        controller_x_position = 0.0;
        controller_y_position = 0.0;
        controller_yaw = 0.0;
        controller_z_position = 0.0;
    }

    // hcy
    C_Combine3DPositionControl_v2::C_Combine3DPositionControl_v2()
    {
        shape = 0;

        robot_x_position = 0.0;
        robot_y_position = 0.0;
        robot_yaw = 0.0;
        robot_z_position = 0.0;

        robot_exp_x_position = 0.0;
        robot_exp_y_position = 0.0;
        robot_exp_yaw = 0.0;
        robot_exp_z_position = 0.0;

        controller_x_position = 0.0;
        controller_y_position = 0.0;
        controller_yaw = 0.0;
        controller_z_position = 0.0;
    }

    // hcy
    void C_Combine3DPositionControl::adrcSelectPositionControllerAndOutput(int _shape,
                                                                           float _exp_p_x, float _exp_p_y,
                                                                           float _exp_yaw, float _exp_p_z,

                                                                           float _cur_p_x, float _cur_p_y,
                                                                           float _cur_p_yaw, float _cur_p_z)
    {

        robot_exp_x_position = _exp_p_x;
        robot_exp_y_position = _exp_p_y;
        robot_exp_yaw = _exp_yaw;
        robot_exp_z_position = _exp_p_z;

        robot_x_position = _cur_p_x;
        robot_y_position = _cur_p_y;
        robot_yaw = _cur_p_yaw;
        robot_z_position = _cur_p_z;

        switch (_shape)
        {
        case SHAPE_H:
        {

            // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
            robot_x_distance = cos(robot_yaw / 180.0 * pi) * (robot_exp_x_position - robot_x_position) + sin(robot_yaw / 180.0 * pi) * (robot_exp_y_position - robot_y_position);

            // 单环控制量
            controller_x_position = adrc_x_ctrl.adrcOutput(robot_x_distance, 0);
            // 斜线速度
            controller_y_position = 0;

            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
            controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);                    // 航向角调节

            // 向右转弯迟钝，增大向右转弯能量
            if (controller_yaw < 0)
            {
                controller_yaw = 1.3 * controller_yaw;
            }
            else
            {
                controller_yaw = 1.1 * controller_yaw;
            }
            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
            /// DEBUG
            // 航向角权重设置为2 更看重转弯
            vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, 2 * controller_yaw);
            ///
        };
        break;

        case SHAPE_H_T:
        {

            // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
            robot_y_distance = (-sin(robot_yaw / 180.0 * pi)) * (robot_exp_x_position - robot_x_position) + cos(robot_yaw / 180.0 * pi) * (robot_exp_y_position - robot_y_position);
            // 速度是个向量 不平移 直接旋转

            controller_x_position = 0;                                           // x方向没有控制量
            controller_y_position = adrc_y_ctrl.adrcOutput(robot_y_distance, 0); // 斜线位移

            // 斜线速度
            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
            controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);                    // 航向角调节

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  " << endl;
            /// DEBUG
            // 航向角权重设置为5 更看重转弯的快速性质
            vec_ctrl.space3DControl(SHAPE_H_T, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
            ///
        };
        break;

        case SHAPE_X:
        {

            // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
            robot_x_distance = cos(robot_yaw / 180.0 * pi) * (robot_exp_x_position - robot_x_position) + sin(robot_yaw / 180.0 * pi) * (robot_exp_y_position - robot_y_position);
            robot_y_distance = (-sin(robot_yaw / 180.0 * pi)) * (robot_exp_x_position - robot_x_position) + cos(robot_yaw / 180.0 * pi) * (robot_exp_y_position - robot_y_position);
            // 速度是个向量 不平移 直接旋转
            // x y 都施加双环控制
            controller_x_position = adrc_x_ctrl.adrcOutput(robot_x_distance, 0);

            controller_y_position = adrc_y_ctrl.adrcOutput(robot_y_distance, 0);

            // z正常控制
            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
            controller_yaw = 0;                                                                     // x型模式无法调节航向角

            cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            vec_ctrl.space3DControl(SHAPE_X, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
        };
        break;
        }

        cout << " controller_p_x:  " << controller_x_position << endl;
        cout << " controller_p_y:  " << controller_y_position << endl;
        cout << " controller_p_z:  " << controller_z_position << endl;
        cout << " controller_yaw:  " << controller_yaw << endl;
        cout << "robot_x_distance  " << robot_x_distance << endl;
        cout << "robot_y_distance  " << robot_y_distance << endl;
        cout << "exp_p_x:  " << robot_exp_x_position << "  " << "cur_p_x:  " << robot_x_position << endl;
        cout << "exp_p_y:  " << robot_exp_y_position << "  " << "cur_p_y:  " << robot_y_position << endl;
        cout << "exp_yaw:  " << robot_exp_yaw << "  " << "cur_yaw:  " << robot_yaw << endl;
        cout << "exp_p_z:  " << robot_exp_z_position << "  " << "cur_p_z:  " << robot_z_position << endl;
    }

    // hcy:（重要）视觉、深度、航向角用到
    void C_Combine3DPositionControl_v2::adrcSelectPositionControllerAndOutput(int _shape,
                                                                              float _exp_p_x, float _exp_p_y,
                                                                              float _exp_yaw, float _exp_p_z,
                                                                              float _cur_p_x, float _cur_p_y,
                                                                              float _cur_p_yaw, float _cur_p_z)
    {

        // 创建了一个临时的坐标系，这个坐标系的X轴与机器人当前的前进方向对齐，原点是世界坐标系的原点，叫新全局坐标系
        // 全局的目标点投影到这个临时坐标系上，得到 (robot_exp_x_position, robot_exp_y_position)
        // 机器人当前的全局位置也投影到这个同一个临时坐标系上，得到 (robot_x_position, robot_y_position)
        // 正确逻辑是传入全局坐标下目标点的位姿和当前机器人在全局下的位姿，然后通过坐标转化到新全局坐标下
        // 按照我docking aruco码传入目标点位姿是aruco码相对于机体的，而当前机体的x、y、yaw设置为0，在数学逻辑上进行以下代码是通的

        //     cout << "**************************4.统一到新全局坐标系下**************************" << endl;
        //  目标点的角度和深度
        robot_exp_yaw = _exp_yaw; // hy:根据固定的规划点和机器人在新全局坐标系下的坐标算出来的yaw，
                                  // 机器人在规划点左侧时为负，右侧时为正  //e_yaw
        robot_exp_z_position = _exp_p_z;
        // 机器人当前的朝向和深度
        robot_yaw = _cur_p_yaw; // hy:即cam_yaw，视觉得出的yaw    //c_yaw
        robot_z_position = _cur_p_z;

        // 使用旋转矩阵
        // 全局期望点在这个新中间坐标系下的坐标
        robot_exp_x_position = cos(robot_yaw / 180.0 * pi) * _exp_p_x + sin(robot_yaw / 180.0 * pi) * _exp_p_y;
        robot_exp_y_position = -sin(robot_yaw / 180.0 * pi) * _exp_p_x + cos(robot_yaw / 180.0 * pi) * _exp_p_y;

        // 机器人当前的全局位置在这个新中间坐标系下的坐标
        robot_x_position = cos(robot_yaw / 180.0 * pi) * _cur_p_x + sin(robot_yaw / 180.0 * pi) * _cur_p_y;
        robot_y_position = -sin(robot_yaw / 180.0 * pi) * _cur_p_x + cos(robot_yaw / 180.0 * pi) * _cur_p_y;

        switch (_shape)
        {
        case SHAPE_H:
        {

            //    // 单环控制量
            //    //controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

            //    //斜线速度
            //    controller_y_position = 0;

            //    controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position,robot_z_position); // 控制深度
            //    controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw); // 航向角调节

            //    // 向右转弯迟钝，增大向右转弯能量
            // //    if(controller_yaw  < 0)//hy:现限制在-1.2~1.2
            // //    {
            // //        controller_yaw = 0.6* controller_yaw;//hcy:原始值1.3
            // //    }
            // //    else
            // //    {
            // //        controller_yaw = 0.6* controller_yaw;//hcy:原始值1.1
            // //    }
            //    if(controller_yaw  < 0)//hy:现限制在-1.2~1.2
            //    {
            //        controller_yaw = 0.001* controller_yaw;//hcy:原始值1.3
            //    }
            //    else
            //    {
            //        controller_yaw = 0.001* controller_yaw;//hcy:原始值1.1
            //    }

            //     //hcy:通过航向角设置前进方向的力
            //     if(controller_yaw > 0 && _exp_p_x >_cur_p_x)
            //     {
            //         controller_x_position =1.25 - controller_yaw;
            //         //controller_x_position =0.6 - 0.5*controller_yaw;
            //     }
            //     else if(controller_yaw < 0 && _exp_p_x > _cur_p_x)
            //     {
            //         controller_x_position =1.25 + controller_yaw;
            //         //controller_x_position =0.6 + 0.5*controller_yaw;
            //     }

            //    cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
            //    /// DEBUG
            //    // 航向角权重设置为2 更看重转弯
            //    vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, 2*controller_yaw);
            //    ///

            // hcy
            //  单环控制量
            controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

            // 斜线速度
            controller_y_position = 0;

            controller_z_position = adrc_Pz_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度

            //    robot_exp_yaw = robot_exp_yaw/57.3;//hcy
            //    robot_yaw = robot_yaw/57.3;
            robot_exp_yaw = robot_exp_yaw / 100; // hcy
            robot_yaw = robot_yaw / 100;

            controller_yaw = adrc_Pyaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw); // 航向角调节
                                                                                  //****************************************原来程序*********************************************** */
            // 向右转弯迟钝，增大向右转弯能量
            // 向左转弯迟钝，增大向左转弯能量
            if (controller_yaw < 0) // hy:现限制在-1.2~1.2
            {
                controller_yaw = 0.5 * controller_yaw; // hcy:原始值1.3
            }
            else
            {
                controller_yaw = 0.7 * controller_yaw; // hcy:原始值1.1
            }

            // hcy:通过航向角设置前进方向的力
            if (controller_yaw > 0 && _exp_p_x > _cur_p_x)
            {
                controller_x_position = 0.2 * (2 - controller_yaw);
                // controller_x_position =0.6 - 0.5*controller_yaw;
            }
            else if (controller_yaw < 0 && _exp_p_x > _cur_p_x)
            {
                controller_x_position = 0.2 * (2 + controller_yaw);
                // controller_x_position =0.6 + 0.5*controller_yaw;
            }
            cout << "**************************SHAPE_H**************************" << endl;
            /// DEBUG
            // 航向角权重设置为2 更看重转弯
            //    vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, 2*controller_yaw);
            vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, controller_yaw); // hcy

            ///
        };
        break;

        case SHAPE_H_T:
        {

            controller_x_position = 0;                                                              // x方向没有控制量
            controller_y_position = adrc_y_ctrl.adrcOutput(robot_exp_y_position, robot_y_position); // 斜线位移

            // 斜线速度
            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
            controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);                    // 航向角调节

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  " << endl;
            /// DEBUG
            // 航向角权重设置为1 更看重转弯的快速性质
            vec_ctrl.space3DControl(SHAPE_H_T, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
            ///
        };
        break;

        case SHAPE_X:
        {

            //    // x y
            //    controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

            //    controller_y_position = adrc_y_ctrl.adrcOutput(robot_exp_y_position, robot_y_position);

            //    // z正常控制
            //    controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position,robot_z_position); // 控制深度

            //    controller_yaw = 0; // x型模式无法调节航向角

            //    cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            //    vec_ctrl.space3DControl(SHAPE_X, controller_x_position, controller_y_position, controller_z_position, controller_yaw );

            // hcy
            //  x y

            controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

            controller_y_position = adrc_y_ctrl.adrcOutput(robot_exp_y_position, robot_y_position);

            // z正常控制
            controller_z_position = adrc_Pz_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度

            // vec_ctrl.mcDepthPID.initPID(0.55, 0.02, 0.02);//PID初始化用
            // controller_z_position = vec_ctrl.mcDepthPID.pidNewCalc(robot_exp_z_position - robot_z_position);

            controller_x_position = 1 * controller_x_position; // ljl减慢速度
            controller_y_position = 1 * controller_y_position; // ljl减慢速度

            controller_z_position = 1 * controller_z_position; // ljl减慢速度

            controller_yaw = 0; // x型模式无法调节航向角

            //    cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;
            cout << "**************************SHAPE_X**************************" << endl;

            vec_ctrl.space3DControl(SHAPE_X, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
        };
        break;

        case SHAPE_G:
        {
            float error_forward = sqrt(pow(_exp_p_x - _cur_p_x, 2) + pow(_exp_p_y - _cur_p_y, 2)) * abs(cos(_exp_yaw - _cur_p_yaw * 57.3 / 180));

            cout << "***********************controller forward caculate*************************" << endl;
            float controller_forward = vec_ctrl.mcSpeedPID.pidNewCalc(error_forward);

            float controller_horizontal = 0;
            float error_vertical = _exp_p_z - _cur_p_z;
            cout << "***********************controller vertical caculate*************************" << endl;
            float controller_vertical = vec_ctrl.mcDepthPID.pidNewCalc(error_vertical); // 控制深度

            // controller_z_position = 0.5 * controller_z_position; // ljl减慢速度
            float error_yaw = _cur_p_yaw - _exp_yaw;
            if (error_yaw > 180)
                error_yaw = error_yaw - 360;
            if (error_yaw < -180)
                error_yaw = error_yaw + 360;
            cout << "***********************controller torque caculate*************************" << endl;
            float controller_torque = vec_ctrl.mcYawPID.pidNewCalc(error_yaw);

            vec_ctrl.space3DControl(SHAPE_G, controller_forward, controller_horizontal, controller_vertical, controller_torque);

            cout << "**************************控制量******************************" << endl;
            cout << " controller_forward:  " << controller_forward << endl;
            cout << " controller_horizontal:  " << controller_horizontal << endl;
            cout << " controller_vertical:  " << controller_vertical << endl;
            cout << " controller_torque:  " << controller_torque << endl;
            cout << "***********************************************************************" << endl;
            cout << "exp_p_x:  " << _exp_p_x << "  " << "cur_p_x:  " << _cur_p_x << endl;
            cout << "exp_p_y:  " << _exp_p_y << "  " << "cur_p_y:  " << _cur_p_y << endl;
            cout << "exp_yaw:  " << _exp_yaw << "  " << "cur_yaw:  " << _cur_p_yaw << endl;
            cout << "exp_p_z:  " << robot_exp_z_position << "  " << "cur_p_z:  " << robot_z_position << endl;
        }
        }

        //    cout << "**************************控制量******************************" << endl;
        //    cout << " controller_p_x:  " << controller_x_position <<endl;
        //    cout << " controller_p_y:  " << controller_y_position <<endl;
        //    cout << " controller_p_z:  " << controller_z_position <<endl;
        //    cout << " controller_yaw:  " << controller_yaw <<endl;
        //    cout << "***********************************************************************" << endl;
        //    cout << "exp_p_x:  "<< robot_exp_x_position <<"  "<<"cur_p_x:  "<<robot_x_position <<endl;
        //    cout << "exp_p_y:  "<< robot_exp_y_position <<"  "<<"cur_p_y:  "<<robot_y_position <<endl;
        //    cout << "exp_yaw:  "<< robot_exp_yaw <<"  "<<"cur_yaw:  "<<robot_yaw <<endl;
        //    cout << "exp_p_z:  " << robot_exp_z_position << "  " << "cur_p_z:  " << robot_z_position <<endl;
    }

    //===============================================================================================================
    //                                                 New：PID
    //===============================================================================================================

    C_Combine3DPidControl::C_Combine3DPidControl()
        : cur_p_x_(0), cur_p_y_(0), cur_p_z_(0), cur_yaw_(0), exp_p_x_(0), exp_p_y_(0), exp_p_z_(0), exp_yaw_(0), cur_v_x_(0), cur_v_y_(0), cur_v_z_(0), exp_v_x_(0), exp_v_y_(0), exp_v_z_(0), filtered_cur_v_z_(0), shape_(0), current_debug_mode_(NAV_NORMAL), // 默认是正常导航模式
          debug_target_value_(0.0f)
    {
        initialize_all_PIDs();
        // setControlMode(DUAL_LOOP);
        // setControlMode(SINGLE_LOOP);
    }

    void C_Combine3DPidControl::setControlMode(ControlMode mode)
    {
        this->current_mode_ = mode;
    }

    void C_Combine3DPidControl::setDebugMode(ControlDebugMode mode, float debug_target)
    {
        this->current_debug_mode_ = mode;
        this->debug_target_value_ = debug_target;

        // (可选) 打印一条信息，让你知道模式已切换
        std::cout << "[Control Mode] Switched to debug mode: " << mode
                  << " with target: " << debug_target << std::endl;
    }
    void C_Combine3DPidControl::initialize_all_PIDs()
    {
        // 所有的初始化逻辑都移到这里来了
        // 这样做的好处是，以后你可以让这个函数从文件读取参数

        // --- 初始化双环PID ---
        //vec_ctrl.mcDualLoop_Heave.init(0.05, 0, 20, 5, 0.2, 0.7);
        vec_ctrl.mcDualLoop_Heave.init(0.3, 0, 0.03, 16, 0.02, 5);
        // ... 其他PID的初始化 ...
    }

    void C_Combine3DPidControl::setHeaveInnerLoopParams(float kp, float ki, float kd)
    {
        // 这个函数只是简单地调用底层PID对象的init/set函数。
        // 它将PID参数更新为从外部传入的新值。
        vec_ctrl.mcDualLoop_Heave.setInnerLoopParams(kp, ki, kd);
    }
    void C_Combine3DPidControl::setHeaveOuterLoopParams(float kp, float ki, float kd)
    {
        // 这个函数只是简单地调用底层PID对象的init/set函数。
        // 它将PID参数更新为从外部传入的新值。
        vec_ctrl.mcDualLoop_Heave.setOuterLoopParams(kp, ki, kd);
    }

    void C_Combine3DPidControl::updateControl(
        int shape,
        float exp_p_x, float exp_p_y, float exp_yaw, float exp_p_z,
        float cur_p_x, float cur_p_y, float cur_yaw, float cur_p_z)
    {
        // 1. 将传入的参数存储到类的成员变量中
        this->shape_ = shape;
        this->exp_p_x_ = exp_p_x;
        this->exp_p_y_ = exp_p_y;
        this->exp_p_z_ = exp_p_z;
        this->exp_yaw_ = exp_yaw;
        this->cur_p_x_ = cur_p_x;
        this->cur_p_y_ = cur_p_y;
        this->cur_p_z_ = cur_p_z;
        this->cur_yaw_ = cur_yaw;

        // 2. 调用内部的控制逻辑函数
        this->updatePidControl();
    }

    void C_Combine3DPidControl::updateControl(
        int shape,
        float exp_p_x, float exp_p_y, float exp_yaw, float exp_p_z,
        float exp_v_x, float exp_v_y, float exp_v_z,
        float cur_p_x, float cur_p_y, float cur_yaw, float cur_p_z,
        float cur_v_x, float cur_v_y, float cur_v_z)
    {
        // 1. 将传入的参数存储到类的成员变量中
        this->shape_ = shape;
        this->exp_p_x_ = exp_p_x;
        this->exp_p_y_ = exp_p_y;
        this->exp_p_z_ = exp_p_z;
        this->exp_v_x_ = exp_v_x;
        this->exp_v_y_ = exp_v_y;
        this->exp_v_z_ = exp_v_z;

        this->exp_yaw_ = exp_yaw;
        this->cur_p_x_ = cur_p_x;
        this->cur_p_y_ = cur_p_y;
        this->cur_p_z_ = cur_p_z;
        this->cur_v_x_ = cur_v_x;
        this->cur_v_y_ = cur_v_y;
        this->cur_v_z_ = cur_v_z;
        this->cur_yaw_ = cur_yaw;

        // 2. 调用内部的控制逻辑函数
        this->updatePidControl();
    }

    // =================================================================
    // ===           总控制器输出函数 (重构后)                     ===
    // =================================================================
    void C_Combine3DPidControl::updatePidControl()
    {
        // 根据机器人当前的【形态】和【控制模式】，调用对应的具体控制逻辑处理函数
        if (this->shape_ == SHAPE_H && this->current_mode_ == SINGLE_LOOP)
        {
            handleShapeH_SingleLoop_Control();
        }
        else if (this->shape_ == SHAPE_H && this->current_mode_ == DUAL_LOOP)
        {
            handleShapeH_DualLoop_Control();
        }
        else if (this->shape_ == SHAPE_X && this->current_mode_ == SINGLE_LOOP)
        {
            handleShapeX_SingleLoop_Control();
        }
        else if (this->shape_ == SHAPE_X && this->current_mode_ == DUAL_LOOP)
        {
            handleShapeX_DualLoop_Control();
        }
        else
        {
            // 默认情况：处理未知的形态或模式组合
            // 停止所有电机或进行错误处理，保证安全
            vec_ctrl.space3DControl(this->shape_, 0, 0, 0, 0);
            // 你也可以在这里打印一条错误日志
            // ROS_ERROR("Unknown shape or mode combination! Shape: %d, Mode: %d", this->shape_, this->current_mode_);
        }

        // 步骤 2: 统一打印调试信息
        printDebugInfo();
    }

    // =================================================================
    // ===           私有成员函数：H型控制逻辑处理                   ===
    // =================================================================
    // 单环H型
    void C_Combine3DPidControl::handleShapeH_SingleLoop_Control()
    {
        // =============================================================
        // ===                      单环控制逻辑                       ===
        // =============================================================
        // (位置误差 -> 直接输出力)
        switch (current_debug_mode_)
        {

        case DEBUG_YAW_ONLY:
        {
            // --- 只调试偏航 ---
            float target_yaw = debug_target_value_; // 使用固定的航向目标
            float yaw_error = target_yaw - cur_yaw_;
            float desired_turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error); // ---  控制量分配 ---
            vec_ctrl.space3DControl(SHAPE_H,
                                    0,
                                    0,
                                    0,
                                    desired_turning_torque);
            // 打印关键调试信息
            printf("[SingleLoop yaw] Tgt_yaw: %.2f | Cur_yaw: %.2f | Error_yaw: %.2f | Out_F: %.2f\n",
                   target_yaw,
                   cur_yaw_,
                   yaw_error,
                   desired_turning_torque);
        }
        break;

        case DEBUG_Z_ONLY:
        {
            // ---  深度误差 ---
            float exp_p_z_ = debug_target_value_; // 使用固定的航向目标
            float depth_error = exp_p_z_ - cur_p_z_;
            float desired_up_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
            float z_force_for_alloc = desired_up_force;

            vec_ctrl.space3DControl(SHAPE_H,
                                    0,
                                    0,
                                    z_force_for_alloc,
                                    0);
            // 打印关键调试信息
            printf("[SingleLoop Dep] Tgt_z: %.2f | Cur_z: %.2f | Error_z: %.2f | Out_F: %.2f\n",
                   exp_p_z_,
                   cur_p_z_,
                   depth_error,
                   z_force_for_alloc);
        }
        break;

        case DEBUG_DEPYAW_ONLY:
        {
            // --- 调试偏航 ---
            float target_yaw = debug_target_value_; // 使用固定的航向目标
            float yaw_error = target_yaw - cur_yaw_;
            float desired_turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error); // ---  控制量分配 ---
                                                                                    // ---  深度误差 ---
            float depth_error = exp_p_z_ - cur_p_z_;
            float desired_up_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
            float z_force_for_alloc = desired_up_force;
            vec_ctrl.space3DControl(SHAPE_H,
                                    0,
                                    0,
                                    z_force_for_alloc,
                                    desired_turning_torque);
            // 打印关键调试信息
            printf("[SingleLoop yaw] Tgt_yaw: %.2f | Cur_yaw: %.2f | Error_yaw: %.2f | Out_F: %.2f\n",
                   target_yaw,
                   cur_yaw_,
                   yaw_error,
                   desired_turning_torque);
            printf("[SingleLoop dep] Tgt_dep: %.2f | Cur_dep: %.2f | Error_dep: %.2f | Out_F: %.2f\n",
                   exp_p_z_,
                   cur_p_z_,
                   depth_error,
                   z_force_for_alloc);
        }
        break;

        case NAV_NORMAL:
        {
            // 默认情况是正常导航
            // --- 正常导航模式 ---

            // --- 1. 计算核心误差 ---

            // a. 计算世界坐标系下的位置误差向量
            float error_vec_x = exp_p_x_ - cur_p_x_;
            float error_vec_y = exp_p_y_ - cur_p_y_;

            // b. 计算到目标的【直线距离误差】-> 这是前进控制器的输入
            float distance_error = sqrt(error_vec_x * error_vec_x + error_vec_y * error_vec_y);

            // c. 计算机器人应该朝向的【目标航向】
            float target_yaw = atan2(error_vec_y, error_vec_x) * 180.0 / M_PI; // 转换为角度
            exp_yaw_ = target_yaw;

            // d. 计算【航向误差】，并处理环绕问题
            float yaw_error = target_yaw - cur_yaw_;
            // ... (处理 >180 和 < -180 的情况) -> 方向问题，让其以最小的角度转到期望的yaw...
            if (yaw_error > 180.0)
            {
                yaw_error -= 360.0;
            }
            else if (yaw_error < -180.0)
            {
                yaw_error += 360.0;
            }
            // --- 2. PID决策 (两个控制器独立计算) ---

            // a. 前进控制器决策
            float desired_forward_force = vec_ctrl.mcSpeedPID.pidNewCalc(distance_error);
            // float desired_forward_force = 0;//调式转向时前进控制量为0

            // b. 航向控制器决策
            float desired_turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error);

            // --- 3. 智能协调策略 (让两个决策者“沟通”) ---

            // 这是设计的精髓！我们不希望在方向盘还没打对的时候就猛踩油门。
            const float YAW_ERROR_THRESHOLD = 15.0; // 航向误差容忍阈值，单位：度
            const float ACCEPTANCE_RADIUS = 0.08;   // 到达目标的容忍半径，单位：米

            // 策略A：到达目标点附近，停车！
            if (distance_error < ACCEPTANCE_RADIUS)
            {
                desired_forward_force = 0.0;
                desired_turning_torque = 0.0; // 也可以选择让它继续微调航向
            }
            // 策略B：航向偏差太大，优先转弯！
            else if (abs(yaw_error) > YAW_ERROR_THRESHOLD)
            {
                // 将前进的力设置为0或一个很小的值，让机器人几乎原地转弯
                // desired_forward_force = 0.0;

                // (可选) 甚至可以稍微加大转向力矩，让它转得更快
                desired_turning_torque *= 1.2;
            }

            // ---  深度误差 ---
            float depth_error = exp_p_z_ - cur_p_z_;
            float desired_up_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
            float z_force_for_alloc = desired_up_force;

            // ---  控制量分配 ---
            vec_ctrl.space3DControl(SHAPE_H,
                                    desired_forward_force,
                                    0,
                                    z_force_for_alloc,
                                    desired_turning_torque);
            // 打印关键调试信息
            printf("[SingleLoop yaw] Tgt_yaw: %.2f | Cur_yaw: %.2f | Error_yaw: %.2f | Out_F: %.2f\n",
                   exp_yaw_,
                   cur_yaw_,
                   yaw_error,
                   desired_turning_torque);

            printf("[SingleLoop forward] Error_forward: %.2f | Out_F: %.2f\n",
                   distance_error,
                   desired_forward_force);
        }
        break;
        default:
            break;
        }
    }

    // 双环H型
    void C_Combine3DPidControl::handleShapeH_DualLoop_Control()
    {
        // =============================================================
        // ===                      双环控制逻辑-先用单环的                       ===
        // =============================================================
        switch (current_debug_mode_)
        {

        case DEBUG_YAW_ONLY:
        {
            // --- 只调试偏航 ---
            float target_yaw = debug_target_value_; // 使用固定的航向目标
            float yaw_error = target_yaw - cur_yaw_;
            float desired_turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error); // ---  控制量分配 ---
            vec_ctrl.space3DControl(SHAPE_H,
                                    0,
                                    0,
                                    0,
                                    desired_turning_torque);
            // 打印关键调试信息
            printf("[SingleLoop yaw] Tgt_yaw: %.2f | Cur_yaw: %.2f | Error_yaw: %.2f | Out_F: %.2f\n",
                   target_yaw,
                   cur_yaw_,
                   yaw_error,
                   desired_turning_torque);
        }
        break;

        case DEBUG_Z_ONLY:
        {
            // ---  深度误差 ---
            float exp_p_z_ = debug_target_value_; // 使用固定的航向目标
            float depth_error = exp_p_z_ - cur_p_z_;
            float desired_up_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
            float z_force_for_alloc = desired_up_force;

            vec_ctrl.space3DControl(SHAPE_H,
                                    0,
                                    0,
                                    z_force_for_alloc,
                                    0);
            // 打印关键调试信息
            printf("[SingleLoop Dep] Tgt_z: %.2f | Cur_z: %.2f | Error_z: %.2f | Out_F: %.2f\n",
                   exp_p_z_,
                   cur_p_z_,
                   depth_error,
                   z_force_for_alloc);
        }
        break;

        case DEBUG_DEPYAW_ONLY:
        {
            // --- 调试偏航 ---
            float target_yaw = debug_target_value_; // 使用固定的航向目标
            float yaw_error = target_yaw - cur_yaw_;
            float desired_turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error); // ---  控制量分配 ---
                                                                                    // ---  深度误差 ---
            float depth_error = exp_p_z_ - cur_p_z_;
            float desired_up_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
            float z_force_for_alloc = desired_up_force;
            vec_ctrl.space3DControl(SHAPE_H,
                                    0,
                                    0,
                                    z_force_for_alloc,
                                    desired_turning_torque);
            // 打印关键调试信息
            printf("[SingleLoop yaw] Tgt_yaw: %.2f | Cur_yaw: %.2f | Error_yaw: %.2f | Out_F: %.2f\n",
                   target_yaw,
                   cur_yaw_,
                   yaw_error,
                   desired_turning_torque);
            printf("[SingleLoop dep] Tgt_dep: %.2f | Cur_dep: %.2f | Error_dep: %.2f | Out_F: %.2f\n",
                   exp_p_z_,
                   cur_p_z_,
                   depth_error,
                   z_force_for_alloc);
        }
        break;

        case NAV_NORMAL:
        {
            // 默认情况是正常导航
            // --- 正常导航模式 ---

            // --- 1. 计算核心误差 ---

            // a. 计算世界坐标系下的位置误差向量
            float error_vec_x = exp_p_x_ - cur_p_x_;
            float error_vec_y = exp_p_y_ - cur_p_y_;

            // b. 计算到目标的【直线距离误差】-> 这是前进控制器的输入
            float distance_error = sqrt(error_vec_x * error_vec_x + error_vec_y * error_vec_y);

            // c. 计算机器人应该朝向的【目标航向】
            float target_yaw = atan2(error_vec_y, error_vec_x) * 180.0 / M_PI; // 转换为角度
            exp_yaw_ = target_yaw;

            // d. 计算【航向误差】，并处理环绕问题
            float yaw_error = target_yaw - cur_yaw_;
            // ... (处理 >180 和 < -180 的情况) -> 方向问题，让其以最小的角度转到期望的yaw...
            if (yaw_error > 180.0)
            {
                yaw_error -= 360.0;
            }
            else if (yaw_error < -180.0)
            {
                yaw_error += 360.0;
            }
            // --- 2. PID决策 (两个控制器独立计算) ---

            // a. 前进控制器决策
            float desired_forward_force = vec_ctrl.mcSpeedPID.pidNewCalc(distance_error);
            // float desired_forward_force = 0;//调式转向时前进控制量为0

            // b. 航向控制器决策
            float desired_turning_torque = vec_ctrl.mcYawPID.pidNewCalc(yaw_error);

            // --- 3. 智能协调策略 (让两个决策者“沟通”) ---

            // 这是设计的精髓！我们不希望在方向盘还没打对的时候就猛踩油门。
            const float YAW_ERROR_THRESHOLD = 15.0; // 航向误差容忍阈值，单位：度
            const float ACCEPTANCE_RADIUS = 0.08;   // 到达目标的容忍半径，单位：米

            // 策略A：到达目标点附近，停车！
            if (distance_error < ACCEPTANCE_RADIUS)
            {
                desired_forward_force = 0.0;
                desired_turning_torque = 0.0; // 也可以选择让它继续微调航向
            }
            // 策略B：航向偏差太大，优先转弯！
            else if (abs(yaw_error) > YAW_ERROR_THRESHOLD)
            {
                // 将前进的力设置为0或一个很小的值，让机器人几乎原地转弯
                // desired_forward_force = 0.0;

                // (可选) 甚至可以稍微加大转向力矩，让它转得更快
                desired_turning_torque *= 1.2;
            }

            // ---  深度误差 ---
            float depth_error = exp_p_z_ - cur_p_z_;
            float desired_up_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);
            float z_force_for_alloc = desired_up_force;

            // ---  控制量分配 ---
            vec_ctrl.space3DControl(SHAPE_H,
                                    desired_forward_force,
                                    0,
                                    z_force_for_alloc,
                                    desired_turning_torque);
            // 打印关键调试信息
            printf("[SingleLoop yaw] Tgt_yaw: %.2f | Cur_yaw: %.2f | Error_yaw: %.2f | Out_F: %.2f\n",
                   exp_yaw_,
                   cur_yaw_,
                   yaw_error,
                   desired_turning_torque);

            printf("[SingleLoop forward] Error_forward: %.2f | Out_F: %.2f\n",
                   distance_error,
                   desired_forward_force);
        }
        break;
        default:
            break;
        }
    }

    // =================================================================
    // ===           私有成员函数：X型控制逻辑处理                   ===
    // =================================================================
    // 单环X型
    void C_Combine3DPidControl::handleShapeX_SingleLoop_Control()
    {
        // =============================================================
        // ===                      单环控制逻辑                       ===
        // =============================================================
        // (位置误差 -> 直接输出力)
        switch (current_debug_mode_)
        {
        case NAV_NORMAL:
        {
            // =================================================================
            // 步骤 1: 在【世界坐标系】下计算误差并得到期望力
            // =================================================================
            float x_error = exp_p_x_ - cur_p_x_;
            float y_error = exp_p_y_ - cur_p_y_;
            float depth_error = exp_p_z_ - cur_p_z_;

            float desired_world_x_force = vec_ctrl.mcXPID.pidNewCalc(x_error);
            float desired_world_y_force = vec_ctrl.mcYPID.pidNewCalc(y_error);
            float desired_z_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);

            // =================================================================
            // 步骤 2: 【关键】将世界坐标系力旋转到机器人坐标系
            // =================================================================
            // 获取当前航向角 (需要是弧度制)
            float cur_yaw_rad = cur_yaw_ * M_PI / 180.0;
            float cos_yaw = cos(cur_yaw_rad);
            float sin_yaw = sin(cur_yaw_rad);

            // 应用旋转矩阵的逆（从世界系到机器人系）
            float desired_robot_x_force = desired_world_x_force * cos_yaw + desired_world_y_force * sin_yaw;
            float desired_robot_y_force = -desired_world_x_force * sin_yaw + desired_world_y_force * cos_yaw;

            // =================== 在这里添加调试打印 ===================
            std::cout << "DEBUG: Target Force -> X: " << desired_robot_x_force
                      << ", Y: " << desired_robot_y_force
                      << ", Z: " << desired_z_force << std::endl;
            // ==========================================================

            // =================================================================
            // 步骤 3: 调用底层控制器
            // =================================================================
            vec_ctrl.space3DControl(SHAPE_X,
                                    desired_robot_x_force,
                                    desired_robot_y_force,
                                    desired_z_force,
                                    0 // 航向力矩可以由独立的PID控制，这里暂不考虑
            );
            // =================================================================
            // 步骤 5: (可选) 更新调试信息
            // =================================================================
            // 你可以把计算出的中间值存到成员变量里，方便外部打印或发布
            // this->vec_ctrl.fmcSpeedpidError = distance_error;
            // this->vec_ctrl.fmcYawpidError = yaw_error;
            // this->vec_ctrl.fmcDepthpidError = depth_error;
            // this->vec_ctrl.fmcSpeedpidValue = desired_forward_force;
            // this->vec_ctrl.fmcYawpidValue = desired_turning_torque; // 注意，这里存的是力矩，而不是Y力
            // this->vec_ctrl.fmcDepthpidValue = desired_z_force;
        }
        break;
        default:
            break;
        }
    }

    // 双环X型
    void C_Combine3DPidControl::handleShapeX_DualLoop_Control()
    {
        // =============================================================
        // ===                      双环控制逻辑，只有z实现双环                       ===
        // =============================================================
        // 级联PID：(位置误差 -> 期望速度 -> 速度误差 -> 期望的力)

        switch (current_debug_mode_)
        {
        case NAV_NORMAL:
        {

            bool test_inner_loop_only = false; // 用来内环调参

            if (test_inner_loop_only)
            {
                // --- 只测试内环 ---
                float desired_velocity_test = -0.15f;
                exp_v_z_ = desired_velocity_test;

                float z_force_for_alloc = vec_ctrl.mcDualLoop_Heave.calculateInnerLoopOnly(
                    desired_velocity_test, // 手动给期望速度
                    cur_v_z_               // 传入当前速度
                );

                vec_ctrl.space3DControl(SHAPE_X, 0, 0, z_force_for_alloc, 0);

                printf("[InnerLoop Z] Tgt: %.2f | Cur: %.2f | Err: %.2f | Out: %.2f\n",
                       desired_velocity_test,
                       cur_v_z_,
                       vec_ctrl.mcDualLoop_Heave.getInnerLoopError(),
                       z_force_for_alloc);
            }
            else
            {
                // 一阶滤波
                // 公式: new_filtered = raw_measurement * alpha + old_filtered * (1 - alpha)
                const float VELOCITY_FILTER_ALPHA = 0.2; // 滤波系数，可以调整 (0.1-0.8)
                // this->filtered_cur_v_z_ = cur_v_z_ * VELOCITY_FILTER_ALPHA + (1 - VELOCITY_FILTER_ALPHA) * this->filtered_cur_v_z_;
                //  --- 正常进行双环计算 ---
                float z_force_for_alloc = vec_ctrl.mcDualLoop_Heave.calculate(
                    exp_p_z_,
                    cur_p_z_,
                    cur_v_z_);

                exp_v_z_ = vec_ctrl.mcDualLoop_Heave.getOuterLoopOutput();

                float x_error = exp_p_x_ - cur_p_x_;
                float y_error = exp_p_y_ - cur_p_y_;
                float desired_world_x_force = vec_ctrl.mcXPID.pidNewCalc(x_error);
                float desired_world_y_force = vec_ctrl.mcYPID.pidNewCalc(y_error);
                float cur_yaw_rad = cur_yaw_ * M_PI / 180.0;
                float cos_yaw = cos(cur_yaw_rad);
                float sin_yaw = sin(cur_yaw_rad);

                // 应用旋转矩阵的逆（从世界系到机器人系）
                float desired_robot_x_force = desired_world_x_force * cos_yaw + desired_world_y_force * sin_yaw;
                float desired_robot_y_force = -desired_world_x_force * sin_yaw + desired_world_y_force * cos_yaw;

                // =================== 在这里添加调试打印 ===================
                std::cout << "DEBUG: Target Force -> X: " << desired_robot_x_force
                          << ", Y: " << desired_robot_y_force
                          << ", Z: " << z_force_for_alloc << std::endl;
                // ==========================================================

                // =================================================================
                // 步骤 3: 调用底层控制器
                // =================================================================
                vec_ctrl.space3DControl(SHAPE_X,
                                        desired_robot_x_force,
                                        desired_robot_y_force,
                                        z_force_for_alloc,
                                        0 // 航向力矩可以由独立的PID控制，这里暂不考虑
                );

                // vec_ctrl.space3DControl(SHAPE_X, 0, 0, z_force_for_alloc, 0);

                // 打印关键调试信息
                printf("[DualLoop Z] Tgt_p: %.2f | Cur_p: %.2f | Tgt_v: %.2f | Cur_v: %.2f | Out_F: %.2f\n",
                       exp_p_z_,
                       cur_p_z_,
                       exp_v_z_,
                       cur_v_z_,
                       z_force_for_alloc);
            }
            //         // =================================================================
            //         // 步骤 1: 在【世界坐标系】下计算误差并得到期望力
            //         // =================================================================
            //         float x_error = exp_p_x_ - cur_p_x_;
            //         float y_error = exp_p_y_ - cur_p_y_;
            //         float depth_error = exp_p_z_ - cur_p_z_;

            //         float desired_world_x_force = vec_ctrl.mcXPID.pidNewCalc(x_error);
            //         float desired_world_y_force = vec_ctrl.mcYPID.pidNewCalc(y_error);
            //         float desired_z_force = vec_ctrl.mcDepthPID.pidNewCalc(depth_error);

            //         // =================================================================
            //         // 步骤 2: 【关键】将世界坐标系力旋转到机器人坐标系
            //         // =================================================================
            //         // 获取当前航向角 (需要是弧度制)
            //         float cur_yaw_rad = cur_yaw_ * M_PI / 180.0;
            //         float cos_yaw = cos(cur_yaw_rad);
            //         float sin_yaw = sin(cur_yaw_rad);

            //         // 应用旋转矩阵的逆（从世界系到机器人系）
            //         float desired_robot_x_force = desired_world_x_force * cos_yaw + desired_world_y_force * sin_yaw;
            //         float desired_robot_y_force = -desired_world_x_force * sin_yaw + desired_world_y_force * cos_yaw;

            //         // =================== 在这里添加调试打印 ===================
            //         std::cout << "DEBUG: Target Force -> X: " << desired_robot_x_force
            //                 << ", Y: " << desired_robot_y_force
            //                 << ", Z: " << desired_z_force << std::endl;
            //         // ==========================================================

            //         // =================================================================
            //         // 步骤 3: 调用底层控制器
            //         // =================================================================
            //         vec_ctrl.space3DControl(SHAPE_X,
            //                                 desired_robot_x_force,
            //                                 desired_robot_y_force,
            //                                 desired_z_force,
            //                                 0 // 航向力矩可以由独立的PID控制，这里暂不考虑
            //         );
        }
        break;
        }
    }
    void C_Combine3DPidControl::debug_msg(ros::Publisher &pub)
    {
        asrobot::mc motion_debug;
        motion_debug.c_x = this->cur_p_x_;
        motion_debug.c_y = cur_p_y_;
        motion_debug.c_z = cur_p_z_;
        motion_debug.c_speedx = cur_v_x_;
        motion_debug.c_speedy = cur_v_y_;
        motion_debug.c_speedz = cur_v_z_;
        motion_debug.c_navyaw = cur_yaw_;

        motion_debug.e_x = exp_p_x_;
        motion_debug.e_y = exp_p_y_;
        motion_debug.e_z = exp_p_z_;
        motion_debug.e_speedx = exp_v_x_;
        motion_debug.e_speedy = exp_v_y_;
        motion_debug.e_speedz = exp_v_z_;
        motion_debug.e_navyaw = exp_yaw_;

        // 统一发布调试信息
        pub.publish(motion_debug);
    }

    // =================================================================
    // ===           私有成员函数：统一打印调试信息                  ===
    // =================================================================
    void C_Combine3DPidControl::printDebugInfo()
    {
        if (current_mode_ == SINGLE_LOOP)
        {
            cout << "************************** SINGLE_LOOP DEBUG ******************************" << endl;
            cout << "Speed PID -> Force:  " << vec_ctrl.fmcSpeedpidValue << endl;
            cout << "Yaw PID -> Torque:   " << vec_ctrl.fmcYawpidValue << endl;
            cout << "Depth PID -> Force:  " << vec_ctrl.fmcDepthpidValue << endl;
            cout << "---------------------------------------------------------------------------" << endl;
            cout << "Target Pos (X,Y,Z,Yaw): " << exp_p_x_ << ", " << exp_p_y_ << ", " << exp_p_z_ << ", " << exp_yaw_ << endl;
            cout << "Current Pos(X,Y,Z,Yaw): " << cur_p_x_ << ", " << cur_p_y_ << ", " << cur_p_z_ << ", " << cur_yaw_ << endl;
            cout << "***************************************************************************" << endl;
        }
        else if (current_mode_ == DUAL_LOOP)
        {
            cout << "************************** DUAL_LOOP DEBUG (Heave) ************************" << endl;
            cout << "Outer Loop (Position): " << endl;
            cout << "  - Target Pos: " << exp_p_z_ << " | Current Pos: " << cur_p_z_ << " | Error: " << vec_ctrl.mcDualLoop_Heave.getOuterLoopOutput() << endl;
            cout << "  - Output (Desired Vel): " << vec_ctrl.mcDualLoop_Heave.getOuterLoopOutput() << endl;
            cout << "---------------------------------------------------------------------------" << endl;
            cout << "Inner Loop (Velocity): " << endl;
            cout << "  - Target Vel: " << vec_ctrl.fmcDualLoop_Heave_OuterOutput << " | Current Vel: " << cur_v_z_ << " | Error: " << vec_ctrl.mcDualLoop_Heave.getInnerLoopError() << endl;
            cout << "  - Output (Final Force): " << vec_ctrl.mcDualLoop_Heave.getInnerLoopOutput() << endl;
            cout << "***************************************************************************" << endl;
        }
    }
} // namespace 括号
