#include "../../include/asrobot/c_sim_3d_ctrl.h"

//#define pi 3.1415926535897932384626
 
namespace CombineCtrl {
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
         is_running = true;         // 机器人运行标志位 初始时启动机器人运行

         shape = 0;  // 腿部变形表示
         region = 0; // 目标点所在的实际区域

         // 机器人坐标系下 期望变量
         robot_whole_distance = 0;  // 斜向距离
         robot_x_distance = 0;      // 机器人坐标系下,目标点的x值
         robot_y_distance = 0;     // 机器人坐标系下,目标点的y值
         robot_x_speed = 0;        // 机器人坐标系下, x方向的速度
         robot_y_speed = 0;        // 机器人坐标系下, y方向的速度

         // 期望变量 这些量在全局坐标系下
         exp_p_x = 0;             // 全局坐标系下 目标的x坐标
         exp_v_x = 0;             // 全局坐标系下 期望的x方向速度 (这个量一般不直接给)

         exp_p_y = 0;             // 全局坐标系下 目标的y坐标
         exp_v_y = 0;             // 全局坐标系下 期望的y方向速度 (这个量一般不直接给)

         exp_p_z = 0;             // 全局坐标系下 目标的z坐标

         exp_yaw = 0;             // 期望航线角度
         exp_roll = 0;
         exp_pitch = 0;

         // 反馈(实时采集)量
         cur_p_x = 0;            // 全局坐标系下 当前的x坐标
         cur_v_x = 0;           // 全局坐标系下 机器人x方向前进速度

         cur_p_y = 0;            // 全局坐标系下 当前的y坐标
         cur_v_y = 0;            // 全局坐标系下 机器人y方向前进速度

         cur_p_z = 0;            // 全局坐标系下 当前的z坐标

         cur_yaw = 0;            // 当前航向角
         cur_roll = 0;
         cur_pitch = 0;


         // 控制器输出量
         controller_p_x = 0;        // 机器人坐标系下 沿着x方向位置控制量
         controller_v_x = 0;        // 机器人坐标系下 沿着x方向速度控制量

         controller_p_y = 0;         // 机器人坐标系下 沿着y方向位置控制量
         controller_v_y = 0;         // 机器人坐标系下 沿着y方向速度控制量

         controller_p_z = 0;         // 机器人坐标系下 沿着z方向的位置控制量

         controller_yaw = 0;

         // 调试指示变量
         debug_mode = 0;              // 调试模式指示变量
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
     void C_Combine3DControl::judge_Direction(float _exp_yaw, float _cur_yaw ) // 均为角度制
     {
         if(is_judge_direction == false)
         {
             // 航向角在 0 - 45 上

             if( (_cur_yaw > 0) && (_cur_yaw < 45 ) )
             {
                 if( ( _exp_yaw > (-45+_cur_yaw) ) && ( _exp_yaw < (45+_cur_yaw) ) )
                 {
                     region = ROBOT_REGION_1;
                 }
                 else if( ( _exp_yaw > (45+_cur_yaw) ) && ( _exp_yaw < (135+_cur_yaw) ) )
                 {
                     region = ROBOT_REGION_2;
                 }
                 else if( ( (_exp_yaw > (135+_cur_yaw)) && (_exp_yaw < 180 ) ) ||
                          ( ( _exp_yaw > -180) && (_exp_yaw < (-135+_cur_yaw))) )
                 {
                     region = ROBOT_REGION_3;
                 }
                 else if( ( _exp_yaw > (-135+cur_yaw) )  &&  ( _exp_yaw < (-45+cur_yaw) ) )
                 {
                     region = ROBOT_REGION_4;
                 }
             }
             else if( (_cur_yaw > 45) && (_cur_yaw < 135) )
             {
                 if( (_exp_yaw > (-45+_cur_yaw)) && (_exp_yaw>(45+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_1;
                 }
                 else if( ((_exp_yaw > (45+_cur_yaw)) &&(_exp_yaw < 180)) ||
                          ((_exp_yaw > -180) && (_exp_yaw < (135+_cur_yaw-360))) )
                 {
                     region = ROBOT_REGION_2;
                 }
                 else if( (_exp_yaw > (135+_cur_yaw-360)) && (_exp_yaw < (-135+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_3;
                 }
                 else if( (_exp_yaw > (-135+_cur_yaw)) && (_exp_yaw < (-45+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_4;
                 }
             }
             else if( (_cur_yaw > 135) && (_cur_yaw < 180) )
             {
                 if( ((_exp_yaw > (-45+_cur_yaw)) && (_exp_yaw < 180)) ||
                         ((_exp_yaw > -180) && (_exp_yaw < (45+_cur_yaw-360))) )
                 {
                     region = ROBOT_REGION_1;
                 }
                 else if( (_exp_yaw > (45+_cur_yaw-360)) && (_exp_yaw < (135+_cur_yaw-360)) )
                 {
                     region = ROBOT_REGION_2;
                 }
                 else if( (_exp_yaw > (135+_cur_yaw-360)) && (_exp_yaw < (-135+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_3;
                 }
                 else if( (_exp_yaw > (-135+_cur_yaw)) && (_exp_yaw < (-45+_cur_yaw) ) )
                 {
                     region = ROBOT_REGION_4;
                 }
             }
             else if( (_cur_yaw > -45) && (_cur_yaw < 0) )
             {
                 if( (_exp_yaw > (-45+_cur_yaw)) && (_exp_yaw < (45+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_1;
                 }
                 else if( (_exp_yaw > (45+_cur_yaw)) && (_exp_yaw < (135+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_2;
                 }
                 else if( ((_exp_yaw > (135+_cur_yaw)) && (_exp_yaw < 180)) ||
                          ( (_exp_yaw > -180) && (_exp_yaw < (-135+_cur_yaw))) )
                 {
                     region = ROBOT_REGION_3;
                 }
                 else if( (_exp_yaw > (-135+_cur_yaw)) && ( _exp_yaw < (-45+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_4;
                 }
             }
             else if( (_cur_yaw > -135) && (_cur_yaw < -45) )
             {
                 if( (_exp_yaw > (-45+_cur_yaw)) && ( _exp_yaw < (45+_cur_yaw)) )
                 {
                     region = ROBOT_REGION_1;
                 }
                 else if( ((_exp_yaw) > (45+_cur_yaw)) && (_exp_yaw < (135+_cur_yaw))  )
                 {
                     region = ROBOT_REGION_2;
                 }
                 else if( ( _exp_yaw > (135+_cur_yaw) ) && (_exp_yaw < (-135+_cur_yaw+360)) )
                 {
                     region = ROBOT_REGION_3;
                 }
                 else if( ((_exp_yaw > (-135+_cur_yaw+360)) && (_exp_yaw < 180)) ||
                          (( _exp_yaw>-180 ) && ( _exp_yaw < (-45+_cur_yaw) ))  )
                 {
                     region = ROBOT_REGION_4;
                 }
             }
             else if( (_cur_yaw > -180) && (_cur_yaw < -135) )
             {
                 if( ((_exp_yaw > (-45+_cur_yaw+360)) && (_exp_yaw < 180)) ||
                     ( (_exp_yaw > -180) && (_exp_yaw < (45+_cur_yaw)) )   )
                 {
                     region = ROBOT_REGION_1;
                 }
                 else if( (_exp_yaw > (45+_cur_yaw)) && ( _exp_yaw < (135+_cur_yaw) ) )
                 {
                     region = ROBOT_REGION_2;
                 }
                 else if( (_exp_yaw > (135+_cur_yaw)) && ( _exp_yaw < (-135+_cur_yaw+360) ) )
                 {
                     region = ROBOT_REGION_3;
                 }
                 else if( (_exp_yaw > (-135+_cur_yaw+360)) && (_exp_yaw < (-45+_cur_yaw+360)) )
                 {
                     region = ROBOT_REGION_4;
                 }
             }

             // 调整形状
             if( (region == ROBOT_REGION_1) || (region == ROBOT_REGION_3) )
             {
                 shape = SHAPE_H;
             }
             else
             {
                 shape = SHAPE_H_T;
             }

             is_running = true ; // 打开机器人运行开关

             is_judge_direction = true;  // 表示已经判断过标志位了
         }
         cout << "region : " << region <<endl;
         cout << "shape :" << shape <<endl;
         cout << "_exp_yaw:  " << _exp_yaw <<endl;
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
     //hcy:gazebo原程序p使用的
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


         switch(_mode)
         {
         case 0:
             {
                 cout << " now it has stop !!! " << endl;
             };break;
         case Px_CONTROL_MODE:  // 控x方向位置
             {
                 robot_x_distance = cos(_cur_yaw / 180.0 * pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw/180*pi ) * ( _exp_p_y - _cur_p_y );
                 controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                 vec_ctrl.motorforce(controller_p_x, 0, 0, 0, cur_roll, _cur_yaw, cur_pitch);
		         vec_ctrl.PxControlWithPxForce(controller_p_x);

                 cout << " controller_p_x:  " << controller_p_x <<endl;
                 cout << "robot_x_distance" << robot_x_distance << endl;
                 cout << "_cur_yaw" << _cur_yaw <<endl;
             };break;
         case Vx_CONTROL_MODE:  // 控x方向速度
             {
                 controller_v_x = adrc_Vx_ctrl.adrcOutput(_exp_v_x, _cur_v_x);
                 vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.VxControlWithVxForce(controller_v_x);
                 cout << " controller_v_x:  " << controller_v_x <<endl;
             };break;

         case Py_CONTROL_MODE: // 控y方向位置
             {

                 controller_p_y = adrc_Py_ctrl.adrcOutput(_exp_p_y, _cur_p_y);
                 vec_ctrl.motorforce(0, controller_p_y, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.PyControlWithPyForce(controller_p_y);
                 cout << " controller_p_y:  " << controller_p_y <<endl;
             };break;
         case Vy_CONTROL_MODE: // 控y方向速度
             {
                 controller_v_y = adrc_Vy_ctrl.adrcOutput(_exp_v_y, _cur_v_y);
                 vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.VyControlWithVyForce(controller_v_y);
                 cout << " controller_v_y:  " << controller_v_y <<endl;
             };break;

         case Pz_CONTROL_MODE: // 控z方向位置  0.7 为抵消浮力的参考量
             {
                 controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2;
                 vec_ctrl.motorforce(0, 0, controller_p_z, 0, 0, _cur_yaw, 0);
		 vec_ctrl.depthControlWithPzForce(controller_p_z);
                 cout << " controller_p_z:  " << controller_p_z <<endl;
             };break;
         case YAW_CONTROL_MODE: // 控航向角
             {
                 controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw);
                 vec_ctrl.motorforce(0, 0, 0, controller_yaw, 0, _cur_yaw, 0);
		 vec_ctrl.yawControlWithYawForce(controller_yaw);
                 cout << " controller_yaw:  " << controller_yaw <<endl;
             };break;
         case Px_Vx_CONTROL_MODE:  //双环控制x
             {
                 robot_x_distance = cos(_cur_yaw/180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw /180.0*pi) * ( _exp_p_y - _cur_p_y );
                 robot_y_distance = (-sin(_cur_yaw/180.0*pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw/180.0*pi) * ( _exp_p_y - _cur_p_y );

                 robot_x_speed = cos(_cur_yaw/180.0*pi)* _cur_v_x + sin(_cur_yaw/180.0*pi)* _cur_v_y;
                 robot_y_speed = (-sin(_cur_yaw/180.0*pi))* _cur_v_x + cos(_cur_yaw/180.0*pi)* _cur_v_y;

                 controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                 controller_v_x = adrc_Vx_ctrl.adrcOutput(controller_p_x, robot_x_speed);
                 vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.doubleClosedLoopControlPx(controller_v_x);
                 cout << " controller_p_x:  " << controller_p_x <<endl;
                 cout << " controller_v_x:  " << controller_v_x <<endl;
             };break;
         case Py_Vy_CONTROL_MODE: // 双环控制y
             {
                 controller_p_y = adrc_Py_ctrl.adrcOutput(_exp_p_y, _cur_p_y);
                 controller_v_y = adrc_Vx_ctrl.adrcOutput(controller_p_y, _cur_v_y);
                 vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.doubleClosedLoopControlPy(controller_v_y);
                 cout << " controller_p_y:  " << controller_p_y <<endl;
                 cout << " controller_v_y:  " << controller_v_y <<endl;
             };break;

         case ALL_CONTROL_MODE_SINGLE_INTEG: // 环全方位控制
             {
                // 根据实际情况切换形状 /////////////////////////////
                if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                       ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) > DECREASE_SPEED_RADIUS  )
                {
                    shape = shape ; // 以前是哪种形状,现在还用哪种形状
                }
                else
                {
                    shape = SHAPE_X; // 转换为x形状
                }


                // 根据形状施加控制 //////////////////////////////////
                if(shape == SHAPE_H_T) // y方向加控制量 位置单环控制
                {
                    // 根据区域重新分配航向角,考虑到 2号区域和4号区域需要转向的问题
                    _exp_yaw -= 90;  // 跟正常航向角差90度
                    if(region == ROBOT_REGION_2)
                    {
                        _exp_yaw = _exp_yaw;  // 2号区域不用动
                    }
                    else if(region == ROBOT_REGION_4)
                    {
                        _exp_yaw += 180;     // 四号区域 加180 加到2号区域那边
                    }
                    if(_exp_yaw > 180)       // 限制在-180 到 180内
                        _exp_yaw -= 360;
                    else if(_exp_yaw < -180)
                        _exp_yaw += 360;

                    // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                    robot_y_distance = (-sin(_cur_yaw / 180.0 * pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw / 180.0 *pi) * ( _exp_p_y - _cur_p_y );
                    // 速度是个向量 不平移 直接旋转

 // 暂时不用全距离
 //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
 //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                    controller_p_x = 0; // x方向没有控制量
                    controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0); // 斜线位移

                    //斜线速度
                    /// DEBUG
                    /// controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                    controller_p_z = 0;
                    controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw); // 航向角调节

                    cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

                    // 解算前的方法
                    // vec_ctrl.yawLocomMode_LR();
                    // vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);// 质点控制法
                    // 解算后的方法
                    ///DEBUG 航向角权重设置为5
                    vec_ctrl.space3DControl(SHAPE_H_T, controller_p_x, controller_p_y, controller_p_z, 5*controller_yaw);
                    ///vec_ctrl.space3DControl(SHAPE_H_T, 0, 0, 0, controller_yaw);
                    ///
                }
                else if(shape == SHAPE_H) //x方向加控制量 位置单环控制
                {

                    if(region == ROBOT_REGION_1)
                        _exp_yaw = _exp_yaw;    // 如果在1号区域,不用动,因为机器人的头就在这个区域
                    else if(region == ROBOT_REGION_3)
                    {
                        _exp_yaw += 180;        // 加到1号区域那边,跟1号区域共用期望角度
                    }
                    if(_exp_yaw > 180)          // 加过辙了 往下扣360
                        _exp_yaw -= 360;
                    else if(_exp_yaw < -180)
                        _exp_yaw += 360;
                    // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                    robot_x_distance = cos(_cur_yaw / 180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw / 180.0*pi) * ( _exp_p_y - _cur_p_y );
                    // 速度是个向量 不平移 直接旋转

 // 暂时不用全距离
 //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
 //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                    // 双环x控制量
                    controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                    //斜线速度
                    controller_p_y = 0;
                    /// DEBUG
                    ///controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                    controller_p_z = 0;
                    ///
                    controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw); // 航向角调节

                    cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
                    // 解算前的方法
                    // vec_ctrl.yawLocomMode_FB();
                    // vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                    // 解算后的方法
                    /// DEBUG  航向角权重设置为5
                    vec_ctrl.space3DControl(SHAPE_H, controller_p_x, controller_p_y, controller_p_z, 5*controller_yaw);
                    //vec_ctrl.space3DControl(SHAPE_H, 0, 0, 0, controller_yaw);
                    ///
                }

                else if( shape == SHAPE_X)  // 位置单环控制
                {
                    // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                    robot_x_distance = cos(_cur_yaw/180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw /180.0*pi) * ( _exp_p_y - _cur_p_y );
                    robot_y_distance = (-sin(_cur_yaw/180.0*pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw/180.0*pi) * ( _exp_p_y - _cur_p_y );
                    // 速度是个向量 不平移 直接旋转
                    // x y 都施加双环控制
                    controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);

                    controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0);
                    ///DEBUG
                    ///controller_p_x = adrc_Px_ctrl.adrcOutput(1, 0);
                    ///controller_p_y = adrc_Py_ctrl.adrcOutput(1, 0);
                    ///
                    // z正常控制
                    //controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2; // 控制深度
                    ///DEBUG
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
                if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) < finish_radius )
                {
                    vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0); //全关
                    is_judge_direction = false;
                    is_running = false;
                    debug_mode = 0; // 强制让调试模式为0 终止运行
                    shape = 0 ;// 强制将形状取消, 不接受控制
                }
                cout << " controller_p_x:  " << controller_p_x <<endl;
                cout << " controller_p_y:  " << controller_p_y <<endl;
                cout << " controller_p_z:  " << controller_p_z <<endl;
                cout << " controller_yaw:  " << controller_yaw <<endl;
                cout << "robot_x_distance  "<< robot_x_distance << endl;
                cout << "robot_y_distance  "<< robot_y_distance << endl;
                cout << "exp_p_x:  "<< _exp_p_x <<"  "<<"cur_p_x:  "<<_cur_p_x <<endl;
                cout << "exp_p_y:  "<< _exp_p_y <<"  "<<"cur_p_y:  "<<_cur_p_y <<endl;
                cout << "exp_yaw:  "<< _exp_yaw <<"  "<<"cur_yaw:  "<<_cur_yaw <<endl;
              };break;

         case ALL_CONTROL_MODE_DOUBLE_INTEG: // 双环全方位控制
             {
                 // 根据实际情况切换形状 /////////////////////////////
                 if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                           ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) > DECREASE_SPEED_RADIUS  )
                 {
                     shape = shape ; // 以前是哪种形状,现在还用哪种形状
                 }
                 else
                 {
                     shape = SHAPE_X; // 转换为x形状
                 }


                 // 根据形状施加控制 //////////////////////////////////
                 if(shape == SHAPE_H_T) // y方向加控制量 双环
                 {
                     // 根据区域重新分配航向角,考虑到 2号区域和4号区域需要转向的问题
                     _exp_yaw -= 90;  // 跟正常航向角差90度
                     if(region == ROBOT_REGION_2)
                     {
                         _exp_yaw = _exp_yaw;  // 2号区域不用动
                     }
                     else if(region == ROBOT_REGION_4)
                     {
                         _exp_yaw += 180;     // 四号区域 加180 加到2号区域那边
                     }
                     if(_exp_yaw > 180)       // 限制在-180 到 180内
                         _exp_yaw -= 360;
                     else if(_exp_yaw < -180)
                         _exp_yaw += 360;

                     // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                     robot_y_distance = -sin(_cur_yaw / 180.0*pi)* ( _exp_p_x - _cur_p_x ) + cos( _cur_yaw / 180.0*pi) * ( _exp_p_y - _cur_p_y );
                     // 速度是个向量 不平移 直接旋转
                     robot_y_speed = (-sin(_cur_yaw / 180.0 * pi))* _cur_v_x + cos(_cur_yaw/180.0 * pi)* _cur_v_y;

     // 暂时不用全距离
     //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
     //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                     controller_v_x = 0; // x方向没有控制量
                     controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0); // 斜线位移

                     //斜线速度
                     controller_v_y = adrc_Vy_ctrl.adrcOutput(controller_p_y, robot_y_speed);

                     controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                     controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw); // 航向角调节

                     cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;
                     // 解算前的方法
                     // vec_ctrl.yawLocomMode_LR();
                     // vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);// 质点控制法
                     // 解算后的方法
                     vec_ctrl.space3DControl(SHAPE_H_T, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
                 }
                 else if(shape == SHAPE_H) //x方向加控制量 双环
                 {
                     if(region == ROBOT_REGION_1)
                         _exp_yaw = _exp_yaw;    // 如果在1号区域,不用动,因为机器人的头就在这个区域
                     else if(region == ROBOT_REGION_3)
                     {
                         _exp_yaw += 180;        // 加到1号区域那边,跟1号区域共用期望角度
                     }
                     if(_exp_yaw > 180)          // 加过辙了 往下扣360
                         _exp_yaw -= 360;
                     else if(_exp_yaw < -180)
                         _exp_yaw += 360;
                     // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                     robot_x_distance = cos(_cur_yaw / 180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw / 180.0*pi) * ( _exp_p_y - _cur_p_y );
                     // 速度是个向量 不平移 直接旋转
                     robot_x_speed = cos(_cur_yaw / 180.0*pi)* _cur_v_x + sin(_cur_yaw / 180.0*pi)* _cur_v_y;

     // 暂时不用全距离
     //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
     //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                     // 双环x控制量
                     controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                     controller_v_x = adrc_Vx_ctrl.adrcOutput(controller_p_x, robot_x_speed);
                     //斜线速度
                     controller_v_y = 0;

                     controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                     controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, _cur_yaw); // 航向角调节

                     cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
                     // 解算前的方法
                     // vec_ctrl.yawLocomMode_FB();
                     // vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                     // 解算后的方法
                     vec_ctrl.space3DControl(SHAPE_H, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
                 }
                 else if( shape == SHAPE_X)
                 {
                     // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                     robot_x_distance = cos(_cur_yaw/180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw /180.0*pi) * ( _exp_p_y - _cur_p_y );
                     robot_y_distance = (-sin(_cur_yaw/180.0*pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw/180.0*pi) * ( _exp_p_y - _cur_p_y );
                     // 速度是个向量 不平移 直接旋转
                     robot_x_speed = cos(_cur_yaw/180.0*pi)* _cur_v_x + sin(_cur_yaw/180.0*pi)* _cur_v_y;
                     robot_y_speed = (-sin(_cur_yaw/180.0*pi))* _cur_v_x + cos(_cur_yaw/180.0*pi)* _cur_v_y;
                     // x y 都施加双环控制
                     controller_p_x = adrc_Px_ctrl.adrcOutput(robot_x_distance, 0);
                     controller_v_x = adrc_Vx_ctrl.adrcOutput(controller_p_x, robot_x_speed);

                     controller_p_y = adrc_Py_ctrl.adrcOutput(robot_y_distance, 0);
                     controller_v_y = adrc_Vy_ctrl.adrcOutput(controller_p_y, robot_y_speed);

                     // z正常控制
                     controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z,_cur_p_z) + 4.2; // 控制深度
                     controller_yaw = 0; // x型模式无法调节航向角

                     cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;
                     // 解算前的方法
                     // vec_ctrl.yawLocomMode_X();
                     // vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                     // 解算后的方法
                     vec_ctrl.space3DControl(SHAPE_X, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
                 }

                 // 判断终止条件 /////////////////////////////////////////////////////
                 if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                           ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) <= finish_radius )
                 {
                     // vec_ctrl.motorforce(0,0,0,0,0,0,0);
                     vec_ctrl.thrusterforce2speed(0, 0, 0, 0); //全关

                     is_judge_direction = false;
                     is_running = false;
                     debug_mode = 0; // 强制让调试模式为0 终止运行
                     shape = 0 ;// 强制将形状取消, 不接受控制
                 }
                 cout << "****double_integ####" <<endl;
                 cout << " controller_v_x:  " << controller_v_x <<endl;
                 cout << " controller_v_y:  " << controller_v_y <<endl;
                 cout << " controller_p_z:  " << controller_p_z <<endl;
                 cout << " controller_yaw:  " << controller_yaw <<endl;
                 cout << "robot_x_distance  "<< robot_x_distance << endl;
                 cout << "robot_y_distance  "<< robot_y_distance << endl;
                 cout << "exp_p_x:  "<< _exp_p_x <<"  "<<"cur_p_x:  "<<_cur_p_x <<endl;
                 cout << "exp_p_y:  "<< _exp_p_y <<"  "<<"cur_p_y:  "<<_cur_p_y <<endl;
                 cout << "exp_yaw:  "<< _exp_yaw <<"  "<<"cur_yaw:  "<<_cur_yaw <<endl;
             };break;
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
                 if ( delta_yaw > 180 )
                     cur_yaw_control += 360;
                 else if( delta_yaw <= -180 )
                     cur_yaw_control += -360;

                 controller_p_z = adrc_Pz_ctrl.adrcOutput(_exp_p_z, _cur_p_z) + 4.2; // 控制深度
                 controller_yaw = adrc_yaw_ctrl.adrcOutput(_exp_yaw, cur_yaw_control); // 航向角调节

                 cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
                 cout << "  controller_yaw " << controller_yaw << endl;
                 vec_ctrl.yawLocomMode_FB();
                 vec_ctrl.motorforce(_exp_p_x, 0, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
                 cout << " exp_depth:  " << _exp_p_z << ", _cur_depth:  " << _cur_p_z << endl;
                 cout << " exp_yaw:  " << _exp_yaw << ", _cur_yaw:  " << _cur_yaw << ", cur_yaw_control: " << cur_yaw_control << endl;
                 cout << " Detpth controller output:  " << controller_p_z <<endl;
                 cout << " Yaw controller output:  " << controller_yaw <<endl;
            };break;
         default:
             {
                 cout << "there is no this control mode !!! " <<endl;
             };break;
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
         switch(_mode)
         {
         case 0:
             {
                 cout << " now it has stop !!! " << endl;
             };break;
         case Px_CONTROL_MODE:  // 控x方向位置
             {
                 robot_x_distance = cos(_cur_yaw / 180.0 * pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw/180*pi ) * ( _exp_p_y - _cur_p_y );
                 controller_p_x = ss_Px_ctrl.CalSSCv2output( robot_x_distance, 0);
                 vec_ctrl.motorforce(controller_p_x, 0, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.PxControlWithPxForce(controller_p_x);
                 cout << " controller_p_x:  " << controller_p_x <<endl;
                 cout << "robot_x_distance" << robot_x_distance << endl;
                 cout << "_cur_yaw" << _cur_yaw <<endl;
             };break;
         case Vx_CONTROL_MODE:  // 控x方向速度
             {
                 controller_v_x = ss_Vx_ctrl.CalSSCv2output(_exp_v_x, _cur_v_x);
                 vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.VxControlWithVxForce(controller_v_x);
                 cout << " controller_v_x:  " << controller_v_x <<endl;
             };break;

         case Py_CONTROL_MODE: // 控y方向位置
             {

                 controller_p_y = ss_Py_ctrl.CalSSCv2output(_exp_p_y, _cur_p_y);
                 vec_ctrl.motorforce(0, controller_p_y, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.PyControlWithPyForce(controller_p_y);
                 cout << " controller_p_y:  " << controller_p_y <<endl;
             };break;
         case Vy_CONTROL_MODE: // 控y方向速度
             {
                 controller_v_y = ss_Vy_ctrl.CalSSCv2output(_exp_v_y, _cur_v_y);
                 vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.VyControlWithVyForce(controller_v_y);
                 cout << " controller_v_y:  " << controller_v_y <<endl;
             };break;

         case Pz_CONTROL_MODE: // 控z方向位置  0.7 为抵消浮力的参考量
             {
                 controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z, _cur_p_z) - 0.7;
                 vec_ctrl.motorforce(0, 0, controller_p_z, 0, 0, _cur_yaw, 0);
		 vec_ctrl.depthControlWithPzForce(controller_p_z);
                 cout << " controller_p_z:  " << controller_p_z <<endl;
             };break;
         case YAW_CONTROL_MODE: // 控航向角
             {
                 controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw);
                 vec_ctrl.motorforce(0, 0, 0, controller_yaw, 0, _cur_yaw, 0);
		 vec_ctrl.yawControlWithYawForce(controller_yaw);
                 cout << " controller_yaw:  " << controller_yaw <<endl;
             };break;
         case Px_Vx_CONTROL_MODE:  //双环控制x
             {
                 robot_x_distance = cos(_cur_yaw/180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw /180.0*pi) * ( _exp_p_y - _cur_p_y );
                 robot_y_distance = (-sin(_cur_yaw/180.0*pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw/180.0*pi) * ( _exp_p_y - _cur_p_y );

                 robot_x_speed = cos(_cur_yaw/180.0*pi)* _cur_v_x + sin(_cur_yaw/180.0*pi)* _cur_v_y;
                 robot_y_speed = (-sin(_cur_yaw/180.0*pi))* _cur_v_x + cos(_cur_yaw/180.0*pi)* _cur_v_y;

                 controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                 controller_v_x = ss_Vx_ctrl.CalSSCv2output(controller_p_x, robot_x_speed);
                 vec_ctrl.motorforce(controller_v_x, 0, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.doubleClosedLoopControlPx(controller_v_x);
                 cout << " controller_p_x:  " << controller_p_x <<endl;
                 cout << " controller_v_x:  " << controller_v_x <<endl;
             };break;
         case Py_Vy_CONTROL_MODE: // 双环控制y
             {
                 controller_p_y = ss_Py_ctrl.CalSSCv2output(_exp_p_y, _cur_p_y);
                 controller_v_y = ss_Vx_ctrl.CalSSCv2output(controller_p_y, _cur_v_y);
                 vec_ctrl.motorforce(0, controller_v_y, 0, 0, 0, _cur_yaw, 0);
		 vec_ctrl.doubleClosedLoopControlPy(controller_v_y);
                 cout << " controller_p_y:  " << controller_p_y <<endl;
                 cout << " controller_v_y:  " << controller_v_y <<endl;
             };break;

         case ALL_CONTROL_MODE_SINGLE_INTEG: // 环全方位控制
             {
                // 根据实际情况切换形状 /////////////////////////////
                if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                       ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) > DECREASE_SPEED_RADIUS  )
                {
                    shape = shape ; // 以前是哪种形状,现在还用哪种形状
                }
                else
                {
                    shape = SHAPE_X; // 转换为x形状
                }


                // 根据形状施加控制 //////////////////////////////////
                if(shape == SHAPE_H_T) // y方向加控制量 位置单环控制
                {
                    _exp_yaw -= 90;
                    if(_exp_yaw < -180)
                    {
                        _exp_yaw += 360;
                    }
                    // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                    robot_y_distance = (-sin(_cur_yaw / 180.0 * pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw / 180.0 *pi) * ( _exp_p_y - _cur_p_y );
                    // 速度是个向量 不平移 直接旋转

 // 暂时不用全距离
 //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
 //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                    controller_p_x = 0; // x方向没有控制量
                    controller_p_y = ss_Py_ctrl.CalSSCv2output( robot_y_distance, 0); // 斜线位移

                    //斜线速度
                    controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z,_cur_p_z); // 控制深度
                    controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                    cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

                    vec_ctrl.yawLocomMode_LR();
                    vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
		    vec_ctrl.space3DControl(SHAPE_H_T, controller_p_x, controller_p_y, controller_p_z, controller_yaw);
                }
                else if(shape == SHAPE_H) //x方向加控制量 位置单环控制
                {
                    // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                    robot_x_distance = cos(_cur_yaw / 180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw / 180.0*pi) * ( _exp_p_y - _cur_p_y );
                    // 速度是个向量 不平移 直接旋转

 // 暂时不用全距离
 //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
 //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                    // 双环x控制量
                    controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                    //斜线速度
                    controller_p_y = 0;

                    controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z,_cur_p_z); // 控制深度
                    controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                    cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

                    vec_ctrl.yawLocomMode_FB();
                    vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
		    vec_ctrl.space3DControl(SHAPE_H, controller_p_x, controller_p_y, controller_p_z, controller_yaw);
                }
                else if( shape == SHAPE_X)  // 位置单环控制
                {
                    // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                    robot_x_distance = cos(_cur_yaw/180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw /180.0*pi) * ( _exp_p_y - _cur_p_y );
                    robot_y_distance = (-sin(_cur_yaw/180.0*pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw/180.0*pi) * ( _exp_p_y - _cur_p_y );
                    // 速度是个向量 不平移 直接旋转
                    // x y 都施加双环控制
                    controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);

                    controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0);

                    // z正常控制
                    controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z,_cur_p_z); // 控制深度
                    controller_yaw = 0; // x型模式无法调节航向角

                    cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

                    vec_ctrl.yawLocomMode_X();
                    vec_ctrl.motorforce(controller_p_x,controller_p_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
		    vec_ctrl.space3DControl(SHAPE_X, controller_p_x, controller_p_y, controller_p_z, controller_yaw );
                }

                // 判断终止条件 /////////////////////////////////////////////////////
                if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) < finish_radius )
                {
                    vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0); //全关
                    is_judge_direction = false;
                    is_running = false;
                    debug_mode = 0; // 强制让调试模式为0 终止运行
                    shape = 0 ;// 强制将形状取消, 不接受控制
                }

                cout << " controller_v_x:  " << controller_v_x <<endl;
                cout << " controller_v_y:  " << controller_v_y <<endl;
                cout << " controller_p_z:  " << controller_p_z <<endl;
                cout << " controller_yaw:  " << controller_yaw <<endl;
                cout << "robot_x_distance  "<< robot_x_distance << endl;
                cout << "robot_y_distance  "<< robot_y_distance << endl;
                cout << "cur_p_x:  "<< _cur_p_x <<endl;
                cout << "cur_p_y:  "<< _cur_p_y <<endl;
              };break;

         case ALL_CONTROL_MODE_DOUBLE_INTEG: // 双环全方位控制
             {
                 // 根据实际情况切换形状 /////////////////////////////
                 if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                           ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) > DECREASE_SPEED_RADIUS  )
                 {
                     shape = shape ; // 以前是哪种形状,现在还用哪种形状
                 }
                 else
                 {
                     shape = SHAPE_X; // 转换为x形状
                 }


                 // 根据形状施加控制 //////////////////////////////////
                 if(shape == SHAPE_H_T) // y方向加控制量 双环
                 {
                     _exp_yaw -= 90;
                     if(_exp_yaw < -180)
                     {
                         _exp_yaw += 360;
                     }
                     // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                     robot_y_distance = -sin(_cur_yaw / 180.0*pi)* ( _exp_p_x - _cur_p_x ) + cos( _cur_yaw / 180.0*pi) * ( _exp_p_y - _cur_p_y );
                     // 速度是个向量 不平移 直接旋转
                     robot_y_speed = (-sin(_cur_yaw / 180.0 * pi))* _cur_v_x + cos(_cur_yaw/180.0 * pi)* _cur_v_y;

     // 暂时不用全距离
     //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
     //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                     controller_v_x = 0; // x方向没有控制量
                     controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0); // 斜线位移

                     //斜线速度
                     controller_v_y = ss_Vy_ctrl.CalSSCv2output(controller_p_y, robot_y_speed);

                     controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z,_cur_p_z); // 控制深度
                     controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                     cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

                     vec_ctrl.yawLocomMode_LR();
                     vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
		     vec_ctrl.space3DControl(SHAPE_H_T, controller_v_x, controller_v_y, controller_p_z, controller_yaw);

                 }
                 else if(shape == SHAPE_H) //x方向加控制量 双环
                 {
                     // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                     robot_x_distance = cos(_cur_yaw / 180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw / 180.0*pi) * ( _exp_p_y - _cur_p_y );
                     // 速度是个向量 不平移 直接旋转
                     robot_x_speed = cos(_cur_yaw / 180.0*pi)* _cur_v_x + sin(_cur_yaw / 180.0*pi)* _cur_v_y;

     // 暂时不用全距离
     //                robot_whole_distance = sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
     //                        ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) );
                     // 双环x控制量
                     controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                     controller_v_x = ss_Vx_ctrl.CalSSCv2output(controller_p_x, robot_x_speed);
                     //斜线速度
                     controller_v_y = 0;

                     controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z,_cur_p_z); // 控制深度
                     controller_yaw = ss_yaw_ctrl.CalSSCv2output(_exp_yaw, _cur_yaw); // 航向角调节

                     cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

                     vec_ctrl.yawLocomMode_FB();
                     vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
		     vec_ctrl.space3DControl(SHAPE_H, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
                 }
                 else if( shape == SHAPE_X)
                 {
                     // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
                     robot_x_distance = cos(_cur_yaw/180.0*pi)* ( _exp_p_x - _cur_p_x ) + sin( _cur_yaw /180.0*pi) * ( _exp_p_y - _cur_p_y );
                     robot_y_distance = (-sin(_cur_yaw/180.0*pi))* ( _exp_p_x - _cur_p_x ) + cos(_cur_yaw/180.0*pi) * ( _exp_p_y - _cur_p_y );
                     // 速度是个向量 不平移 直接旋转
                     robot_x_speed = cos(_cur_yaw/180.0*pi)* _cur_v_x + sin(_cur_yaw/180.0*pi)* _cur_v_y;
                     robot_y_speed = (-sin(_cur_yaw/180.0*pi))* _cur_v_x + cos(_cur_yaw/180.0*pi)* _cur_v_y;
                     // x y 都施加双环控制
                     controller_p_x = ss_Px_ctrl.CalSSCv2output(robot_x_distance, 0);
                     controller_v_x = ss_Vx_ctrl.CalSSCv2output(controller_p_x, robot_x_speed);

                     controller_p_y = ss_Py_ctrl.CalSSCv2output(robot_y_distance, 0);
                     controller_v_y = ss_Vy_ctrl.CalSSCv2output(controller_p_y, robot_y_speed);

                     // z正常控制
                     controller_p_z = ss_Pz_ctrl.CalSSCv2output(_exp_p_z,_cur_p_z); // 控制深度
                     controller_yaw = 0; // x型模式无法调节航向角

                     cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

                     vec_ctrl.yawLocomMode_X();
                     vec_ctrl.motorforce(controller_v_x,controller_v_y, controller_p_z, controller_yaw, 0, _cur_yaw, 0);
		     vec_ctrl.space3DControl(SHAPE_X, controller_v_x, controller_v_y, controller_p_z, controller_yaw);
                 }

                 // 判断终止条件 /////////////////////////////////////////////////////
                 if( sqrt( ( _exp_p_x - _cur_p_x )* ( _exp_p_x - _cur_p_x ) +
                           ( _exp_p_y - _cur_p_y )* ( _exp_p_y - _cur_p_y ) ) <= finish_radius )
                 {
                     vec_ctrl.motorforce(0, 0, 0, 0, 0, 0, 0); //全关
                     is_judge_direction = false;
                     is_running = false;
                     debug_mode = 0; // 强制让调试模式为0 终止运行
                     shape = 0 ;// 强制将形状取消, 不接受控制
                 }

                 cout << " controller_v_x:  " << controller_v_x <<endl;
                 cout << " controller_v_y:  " << controller_v_y <<endl;
                 cout << " controller_p_z:  " << controller_p_z <<endl;
                 cout << " controller_yaw:  " << controller_yaw <<endl;
                 cout << "robot_x_distance  "<< robot_x_distance << endl;
                 cout << "robot_y_distance  "<< robot_y_distance << endl;
                 cout << "exp_p_x:  "<< _exp_p_x <<"  "<<"cur_p_x:  "<<_cur_p_x <<endl;
                 cout << "exp_p_y:  "<< _exp_p_y <<"  "<<"cur_p_y:  "<<_cur_p_y <<endl;
                 cout << "exp_yaw:  "<< _exp_yaw <<"  "<<"cur_yaw:  "<<_cur_yaw <<endl;
             };break;
         default:
             {
                 cout << "there is no this control mode !!! " <<endl;
             };break;
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
    
    //hcy:gazebo原程序使用
    // 注意: 输入的期望速度,是机器人坐标系下的,非全局坐标系下的
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

        switch(_shape)
        {


        case SHAPE_H:{
            // H 型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            /// DEBUG
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            //controller_x_speed = 0;
            // 速度y控制量 H型模式下 机器人的y方向无法推进
            controller_y_speed = 0;

            controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed,robot_yaw_speed); // 航向角调节
//            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度
          //  controller_yaw_speed = 0;
            controller_z_speed = 0;
            ///
            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

            speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
                              
                                         

        } ;break;
        case SHAPE_H_T:{
            // 工型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量 工型模式下 机器人的x方向无法推进
            controller_x_speed = 0;
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);

            controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed,robot_yaw_speed); // 航向角调节
            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

            speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);

        }; break;
        case SHAPE_X:{
            // X型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);
            // 航向角速度为0
            controller_yaw_speed = 0; // 航向角调节
            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度

            cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            //speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
            //                              controller_z_speed, controller_yaw_speed);
            speed_vec_ctrl.space3DControl(SHAPE_X, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
        } ; break;
        }

        cout << " controller_x_speed:  " << controller_x_speed <<endl;
        cout << " controller_y_speed:  " << controller_y_speed <<endl;
        cout << " controller_yaw_speed:  " << controller_yaw_speed <<endl;
        cout << " controller_z_speed:  " << controller_z_speed <<endl;
        cout << "_exp_v_x:  "<< robot_exp_x_speed <<"  "<<"_cur_v_x:  "<<robot_x_speed <<endl;
        cout << "_exp_v_y:  "<< robot_exp_y_speed <<"  "<<"_cur_v_y:  "<<robot_y_speed <<endl;
        cout << "_exp_v_yaw:  "<< robot_exp_yaw_speed <<"  "<<"_cur_v_yaw:  "<<robot_yaw_speed <<endl;
        cout << "_exp_v_z:  " << robot_exp_z_speed << "  " << "_cur_v_z:  " << robot_z_speed <<endl;
    }


    //hcy:gazebo原程序使用
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
        if((_exp_v_yaw < 0.000001) && (_exp_v_yaw > -0.000001))
        {
            __control_v_yaw_flag = false;
        }

        cout << "__control_v_yaw_flag: "<< __control_v_yaw_flag <<endl;

        switch(_shape)
        {


        case SHAPE_H:{
            // H 型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            /// DEBUG
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
//            controller_x_speed = 0;
            // 速度y控制量 H型模式下 机器人的y方向无法推进
            controller_y_speed = 0;

            if(__control_v_yaw_flag == true)
            {
                controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed,robot_yaw_speed); // 航向角调节
               
            }else
            {
                controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);
                
            }
//            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度
//            controller_yaw_speed = 0;
            controller_z_speed = 0;
            ///
            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;

            // 解算的输入有变化
            if(__control_v_yaw_flag == true)
            {
                speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw_speed);
            }
            else
            {
                speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw);
            }

        } ;break;
        case SHAPE_H_T:{
            // 工型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量 工型模式下 机器人的x方向无法推进
            controller_x_speed = 0;
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);

            if(__control_v_yaw_flag == true)
            {
                controller_yaw_speed = adrc_speed_yaw_ctrl.adrcOutput(robot_exp_yaw_speed,robot_yaw_speed); // 航向角调节
            }else
            {
                controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw);
            }

            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T " << endl;

            // 解算的输入有变化
            if(__control_v_yaw_flag == true)
            {
                speed_vec_ctrl.space3DControl(SHAPE_H_T, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw_speed);
            }
            else
            {
                speed_vec_ctrl.space3DControl(SHAPE_H_T, controller_x_speed, controller_y_speed,
                                              controller_z_speed, controller_yaw);
            }

        }; break;
        case SHAPE_X:{
            // X型控制, 上层控制器会发送 期望的前进速度 和 航向角速度
            // 速度x控制量
            controller_x_speed = adrc_speed_x_ctrl.adrcOutput(robot_exp_x_speed, robot_x_speed);
            // 速度y控制量
            controller_y_speed = adrc_speed_y_ctrl.adrcOutput(robot_exp_y_speed, robot_y_speed);
            // 航向角速度为0
            controller_yaw_speed = 0; // 航向角调节
            controller_z_speed = adrc_speed_z_ctrl.adrcOutput(robot_exp_z_speed, robot_z_speed);         // 控制深度

            cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            speed_vec_ctrl.space3DControl(SHAPE_H, controller_x_speed, controller_y_speed,
                                          controller_z_speed, controller_yaw_speed);
        } ; break;
        }
       
        
        cout << " controller_x_speed:  " << controller_x_speed <<endl;
        cout << " controller_y_speed:  " << controller_y_speed <<endl;
        cout << " controller_yaw_speed:  " << controller_yaw_speed <<endl;
        cout << " controller_z_speed:  " << controller_z_speed <<endl;
        cout << " controller_yaw:  " << controller_yaw << endl;
        cout << "_exp_v_x:  "<< robot_exp_x_speed <<"  "<<"_cur_v_x:  "<<robot_x_speed <<endl;
        cout << "_exp_v_y:  "<< robot_exp_y_speed <<"  "<<"_cur_v_y:  "<<robot_y_speed <<endl;
        cout << "_exp_v_yaw:  "<< robot_exp_yaw_speed <<"  "<<"_cur_v_yaw:  "<<robot_yaw_speed <<endl;
        cout << "_exp_v_z:  " << robot_exp_z_speed << "  " << "_cur_v_z:  " << robot_z_speed <<endl;
        cout << "_exp_yaw:  " << robot_exp_yaw << "  " << "_cur_yaw:  " << robot_yaw <<endl;
    }







    //hcy
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


    //hcy 
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

    //hcy
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


        switch(_shape)
        {
        case SHAPE_H:{

            // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
            robot_x_distance = cos(robot_yaw / 180.0*pi)* ( robot_exp_x_position - robot_x_position ) + sin( robot_yaw / 180.0*pi) * ( robot_exp_y_position - robot_y_position );

            // 单环控制量
            controller_x_position = adrc_x_ctrl.adrcOutput(robot_x_distance, 0);
            //斜线速度
            controller_y_position = 0;

            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position,robot_z_position); // 控制深度
            controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw); // 航向角调节

            // 向右转弯迟钝，增大向右转弯能量
            if(controller_yaw  < 0)
            {
                controller_yaw = 1.3* controller_yaw;
            }
            else
	    {   controller_yaw = 1.1* controller_yaw;
	    }
            cout << "  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H  SHAPE_H " << endl;
            /// DEBUG
            // 航向角权重设置为2 更看重转弯
            vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, 2*controller_yaw);
            ///
        };break;

        case SHAPE_H_T:{


            // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
            robot_y_distance = (-sin(robot_yaw / 180.0 * pi))* ( robot_exp_x_position - robot_x_position ) + cos(robot_yaw / 180.0 *pi) * ( robot_exp_y_position - robot_y_position );
            // 速度是个向量 不平移 直接旋转

            controller_x_position = 0; // x方向没有控制量
            controller_y_position = adrc_y_ctrl.adrcOutput(robot_y_distance, 0); // 斜线位移

            //斜线速度
            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
            controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw); // 航向角调节

            cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  " << endl;
            /// DEBUG
            // 航向角权重设置为5 更看重转弯的快速性质
            vec_ctrl.space3DControl(SHAPE_H_T, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
            ///

        };break;

        case SHAPE_X:{

            // 拉旋转变换 转移成机器人坐标系下的距离 先平移 后旋转
            robot_x_distance = cos(robot_yaw/180.0*pi)* ( robot_exp_x_position - robot_x_position ) + sin( robot_yaw /180.0*pi) * ( robot_exp_y_position - robot_y_position );
            robot_y_distance = (-sin(robot_yaw/180.0*pi))* ( robot_exp_x_position - robot_x_position ) + cos(robot_yaw/180.0*pi) * ( robot_exp_y_position - robot_y_position );
            // 速度是个向量 不平移 直接旋转
            // x y 都施加双环控制
            controller_x_position = adrc_x_ctrl.adrcOutput(robot_x_distance, 0);

            controller_y_position = adrc_y_ctrl.adrcOutput(robot_y_distance, 0);

            // z正常控制
            controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position,robot_z_position); // 控制深度
            controller_yaw = 0; // x型模式无法调节航向角

            cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

            vec_ctrl.space3DControl(SHAPE_X, controller_x_position, controller_y_position, controller_z_position, controller_yaw );

        };break;
        }




        cout << " controller_p_x:  " << controller_x_position <<endl;
        cout << " controller_p_y:  " << controller_y_position <<endl;
        cout << " controller_p_z:  " << controller_z_position <<endl;
        cout << " controller_yaw:  " << controller_yaw <<endl;
        cout << "robot_x_distance  "<< robot_x_distance << endl;
        cout << "robot_y_distance  "<< robot_y_distance << endl;
        cout << "exp_p_x:  "<< robot_exp_x_position <<"  "<<"cur_p_x:  "<<robot_x_position <<endl;
        cout << "exp_p_y:  "<< robot_exp_y_position <<"  "<<"cur_p_y:  "<<robot_y_position <<endl;
        cout << "exp_yaw:  "<< robot_exp_yaw <<"  "<<"cur_yaw:  "<<robot_yaw <<endl;
        cout << "exp_p_z:  " << robot_exp_z_position << "  " << "cur_p_z:  " << robot_z_position <<endl;

    }


//hcy:（重要）视觉、深度、航向角用到
    void C_Combine3DPositionControl_v2::adrcSelectPositionControllerAndOutput(int _shape,
                                                                         float _exp_p_x, float _exp_p_y,
                                                                         float _exp_yaw, float _exp_p_z,
                                                                         float _cur_p_x, float _cur_p_y,
                                                                         float _cur_p_yaw, float _cur_p_z)
   {

       // 创建了一个临时的坐标系，这个坐标系的X轴与机器人当前的前进方向对齐，原点是世界坐标系的原点，叫新全局坐标系
       // 全局的目标点投影到这个临时坐标系上，得到 (robot_exp_x_position, robot_exp_y_position)
       // 机器人当前的全局位置也投影到这个同一个临时坐标系上，得到 (robot_x_position, robot_y_position)
       //正确逻辑是传入全局坐标下目标点的位姿和当前机器人在全局下的位姿，然后通过坐标转化到新全局坐标下
       //按照我docking aruco码传入目标点位姿是aruco码相对于机体的，而当前机体的x、y、yaw设置为0，在数学逻辑上进行以下代码是通的


       //     cout << "**************************4.统一到新全局坐标系下**************************" << endl;
       //  目标点的角度和深度
       robot_exp_yaw = _exp_yaw;//hy:根据固定的规划点和机器人在新全局坐标系下的坐标算出来的yaw，
                                //机器人在规划点左侧时为负，右侧时为正  //e_yaw                                                 
       robot_exp_z_position = _exp_p_z;
       // 机器人当前的朝向和深度
       robot_yaw = _cur_p_yaw;//hy:即cam_yaw，视觉得出的yaw    //c_yaw
       robot_z_position = _cur_p_z;

        // 使用旋转矩阵
       // 全局期望点在这个新中间坐标系下的坐标
       robot_exp_x_position = cos(robot_yaw / 180.0*pi) * _exp_p_x + sin( robot_yaw / 180.0*pi) * _exp_p_y;
       robot_exp_y_position = -sin(robot_yaw / 180.0*pi) * _exp_p_x + cos( robot_yaw / 180.0*pi) * _exp_p_y;

       // 机器人当前的全局位置在这个新中间坐标系下的坐标
       robot_x_position = cos(robot_yaw / 180.0*pi) * _cur_p_x + sin( robot_yaw / 180.0*pi) * _cur_p_y;
       robot_y_position = -sin(robot_yaw / 180.0*pi) * _cur_p_x + cos( robot_yaw / 180.0*pi) * _cur_p_y;


       switch(_shape)
       {
       case SHAPE_H:{


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

            //hcy
            // 单环控制量
           controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

           //斜线速度
           controller_y_position = 0;
           
           controller_z_position = adrc_Pz_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度

           //    robot_exp_yaw = robot_exp_yaw/57.3;//hcy
           //    robot_yaw = robot_yaw/57.3;
           robot_exp_yaw = robot_exp_yaw/100;//hcy
           robot_yaw = robot_yaw/100;

           controller_yaw = adrc_Pyaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw); // 航向角调节
//****************************************原来程序*********************************************** */
           // 向右转弯迟钝，增大向右转弯能量
           // 向左转弯迟钝，增大向左转弯能量
        //    if(controller_yaw  < 0)//hy:现限制在-1.2~1.2
        //    {
        //        controller_yaw = 0.5* controller_yaw;//hcy:原始值1.3
        //    }
        //    else
        //    {
        //        controller_yaw = 0.7* controller_yaw;//hcy:原始值1.1
        //    }
           
        //     //hcy:通过航向角设置前进方向的力
        //     if(controller_yaw > 0 && _exp_p_x >_cur_p_x)
        //     {
        //         controller_x_position =0.2*(2 - controller_yaw);
        //         //controller_x_position =0.6 - 0.5*controller_yaw;
        //     }
        //     else if(controller_yaw < 0 && _exp_p_x > _cur_p_x)
        //     {
        //         controller_x_position =0.2*(2 + controller_yaw);
        //         //controller_x_position =0.6 + 0.5*controller_yaw;
        //     }
//*************************************改小控制量********************************* */
           // 调整控制系数
           if (controller_yaw < 0)
           {
               controller_yaw = 0.3 * controller_yaw; // 减小向右的调整力度
           }
           else
           {
               controller_yaw = 0.5 * controller_yaw; // 减小向左的调整力度
           }

           // 限制控制量范围
           if (controller_yaw > 1.0)
           {
               controller_yaw = 1.0;
           }
           else if (controller_yaw < -1.0)
           {
               controller_yaw = -1.0;
           }

           // 减小对 X 方向控制量的修正
           if (controller_yaw > 0 && _exp_p_x > _cur_p_x)
           {
               controller_x_position = 0.1 * (2 - controller_yaw); // 减小比例
           }
           else if (controller_yaw < 0 && _exp_p_x > _cur_p_x)
           {
               controller_x_position = 0.1 * (2 + controller_yaw); // 减小比例
           }
 //*************************************改小控制量********************************* */
           cout << "**************************SHAPE_H**************************" << endl;
           /// DEBUG
           // 航向角权重设置为2 更看重转弯
           //    vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, 2*controller_yaw);
           vec_ctrl.space3DControl(SHAPE_H, controller_x_position, controller_y_position, controller_z_position, controller_yaw); // hcy

           ///

       };break;

       case SHAPE_H_T:{

           controller_x_position = 0; // x方向没有控制量
           controller_y_position = adrc_y_ctrl.adrcOutput(robot_exp_y_position, robot_y_position); // 斜线位移

           //斜线速度
           controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
           controller_yaw = adrc_yaw_ctrl.adrcOutput(robot_exp_yaw, robot_yaw); // 航向角调节

           cout << "  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  SHAPE_H_T  " << endl;
           /// DEBUG
           // 航向角权重设置为1 更看重转弯的快速性质
           vec_ctrl.space3DControl(SHAPE_H_T, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
           ///

       };break;

       case SHAPE_X:{

        //    // x y
        //    controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

        //    controller_y_position = adrc_y_ctrl.adrcOutput(robot_exp_y_position, robot_y_position);

        //    // z正常控制
        //    controller_z_position = adrc_z_ctrl.adrcOutput(robot_exp_z_position,robot_z_position); // 控制深度

        //    controller_yaw = 0; // x型模式无法调节航向角

        //    cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;

        //    vec_ctrl.space3DControl(SHAPE_X, controller_x_position, controller_y_position, controller_z_position, controller_yaw );

            //hcy
            // x y
        
        controller_x_position = adrc_x_ctrl.adrcOutput(robot_exp_x_position, robot_x_position);

        controller_y_position = adrc_y_ctrl.adrcOutput(robot_exp_y_position, robot_y_position);

        // z正常控制
        controller_z_position = adrc_Pz_ctrl.adrcOutput(robot_exp_z_position, robot_z_position); // 控制深度
        
        //vec_ctrl.mcDepthPID.initPID(0.55, 0.02, 0.02);//PID初始化用
        //controller_z_position = vec_ctrl.mcDepthPID.pidNewCalc(robot_exp_z_position - robot_z_position);

        controller_x_position = 0.025 * controller_x_position;                                     // ljl减慢速度
        controller_y_position = 0.025 * controller_y_position;                                     // ljl减慢速度

        controller_z_position = 1 * controller_z_position;//ljl减慢速度

        controller_yaw = 0; // x型模式无法调节航向角

        //    cout << "  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X  SHAPE_X " << endl;
        cout << "**************************SHAPE_X**************************" << endl;

        vec_ctrl.space3DControl(SHAPE_X, controller_x_position, controller_y_position, controller_z_position, controller_yaw);
       };break;
       }

       cout << "**************************控制量******************************" << endl;
       cout << " controller_p_x:  " << controller_x_position <<endl;
       cout << " controller_p_y:  " << controller_y_position <<endl;
       cout << " controller_p_z:  " << controller_z_position <<endl;
       cout << " controller_yaw:  " << controller_yaw <<endl;
       cout << "***********************************************************************" << endl;
       cout << "exp_p_x:  "<< robot_exp_x_position <<"  "<<"cur_p_x:  "<<robot_x_position <<endl;
       cout << "exp_p_y:  "<< robot_exp_y_position <<"  "<<"cur_p_y:  "<<robot_y_position <<endl;
       cout << "exp_yaw:  "<< robot_exp_yaw <<"  "<<"cur_yaw:  "<<robot_yaw <<endl;
       cout << "exp_p_z:  " << robot_exp_z_position << "  " << "cur_p_z:  " << robot_z_position <<endl;
   }















} // namespace 括号
