#include "../../include/asrobot/f_single_robot_control.h"

#define PUBLISH_SWITCH 1   // 发布开关
#define SUBSCRIBE_SWITCH 1 // 接收开关
#define TIMER_SWITCH 1     // 定时器开关

#define CAMERA8_MODE 0      // camera8-based formation mode
#define QUADROTOR_MODE 0   // Quadrotor-based formation mode//hcy:原1
#define MANUAL_INPUT_MODE 0 // manual input mode, this is for adjusting parameters.
#define MPC_mode 0          // manual input mode, this is for adjusting parameters.
#define DEBUG_SPEED 0

#define     DEBUG_YAW           1      // hcy
#define     DEBUG_Depth         1    //hcy
#define     AUTO_CTRL 0   // hcy:若采用手动输入期望值的方式控制机器人位姿，则该标志位置0
#define     DEBUG_H_3D_POS_V2   0//hcy:docking和tracking用的模式
#define     DEBUG_H_3D_POS_V3   0//ljl

#define     DEBUG_MOTION 1 // ljl

bool speed_debug = false;
bool save_data_mode = 0;
int HH = 1;
ros::Time stage3_start_time;
float stage3_initial_cam_exp_p_z;


bool yaw_debug = false;  // hcy
bool depth_debug = false;//hcy


bool h_3d_pos_debug = false;//hcy
bool h_3d_pos_debug_v2 = false; //hcy

bool stage1_horizontal_align = false;
bool stage2_descend_to_near = false;
bool stage3_slow_final_descend = false;
/***************************************************************************************/
/*                                                                                     */
/*                                 全局发布变量                                          */
/*                                                                                     */
/***************************************************************************************/
#if PUBLISH_SWITCH

ros::Publisher pub_pwm;
ros::Publisher pub_status;
//ros::Publisher mc_pub_debug; // 发布单体机器人各种信息 如位置，速度，航向角等

ros::NodeHandle *extern_n;
ros::Publisher *extern_pub_pwm;
ros::Publisher *extern_pub_status;

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 全局消息变量（发布）                                    */
/*                                                                                     */
/***************************************************************************************/
#if PUBLISH_SWITCH

asrobot::pwm pwm_msg; // 输出pwm信息
asrobot::status key;  // 输出舵机开关
asrobot::mc motion_debug;   // 包含该机器人的位置，速度，航向等

#endif

/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 全局订阅变量                                          */
/*                                                                                     */
/***************************************************************************************/
#if SUBSCRIBE_SWITCH

ros::Subscriber sub_depth;
ros::Subscriber sub_cmd;
ros::Subscriber sub_trajectory;
ros::Subscriber sub_multiRobotPosition;
#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 定时器                                               */
/*                                                                                     */
/***************************************************************************************/
#if TIMER_SWITCH

ros::Timer timer_saveData;

#endif

//获取时间帧
string gettime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
    return tmp;
}
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                数据存储                                               */
/*                                                                                     */
/***************************************************************************************/

ofstream fout;
ofstream file_robot1_trajectory("/home/guolab/catkin_ws/src/gazebo_ros_robot/asrobot/data/robot1_reference_trajectory.txt");
ofstream file_robot2_trajectory("/home/guolab/catkin_ws/src/gazebo_ros_robot/asrobot/data/robot2_reference_trajectory.txt");
ofstream file_robot3_trajectory("/home/guolab/catkin_ws/src/gazebo_ros_robot/asrobot/data/robot3_reference_trajectory.txt");
ofstream file_vrRobot_trajectory("/home/guolab/catkin_ws/src/gazebo_ros_robot/asrobot/data/virtureRobot_reference_trajectory.txt");
void reference_trajectory_to_file(MatrixXd robot1_reference_trajectory_,
                                  MatrixXd robot2_reference_trajectory_,
                                  MatrixXd robot3_reference_trajectory_,
                                  MatrixXd virturerobot_reference_trajectory_)
{
    file_robot1_trajectory << "path_x"
                           << "     "
                           << "path_y"
                           << "     "
                           << "path_yaw"
                           << "     "
                           << "path_velocity"
                           << "     "
                           << "path_curvature"
                           << "     " << endl;
    if (0 != robot1_reference_trajectory_.rows())
    {
        for (int i = 0; i < robot1_reference_trajectory_.rows(); i++)
        {
            file_robot1_trajectory << robot1_reference_trajectory_(i, 0) << "     "
                                   << robot1_reference_trajectory_(i, 1) << "     "
                                   << robot1_reference_trajectory_(i, 2) << "     "
                                   << robot1_reference_trajectory_(i, 3) << "     "
                                   << robot1_reference_trajectory_(i, 4) << "     " << endl;
            //        cout << path_3d_.path_x[i] << " " << endl;
        }
    }
    file_robot2_trajectory << "path_x"
                           << "     "
                           << "path_y"
                           << "     "
                           << "path_yaw"
                           << "     "
                           << "path_velocity"
                           << "     "
                           << "path_curvature"
                           << "     " << endl;
    if (0 != robot2_reference_trajectory_.rows())
    {
        for (int i = 0; i < robot2_reference_trajectory_.rows(); i++)
        {
            file_robot2_trajectory << robot2_reference_trajectory_(i, 0) << "     "
                                   << robot2_reference_trajectory_(i, 1) << "     "
                                   << robot2_reference_trajectory_(i, 2) << "     "
                                   << robot2_reference_trajectory_(i, 3) << "     "
                                   << robot2_reference_trajectory_(i, 4) << "     " << endl;
            //        cout << path_3d_.path_x[i] << " " << endl;
        }
    }
    file_robot3_trajectory << "path_x"
                           << "     "
                           << "path_y"
                           << "     "
                           << "path_yaw"
                           << "     "
                           << "path_velocity"
                           << "     "
                           << "path_curvature"
                           << "     " << endl;
    if (0 != robot3_reference_trajectory_.rows())
    {
        for (int i = 0; i < robot3_reference_trajectory_.rows(); i++)
        {
            file_robot3_trajectory << robot3_reference_trajectory_(i, 0) << "     "
                                   << robot3_reference_trajectory_(i, 1) << "     "
                                   << robot3_reference_trajectory_(i, 2) << "     "
                                   << robot3_reference_trajectory_(i, 3) << "     "
                                   << robot3_reference_trajectory_(i, 4) << "     " << endl;
            //        cout << path_3d_.path_x[i] << " " << endl;
        }
    }
    file_vrRobot_trajectory << "path_x"
                            << "     "
                            << "path_y"
                            << "     "
                            << "path_yaw"
                            << "     "
                            << "path_velocity"
                            << "     "
                            << "path_curvature"
                            << "     " << endl;
    if (0 != virturerobot_reference_trajectory_.rows())
    {
        for (int i = 0; i < virturerobot_reference_trajectory_.rows(); i++)
        {
            file_vrRobot_trajectory << virturerobot_reference_trajectory_(i, 0) << "     "
                                    << virturerobot_reference_trajectory_(i, 1) << "     "
                                    << virturerobot_reference_trajectory_(i, 2) << "     "
                                    << virturerobot_reference_trajectory_(i, 3) << "     "
                                    << virturerobot_reference_trajectory_(i, 4) << "     " << endl;
            //        cout << path_3d_.path_x[i] << " " << endl;
        }
    }
}

/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 消息发布函数                                          */
/*                                                                                     */
/***************************************************************************************/
#if PUBLISH_SWITCH

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 消息订阅函数                                          */
/*                                                                                     */
/***************************************************************************************/

#if SUBSCRIBE_SWITCH

void mccallback(const asrobot::mc::ConstPtr &msg)
{

#if DEBUG_SPEED
    cur_p_x = msg->c_x;
    cur_v_x = msg->c_speedx;

    cur_p_y = msg->c_y;
    cur_v_y = msg->c_speedy;

    cur_p_z = msg->c_z;

    cur_yaw = msg->c_navyaw;
    cur_roll = msg->c_navroll;
    cur_pitch = msg->c_navpitch;

    cur_v_yaw = msg->c_navyawvelocity;

    if (speed_debug == true)
    {

        cout << " ******speed debug running!!!!! *****" << endl;
        // 机器人的速度在机器人坐标系下 实际的x方向的速度其实是世界坐标系下的x^2和y^2
        // 先用H型做测试
        simCombineADRC_3DSpeedCtrl(exp_v_x, exp_v_y,
                                   exp_v_yaw, exp_v_z,

                                   sqrt(cur_v_x * cur_v_x + cur_v_y * cur_v_y), cur_v_y,
                                   cur_v_yaw, cur_v_z);//vx=0,vyaw
        // simCombineADRC_3DSpeedCtrl_v2(exp_v_x, exp_v_y,
        //                               exp_v_yaw, exp_v_z,
        //                               exp_yaw,

        //                               sqrt(cur_v_x * cur_v_x + cur_v_y * cur_v_y), cur_v_y,
        //                               cur_v_yaw, cur_v_z,
        //                               cur_yaw); //vx,vyaw=0,yaw=0
        if (save_data_mode)
        {
            save_identi_data(msg->c_x, msg->c_y, msg->c_speedx, msg->c_speedy,
                             msg->c_navyaw, msg->c_navyawvelocity,
                             four_thrust1, four_thrust2, four_thrust3, four_thrust4);
            //cout<<"current_robot_velocity----------:"<<sqrt(msg->c_speedx*msg->c_speedx+msg->c_speedy*msg->c_speedy)<<endl;
            //cout<<"current_yaw_velocity---:"<<msg->c_navyawvelocity<<endl;
        }
    }
    else
    {
        ;
    }

#endif

#if QUADROTOR_MODE // Quadrotor mode
    // 接收实时数据
    cur_p_x = msg->c_x;
    cur_v_x = msg->c_speedx;

    cur_p_y = msg->c_y;
    cur_v_y = msg->c_speedy;

    cur_p_z = msg->c_z;

    cur_yaw = msg->c_navyaw;
    cur_roll = msg->c_navroll;
    cur_pitch = msg->c_navpitch;

    // 接收该机器人期望位置 暂时用手动控制 先不接收
    exp_p_x = msg->e_x;
    exp_p_y = msg->e_y;
    exp_p_z = msg->e_z;

    exp_yaw = msg->e_navyaw;
    exp_yaw = atan2(exp_p_y - cur_p_y, exp_p_x - cur_p_x) * 57.3; // 弧度转角度乘57.3

    cout << "=========e-x==========" << msg->e_x <<"=========e-y==========" << msg->e_y << endl;
    cout << "=========c-x==========" << msg->c_x <<"=========c-y==========" << msg->c_y << endl;
    
    //////////////////////////////////////////////
    
    /////////////////////////////////////////////
    simCombineADRC_3DCtrl(exp_p_x, exp_v_x,
                          exp_p_y, exp_v_y,
                          exp_p_z, exp_yaw,

                          cur_p_x, cur_v_x,
                          cur_p_y, cur_v_y,
                          cur_p_z, cur_yaw);
    if (save_data_mode)
    {
        save_identi_data(msg->c_x, msg->c_y, msg->c_speedx, msg->c_speedy,
                         msg->c_navyaw, msg->c_navyawvelocity,
                         force_1, force_2, force_3, force_4);
    }
#endif

#if CAMERA8_MODE // Camera8 mode
    // 接收实时数据
    cur_p_x = msg->c_x;
    cur_v_x = 0;

    cur_p_y = msg->c_y;
    cur_v_y = 0;

    cur_p_z = msg->c_z;

    cur_yaw = msg->c_navyaw;
    cur_roll = msg->c_navroll;
    cur_pitch = msg->c_navpitch;

    // 接收该机器人期望位置 在手动输入目标点时,需要注释掉
    //    exp_p_x = msg -> e_x;
    //    exp_p_y = msg -> e_y;
    //    exp_p_z = msg -> e_z;

    exp_yaw = atan2(exp_p_y - cur_p_y, exp_p_x - cur_p_x) * 57.3; // 弧度转角度乘57.3
    simCombineADRC_3DCtrl(exp_p_x, exp_v_x,
                          exp_p_y, exp_v_y,
                          exp_p_z, exp_yaw,

                          cur_p_x, cur_v_x,
                          cur_p_y, cur_v_y,
                          cur_p_z, cur_yaw);

#endif

#if MPC_mode
    //cout<<"getting into the MPC_MODE"<<endl;
    VectoredThrustControl::Attitude c_attitude;
    VectoredThrustControl::Position c_position;
    VectoredThrustControl::Speed c_speed;

    //HH=HH+1;
    //cout<<"****current_num:"<<HH<<endl;
    c_position.x = msg->c_x;
    c_position.y = msg->c_y;
    c_position.z = msg->c_z;
    c_speed.x = msg->c_speedx;
    c_speed.y = msg->c_speedy;

    c_speed.yaw = msg->c_navyawvelocity;
    //cout<<"===机器人的角速度"<<msg->c_navyawvelocity<<endl;
    c_attitude.yaw = msg->c_navyaw;
    cur_yaw = msg->c_navyaw;
    c_attitude.roll = msg->c_navroll;
    c_attitude.pitch = msg->c_navpitch;
    /*************************************存储数据***********************/
    if (save_data_mode)
    {
        save_identi_data(msg->c_x, msg->c_y, msg->c_speedx, msg->c_speedy,
                         msg->c_navyaw, msg->c_navyawvelocity,
                         force_1, force_2, force_3, force_4);
        cout << "current_robot_velocity----------:" << sqrt(msg->c_speedx * msg->c_speedx + msg->c_speedy * msg->c_speedy) << endl;
        cout << "current_yaw_velocity---:" << msg->c_navyawvelocity << endl;
    }

    f_mpcTracking(c_position, c_attitude, c_speed);

#endif

#if MANUAL_INPUT_MODE

    c_depthvalue = msg->c_depthvalue;
    cur_yaw = msg->c_navyaw;

    simCombineADRC_3DCtrl(exp_force, 0,
                          0, 0,
                          e_depthvalue, exp_yaw,

                          0, 0,
                          0, 0,
                          c_depthvalue, cur_yaw);

#endif


//hcy:（重要）使用H型只调节yaw
#if DEBUG_YAW
    cur_yaw = msg -> c_navyaw;
    cur_roll = msg -> c_navroll;
    cur_pitch = msg -> c_navpitch;

    exp_yaw = exp_yaw;

#if DEBUG_MOTION
    motion_debug.c_navyaw = cur_yaw;
    motion_debug.e_navyaw = exp_yaw;

#endif

        if (yaw_debug == true)
    {
        cout << "**************************进入航向调节**************************" << endl;
        ADRCYawControl_H(exp_yaw, cur_yaw);
        cout <<  "exp_yaw: " <<  exp_yaw << "  " << "cur_yaw: " << cur_yaw <<endl;
    }
    else
    {
        ;
    }
#endif

//hcy:（重要）使用X型只调节z
#if DEBUG_Depth
    cur_p_x = msg -> c_x;
    cur_p_y = msg -> c_y;
    cur_p_z = msg -> c_z;

    cur_p_z = cur_p_z;    // 输入负数
    exp_p_z = exp_p_z;

#if DEBUG_MOTION
    motion_debug.e_z = exp_p_z;
    motion_debug.c_z = cur_p_z;

#endif

    if(depth_debug == true)
    {
        cout << "**************************进入深度调节**************************" << endl;
        ADRCDepthControl_X(exp_p_z, cur_p_z);
    }
    else
    {
        ;
    }
#endif

//hcy:（重要）docking、tracking用
#if DEBUG_H_3D_POS_V2  // 第二种对接控制方案，仍采用1代ADRC



    // 暂时需要手动给出x和y的坐标
#if AUTO_CTRL
    cam_exp_p_x = msg -> cam_e_x;//hy:三组新全局坐标系下固定的规划点
    cam_exp_p_y = msg -> cam_e_y;
    cam_exp_p_z = msg -> cam_e_z;
    cam_exp_yaw = msg -> cam_e_yaw;//hy:根据固定的规划点和机器人在新全局坐标系下的坐标算出来的yaw，
                                                                            //机器人在规划点左侧时为负，右侧时为正  //e_yaw
#endif
    
  

    cam_cur_p_x = msg -> cam_c_x;//hy:即cam_pos_x，机器人在新全局坐标系下的x    //c_x
    cam_cur_p_y = msg -> cam_c_y;
    cam_cur_p_z = msg -> cam_c_z;
    cam_cur_roll = msg -> cam_c_roll;
    cam_cur_pitch = msg -> cam_c_pitch;
    cam_cur_yaw = msg -> cam_c_yaw;//hy:即cam_yaw，视觉得出的yaw    //c_yaw

    depth_sensor_data = msg -> c_depthvalue;

    heli_traj_radius = msg -> heli_traj_radius;

    if(h_3d_pos_debug_v2 == true)
    {

//        ADRCPosition3DControl_H(0, 0, exp_yaw, 0,
//                                   0,  0, cam_cur_yaw, 0); // 单调节航向

//        ADRCPosition3DControl_H(0, 0, 0, cam_exp_p_z,
//                                   0,  0, 0, cam_cur_p_z); // 单调节深度

//        ADRCPosition3DControl_H(cam_exp_p_x, cam_exp_p_y, cam_exp_yaw, cam_exp_p_z,
//                                   cam_cur_p_x,  cam_cur_p_y, cam_cur_yaw, cam_cur_p_z);
        ADRCPosition3DControl_H_v2(cam_exp_p_x, cam_exp_p_y, cam_exp_yaw, cam_exp_p_z,
                                   cam_cur_p_x,  cam_cur_p_y, cam_cur_yaw, cam_cur_p_z);
//	  ADRCPosition3DControl_H(0, 0, cam_exp_yaw, 0,
 //                                 0,  0, cam_cur_yaw, 0);


        cout << "cam_exp_p_x: " << cam_exp_p_x << " cam_cur_p_x: " << cam_cur_p_x <<endl;//hy:即cam_pos_x，机器人x
        cout << "cam_exp_p_y: " << cam_exp_p_y << " cam_cur_p_y: " << cam_cur_p_y <<endl;
        cout << "cam_exp_p_z: " << cam_exp_p_z << " cam_cur_p_z: " << cam_cur_p_z <<endl;
        cout << "cam_exp_yaw: " << cam_exp_yaw << " cam_cur_yaw: " << cam_cur_yaw <<endl;

	

        // 需要保存的数据有：
        // 1 四个推进器推力和舵机角度
        // 2 机器人轨迹半径值
        // 3 深度计
        // 4 机器人参考位置、机器人实际位置、机器人姿态
        // save_this_robot_docking_data(cam_cur_p_x, cam_cur_p_y, cam_cur_p_z,
        //                              cam_cur_roll, cam_cur_pitch, cam_cur_yaw,
        //                              depth_sensor_data,
        //                              heli_traj_radius,
        //                              cam_exp_p_x, cam_exp_p_y, cam_exp_p_z, cam_exp_yaw,
        //                              comb3DPositionCtrl.controller_x_position, comb3DPositionCtrl.controller_yaw, comb3DPositionCtrl.controller_z_position,
        //                              comb3DPositionCtrl.vec_ctrl.thrust_force_1, comb3DPositionCtrl.vec_ctrl.thrust_force_2, comb3DPositionCtrl.vec_ctrl.thrust_force_3,comb3DPositionCtrl.vec_ctrl.thrust_force_4,
        //                              comb3DPositionCtrl.vec_ctrl.leg_angle_1, comb3DPositionCtrl.vec_ctrl.leg_angle_2, comb3DPositionCtrl.vec_ctrl.leg_angle_3, comb3DPositionCtrl.vec_ctrl.leg_angle_4);

    }
    else
    {
        ;
    }


#endif

#if DEBUG_H_3D_POS_V3 //ljl

    cam_cur_p_x = msg->cam_c_x; 
    cam_cur_p_y = msg->cam_c_y;
    cam_cur_p_z = msg->cam_c_z;
    cam_cur_yaw = msg->cam_c_yaw;

    cam_exp_p_x = msg->cam_e_x;
    cam_exp_p_y = msg->cam_e_y;
    cam_exp_p_z = msg->cam_e_z;
    cam_exp_yaw = msg->cam_e_yaw;


    depth_sensor_data = msg->c_depthvalue;

    heli_traj_radius = msg->heli_traj_radius;

#if DEBUG_MOTION
    motion_debug.e_z = exp_p_z;
    motion_debug.c_z = cur_p_z;

#endif

    /*************控制***************/
    if (stage1_horizontal_align == true)
    {
        ADRCPosition3DControl_H_v2(cam_exp_p_x, cam_exp_p_y, cam_exp_yaw, 0,
                                   cam_cur_p_x, cam_cur_p_y, cam_cur_yaw, 0);
        cout << "Stage 1 ------------------------------------- Current Z: "
             << depth_sensor_data << " -------------------------------------" << endl;
        if (abs(cam_exp_p_x - cam_cur_p_x) < 0.05 && abs(cam_cur_p_y - cam_cur_p_y) < 0.05)
        {
            stage1_horizontal_align = false;
            stage2_descend_to_near = true;
        }
    }

    if (stage2_descend_to_near == true)
    {
        float target_z_relative_stage2 = -0.02f;
        ADRCPosition3DControl_X(cam_exp_p_x, cam_exp_p_y, cam_exp_p_z,
                                cam_cur_p_x, cam_cur_p_y, cam_cur_yaw, cam_cur_p_z);
        cout << "Stage 2 ------------------------------------- Current Z: "
             << depth_sensor_data << " -------------------------------------" << endl;
        if (abs(cam_exp_p_z - cam_cur_p_z) < 0.05)
        {
           // stage3_start_time = ros::Time::now();
           // stage3_initial_cam_exp_p_z = cam_exp_p_z; // 此时 cam_exp_p_z 应该约等于 -0.1//实际-0.13
            //ROS_INFO("Entering Stage 3. stage3_initial_cam_exp_p_z = %f", -stage3_initial_cam_exp_p_z);

            stage2_descend_to_near = false;
            vec_ctrl.thrusterforce2speed(0, 0, 0, 0);
            clearADRCSpeedControllerVariables();
            legSetAngle(0, 0, 0);
            cout << " *****************stop***************" << endl;
           //cout << "Stage 2 Complete. " << endl;
        }
        
    }

#endif

    mc_pub_debug.publish(motion_debug);
}//cb


void Spine_pathCallback(const asrobot::mul_ref_trajectory::ConstPtr &msg)
{
    // cout<<"getting the reference trajectory"<<endl;
    reference_Point_Num = msg->path_length;
    Eigen::MatrixXd temp1(msg->path_length, 5);
    Eigen::MatrixXd temp2(msg->path_length, 5);
    Eigen::MatrixXd temp3(msg->path_length, 5);
    Eigen::MatrixXd temp4(msg->path_length, 5);
    for (int i = 0; i < msg->path_length; i++)
    {
        temp1(i, 0) = msg->robot1_path_x[i];
        temp1(i, 1) = msg->robot1_path_y[i];
        temp1(i, 2) = msg->robot1_path_yaw[i];
        temp1(i, 3) = msg->robot1_path_velocity[i];
        //temp1(i,4)=msg->robot1_path_curvature[i];
        temp1(i, 4) = 0;

        temp2(i, 0) = msg->robot2_path_x[i];
        temp2(i, 1) = msg->robot2_path_y[i];
        temp2(i, 2) = msg->robot2_path_yaw[i];
        temp2(i, 3) = msg->robot2_path_velocity[i];
        //temp2(i,4)=msg->robot2_path_curvature[i];
        temp2(i, 4) = 0;

        temp3(i, 0) = msg->robot3_path_x[i];
        temp3(i, 1) = msg->robot3_path_y[i];
        temp3(i, 2) = msg->robot3_path_yaw[i];
        temp3(i, 3) = msg->robot3_path_velocity[i];
        //temp3(i,4)=msg->robot3_path_curvature[i];
        temp3(i, 4) = 0;

        temp4(i, 0) = msg->virtureRobot_path_x[i];
        temp4(i, 1) = msg->virtureRobot_path_y[i];
        temp4(i, 2) = msg->virtureRobot_path_yaw[i];
        temp4(i, 3) = msg->virtureRobot_path_velocity[i];
        temp4(i, 4) = msg->virtureRobot_path_curvature[i];
    }
    vs_reference_PositionX = msg->vs_reference_x;
    vs_reference_PositionY = msg->vs_reference_y;

    robot1_reference_trajectory = temp1;
    robot2_reference_trajectory = temp2;
    robot3_reference_trajectory = temp3;
    virtureRobot_reference_trajectory = temp4;
    if (robot1_reference_trajectory.rows() > 0)
    {

        if (abs(last_ref_num - robot1_reference_trajectory.rows()) > 5)
        {
            reference_trajectory_to_file(robot1_reference_trajectory, robot2_reference_trajectory, robot3_reference_trajectory, virtureRobot_reference_trajectory);
            last_ref_num = robot1_reference_trajectory.rows();
        }
    }
}
void multiRobotPosition_Callback(const asrobot::quadrotor2robot::ConstPtr &msg)
{
    Robot1_c_position.x = msg->robot1_pos_x;
    Robot1_c_position.y = msg->robot1_pos_y;
    Robot2_c_position.x = msg->robot2_pos_x;
    Robot2_c_position.y = msg->robot2_pos_y;
    Robot3_c_position.x = msg->robot3_pos_x;
    Robot3_c_position.y = msg->robot3_pos_y;
}

void cmdcallback(const std_msgs::String::ConstPtr &msg)
{
    string cmdstring;
    vector<string> cmd;
    //getline(cin,cmdstring);
    cmdstring = msg->data;
    stringstream cmdstream(cmdstring);
    string buf;
    while (cmdstream >> buf)
    {
        cmd.push_back(buf);
    }

    if (cmd[0] == "pid") //set pid parameter,for example:"pid 3 2 1"
    {
        if (cmd.size() == 4)
        {
            //setYawPIDPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()));
            //            setPitchPIDPmt( atof(cmd[1].c_str()),atof(cmd[2].c_str()),atof(cmd[3].c_str()) );
            setDepthPIDPmt( atof(cmd[1].c_str()),atof(cmd[2].c_str()),atof(cmd[3].c_str()) );
            //            setSpeedPIDPmt( atof(cmd[1].c_str()),atof(cmd[2].c_str()),atof(cmd[3].c_str()) );

            cout << "set pid parameter:"
                 << " p=" << atof(cmd[1].c_str()) << " i=" << atof(cmd[2].c_str()) << " d=" << atof(cmd[3].c_str());
            cout << endl
                 << endl;
        }
    }

//hcy:gazebo原程序使用
    // if (cmd[0] == "yaw") //yaw control mode and set expect yaw angle,for example:"yaw 90"
    // {
    //     if (cmd.size() == 2)
    //     {
    //         mode = 0;
    //         LegShape_H();
    //         //e_navyaw = atof(cmd[1].c_str());
    //         e_attitude.yaw = atof(cmd[1].c_str());
    //         cout << "yaw control mode!" << endl
    //              << endl;
    //     }
    // }


    // hcy:设置期望航向角
    if(cmd[0] == "yaw")//yaw control mode and set expect yaw angle,for example:"yaw 90"
    {
        if(cmd.size() == 2){

            yaw_debug = true;
            exp_yaw = atof(cmd[1].c_str());
            cout <<"yaw control mode!" << endl << endl;
        }
    }

    if (cmd[0] == "espeed") //yaw control mode and set expect yaw angle,for example:"yaw 90"
    {
        mode = 0;
        LegShape_H_T();
        //e_navyaw = atof(cmd[1].c_str());
        e_speed.x = atof(cmd[1].c_str());
        cout << "speed control mode!" << endl
             << endl;
    }

    if (cmd[0] == "stop") //stop the water motor
    {
        usleep(100000);
        simMotorForce(0, 0, 0, 0, 0, 0, 0);
        cout << "stop motor!" << endl
             << endl;
        if (mode == 1)
        {
            mode = 100;
            gait_stop();
        }
        mode = 100;
    }

    if (cmd[0] == "w") //set the force and torque of robot,for example:"wrench 1.2 1.0 0.5"
    {
        save_data_mode = 1;
        if (5 == cmd.size())
        {
            setComb3DCtrlMode(0); // 强制关掉 debug mode
            simMotorForce(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), atof(cmd[4].c_str()),
                          0, cur_yaw, 0);
            force_x = atof(cmd[1].c_str());
            force_y = atof(cmd[2].c_str());
            force_z = atof(cmd[3].c_str());
            force_yaw = atof(cmd[4].c_str());
            cout << "**********" << cur_yaw << endl;
            cout << "set wrench value:" << atof(cmd[1].c_str()) << " " << atof(cmd[2].c_str()) << " " << atof(cmd[3].c_str()) << atof(cmd[4].c_str()) << endl
                 << endl;
            cout << "set wrench angle:" << c_attitude.roll << " " << c_attitude.yaw << " " << c_attitude.pitch << endl;
        }
    }

    if (cmd[0] == "t") //set the force and torque of robot,for example:"wrench 1.2 1.0 0.5"
    {
        save_data_mode = 1;
        if (5 == cmd.size())
        {
            setComb3DCtrlMode(0); // 强制关掉 debug mode
            speed_debug = false;  // 停止速度调试
            force_1 = atof(cmd[1].c_str());
            force_2 = atof(cmd[2].c_str());
            force_3 = atof(cmd[3].c_str());
            force_4 = atof(cmd[4].c_str());
            vec_ctrl.thrusterforce2speed(force_1, force_2, force_3, force_4);
        }
    }

    // 控制器指令集 一个一个控制 adrc 参数指令
    if (cmd[0] == "ad")
    {
        if (cmd.size() == 2)
        {
            // 注意调节 K_p, K_d, 带宽先定为 100 暂时不加入补偿因子
            //            setPxADRCPmt( atof(cmd[1].c_str()), 0, 30, 0 );
            //            setVxADRCPmt( atof(cmd[1].c_str()), 0, 100, 0 );
            //            setPyADRCPmt( atof(cmd[1].c_str()), 0, 30, 0 );
            //            setVyADRCPmt( atof(cmd[1].c_str()), 0, 100, 0 );
            //            setPzADRCPmt( atof(cmd[1].c_str()), 0, 100, 0 );
            setYawADRCPmt(atof(cmd[1].c_str()), 0, 30, 0);
            setYawTDandESOPmt(atof(cmd[1].c_str()), 0, 30, 0);
        }

        if (cmd.size() == 3)
        {
            //            setPxADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 30, 0 );
            //            setVxADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 100, 0 );
            //            setPyADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 30, 0 );
            //            setVyADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 100, 0 );
            //            setPzADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 100, 0 );
            setYawADRCPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), 30, 0);
            setYawTDandESOPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), 30, 0);
        }

        if (cmd.size() == 5)
        {
            //            setPxADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), atof(cmd[4].c_str()) );
            //            setVxADRCPmt( atof(cmd[1].c_str()), 0, 100, 0 );
            //            setPyADRCPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), atof(cmd[4].c_str()) );
            //            setVyADRCPmt( atof(cmd[2].c_str()), 0, 100, 0 );
            setYawADRCPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), atof(cmd[4].c_str()));
            setYawTDandESOPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), atof(cmd[4].c_str()));
        }
    }

    // s面指令
    if (cmd[0] == "ss")
    {
        if (cmd.size() == 2)
        {
            // 注意调节 K_p, K_i, K_d, 外部增益 先定为 3
            setPxSSPmt(atof(cmd[1].c_str()), 0, 0, 3);
            //            setVxSSPmt( atof(cmd[1].c_str()), 0, 0, 3);

            //            setPySSPmt( atof(cmd[1].c_str()), 0, 0, 3);
            //            setVySSPmt( atof(cmd[1].c_str()), 0, 0, 3);

            //            setPzSSPmt( atof(cmd[1].c_str()), 0, 0, 3);
            //            setYawSSPmt( atof(cmd[1].c_str()), 0, 0, 3);
        }

        if (cmd.size() == 3)
        {
            setPxSSPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), 0, 3);
            //            setVxSSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 0, 3);

            //            setPySSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 0, 3);
            //            setVySSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 0, 3);

            //            setPzSSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 0, 3);
            //            setYawSSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), 0, 3);
        }
        if (cmd.size() == 4)
        {
            setPxSSPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), 3);
            //            setVxSSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), 3);

            //            setPySSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), 3);
            //            setVySSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), 3);

            //            setPzSSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), 3);
            //            setYawSSPmt( atof(cmd[1].c_str()), atof(cmd[2].c_str()), atof(cmd[3].c_str()), 3);
        }
    }

    // 单环 位置x 期望
    if (cmd[0] == "x")
    {
        if (cmd.size() == 3)
        {
            setComb3DCtrlMode(Px_CONTROL_MODE);
            exp_p_x = atof(cmd[1].c_str());
            exp_p_y = atof(cmd[2].c_str());
        }
    }

    // 单环 速度x 期望
    if (cmd[0] == "vx")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(Vx_CONTROL_MODE);
            exp_v_x = atof(cmd[1].c_str());
        }
    }

    // 双环 位置x 期望
    if (cmd[0] == "xx")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(Px_Vx_CONTROL_MODE);
            exp_p_x = atof(cmd[1].c_str());
        }
    }

    // 单环 位置y 期望
    if (cmd[0] == "y")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(Py_CONTROL_MODE);
            exp_p_y = atof(cmd[1].c_str());
        }
    }

    // 单环 速度y 期望
    if (cmd[0] == "vy")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(Vy_CONTROL_MODE);
            exp_v_y = atof(cmd[1].c_str());
        }
    }

    // 双环 位置y 期望
    if (cmd[0] == "yy")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(Py_Vy_CONTROL_MODE);
            exp_p_y = atof(cmd[1].c_str());
        }
    }

    //  位置z 期望
    if (cmd[0] == "z")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(Pz_CONTROL_MODE);
            exp_p_z = atof(cmd[1].c_str());
        }
    }

    //  航向角 yaw 期望
    if (cmd[0] == "rz")
    {
        if (cmd.size() == 2)
        {
            setComb3DCtrlMode(YAW_CONTROL_MODE);
            exp_yaw = atof(cmd[1].c_str());
        }
    }

    // 全方位控制  添加
    if (cmd[0] == "xyz")
    {
        if (cmd.size() == 4)
        {
            setComb3DCtrlMode(ALL_CONTROL_MODE_SINGLE_INTEG); // 单环三维控制
            //setComb3DCtrlMode( ALL_CONTROL_MODE_DOUBLE_INTEG );   // 双环三维控制
            exp_p_x = atof(cmd[1].c_str());
            exp_p_y = atof(cmd[2].c_str());
            exp_p_z = atof(cmd[3].c_str());
            exp_yaw = atan2(exp_p_y - cur_p_y, exp_p_x - cur_p_x) * 57.3; // 弧度转角度乘57.3
        }
    }

    if (cmd[0] == "ff")
    { // ff: fast formation for quadrotor
        comb3Dctrl.debug_mode = ALL_CONTROL_MODE_DOUBLE_INTEG;
    }

    // 急停
    if (cmd[0] == "st")
    {
        save_data_mode = 0;
        setComb3DCtrlMode(0);
        vec_ctrl.thrusterforce2speed(0, 0, 0, 0); //推进器停转
        speed_debug = false;                      // 停止速度调试
        clearADRCSpeedControllerVariables();      // 清变量

        //        simMotorForce(0, 0, 0, 0, 0, 0, 0);
        fout << gettime() << "\t"
             << "     ";
        fout << "st" << endl;
        iter_num = 0;
        MPC_real_force = Eigen::VectorXd::Zero(2);
        cout << " *************************stop !!!!!!!!!!!!! " << endl;
        legSetAngle(0, 0, 0);

        yaw_debug   = false;                  // 停止航向角调试//hcy
        depth_debug = false;                  // 停止深度调试//hcy
        h_3d_pos_debug = false;//hcy
        stage1_horizontal_align = false; // hcy
        stage2_descend_to_near = false;  // hcy
        stage3_slow_final_descend = false; // hcy
    }

    if (cmd[0] == "mpc")
    {
        setComb3DCtrlMode(MPC_MODE);
    }

    if (cmd[0] == "track")
    {
        save_data_mode = 1;
        setComb3DCtrlMode(ALL_CONTROL_MODE_SINGLE_INTEG);
    }
    if (cmd[0] == "setQ")
    {
        if (cmd.size() == 6)
        {
            q11 = atof(cmd[1].c_str());
            q22 = atof(cmd[2].c_str());
            q33 = atof(cmd[3].c_str());
            q44 = atof(cmd[4].c_str());
            q55 = atof(cmd[5].c_str());
        }
        Init_q33=q33;
        Init_q22=q22;
        Init_q44=q44;
    }


//hcy:gazebo原程序使用
    // if (cmd[0] == "dep") //reserve
    // {
    //     mode = 0;
    //     LegShape_X();
    //     e_depthvalue = atof(cmd[1].c_str());
    //     cout << "depth control mode!" << endl
    //          << endl;
    // }


    // 调节深度//hcy
    if(cmd[0] == "dep")//reserve
    {
        if(cmd.size() == 2)
        {
            depth_debug = true;
            exp_p_z = atof(cmd[1].c_str());
            cout <<"******************depth control mode!******************" << endl << endl;
        }
    }

    if (cmd[0] == "left") //reserve
    {
        dir = LEFT;
        gait_R = atof(cmd[1].c_str());
        cout << "left!" << endl
             << endl;
    }

    if (cmd[0] == "right") //reserve
    {
        dir = RIGHT;
        gait_R = atof(cmd[1].c_str());
        cout << "right!" << endl
             << endl;
    }

    if (cmd[0] == "forward") //reserve
    {
        dir = FORWARD;
        gait_R = atof(cmd[1].c_str());
        cout << "forward!" << endl
             << endl;
    }
    if (cmd[0] == "back")
    {
        dir = BACK;
        gait_R = atof(cmd[1].c_str());
        if (gait_R < 300)
            gait_R = 300;
        cout << "back!" << endl
             << endl;
    }

    if (cmd[0] == "gait") //gait control mode, and set gait_R
    {
        cout << "gait control mode!" << endl
             << endl;
        if (mode != 1)
        {
            mode = 1;
            simMotorForce(0, 0, 0, 0, 0, 0, 0);
        }
        gait_R = atof(cmd[1].c_str());
    }
    if (cmd[0] == "open") //open the power of servo and motor
    {
        key.openservo = 1;
        key.openmotor = 1;
        //extern_pub_status->publish(key);
        cout << "open motor power!" << endl
             << endl;
    }
    if (cmd[0] == "close") //close the power of servo and motor
    {
        key.openservo = 0;
        key.openmotor = 0;
        //extern_pub_status->publish(key);
        cout << "close motor power!" << endl
             << endl;
    }

    if (cmd[0] == "p") //leg position
    {
        float A, B, C;
        A = atof(cmd[1].c_str());
        B = atof(cmd[2].c_str());
        C = atof(cmd[3].c_str());
        printf("set position\n");
        legSetPosition(A, B, C);
    }

    if (cmd[0] == "a") //leg angle
    {
        float A, B, C;
        A = atof(cmd[1].c_str());
        B = atof(cmd[2].c_str());
        C = atof(cmd[3].c_str());
        printf("set angle\n");
        legSetAngle(A, B, C);
    }

    if (cmd[0] == "debug" || cmd[0] == "DEBUG") //debug mode
    {
        if (cmd.size() == 4)
        {
            setComb3DCtrlMode(ADJUST_PARAMETERS_MODE);
            exp_force = atof(cmd[1].c_str());
            e_depthvalue = -atof(cmd[2].c_str());
            exp_yaw = atof(cmd[3].c_str());
        }
    }

    /*  if(cmd[0] == "position")
    {
        e_x = atof(cmd[1].c_str());
        e_y = atof(cmd[2].c_str());

        cout <<"go to position: " <<e_x<<","<<e_y<< endl << endl;
    }*/

    /// DEBUG

    if (cmd[0] == "advte") // 第一组参数
    {
        if (cmd.size() == 5)
        {
            setSpeedXTDandESOPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                                 atof(cmd[3].c_str()), atof(cmd[4].c_str()));

            //            setSpeedYawTDandESOPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
            //                    atof(cmd[3].c_str()), atof(cmd[4].c_str()));
        }
    }

    if (cmd[0] == "advpd") // 第二组参数
    {
        if (cmd.size() == 5)
        {
            setSpeedXNLSEFPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                              atof(cmd[3].c_str()), atof(cmd[4].c_str()));

            //            setSpeedYawNLSEFPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
            //                    atof(cmd[3].c_str()), atof(cmd[4].c_str()));
        }
    }
    if (cmd[0] == "expv") // 期望的速度
    {
        save_data_mode=1;
        if (cmd.size() == 3)
        {
            exp_v_x = atof(cmd[1].c_str());
            exp_v_yaw = atof(cmd[2].c_str());
            //exp_yaw = atof(cmd[3].c_str());
            speed_debug = true;
        }
    }
    ///





    
    if(cmd[0] == "sun")//hcy
    {
	if(cmd.size() == 3)
	{
    		gait_ctrl.get_sun(atof(cmd[1].c_str()),atof(cmd[2].c_str()));
	}
    }

    // //hcy:设置四路电机的期望转速
    if(cmd[0] == "speed")
    {
        if(cmd.size() == 5)
        {
            pwm_msg.rotor_speed1 =  atof(cmd[1].c_str());
            pwm_msg.rotor_speed2 =  atof(cmd[2].c_str());
            pwm_msg.rotor_speed3 =  atof(cmd[3].c_str());
            pwm_msg.rotor_speed4 =  atof(cmd[4].c_str());

            extern_pub_pwm->publish(pwm_msg);
            cout << "the motor speed are: " << atof(cmd[1].c_str()) << "   " << atof(cmd[2].c_str()) << "   "
                                            << atof(cmd[3].c_str()) << "   " << atof(cmd[4].c_str()) << endl;
        }
    }

    if(cmd[0] == "docking") //hcy
    {
        if(cmd.size() == 1)
        {
            h_3d_pos_debug = true;
            h_3d_pos_debug_v2 = true;
            cout << "docking!!!!!!!!!!" << endl; 
        }      
    }

    if(cmd[0] == "tracking") //hcy
    {
        if(cmd.size() == 1)
        {       
            h_3d_pos_debug = true;
            h_3d_pos_debug_v2 = true;
            cout << "tracking!!!!!!!!!!" << endl; 
	    }        
    }

    if (cmd[0] == "ta") // ljl trackaruco
    {

        if (cmd.size() == 1)
        {
            stage1_horizontal_align = true;
            cout << "tracking_xyzyaw" << endl;
        }
    }

    // hcy:深度调试时的adrc参数
    if( cmd[0] == "adz" )
    {
        if(cmd.size() == 5)
        {
            setPzADRCPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                                            atof(cmd[3].c_str()), atof(cmd[4].c_str()));
            //setDebugPzADRCPmt_v2(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
            //                   atof(cmd[3].c_str()), atof(cmd[4].c_str()));
            cout << "adz------ " <<endl;
        }
        if(cmd.size() == 6)
        {
            setDebugPzADRCPmt_v2(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                               atof(cmd[3].c_str()), atof(cmd[4].c_str()),atof(cmd[5].c_str()));
            cout << "adz------ " <<endl;
        }
        if(cmd.size() == 7)
        {                                                            
            setDebugPzADRCPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                               atof(cmd[3].c_str()), atof(cmd[4].c_str()),atof(cmd[5].c_str()),
                               atof(cmd[6].c_str()));
            cout << "adz------ " <<endl;
        }
    }

    // hcy:航向角调试时的adrc参数
    if( cmd[0] == "adyaw" )
    {
        if(cmd.size() == 5)
        {
            setYawADRCPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                                               atof(cmd[3].c_str()), atof(cmd[4].c_str()));
            //setDebugYawADRCPmt_v2(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
            //                   atof(cmd[3].c_str()), atof(cmd[4].c_str()));
            cout << "adyaw------ " <<endl;

        }
        if(cmd.size() == 6)
        {
            setDebugYawADRCPmt_v2(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                               atof(cmd[3].c_str()), atof(cmd[4].c_str()),
                               atof(cmd[5].c_str()));
            cout << "adyaw------ " <<endl;

        }
        if(cmd.size() == 7)
        {
            setDebugYawADRCPmt(atof(cmd[1].c_str()), atof(cmd[2].c_str()),
                               atof(cmd[3].c_str()), atof(cmd[4].c_str()),
                               atof(cmd[5].c_str()),atof(cmd[6].c_str()));
            cout << "adyaw------ " <<endl;

        }
    }

    cmd.clear();
}

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 定时器触发函数                                         */
/*                                                                                     */
/***************************************************************************************/
#if TIMER_SWITCH

void saveDataTimerCallback(const ros::TimerEvent &event)
{
    // 加入定时保存数据
}

#endif
/*-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/

/***************************************************************************************/
/*                                                                                     */
/*                                 decision主程序                                       */
/*                                                                                     */
/***************************************************************************************/

int main(int argc, char **argv)
{

    fout.open("/home/guolab/catkin_ws/src/force.txt");
    fout << "Time"
         << "     "
         << "Force_x"
         << "     "
         << "force_y"
         << "     "
         << "force_z"
         << "     "
         << "force_yaw" << endl;

    /*****************************    ROS初始化     ******************************/
    /*                                                                          */

    ros::init(argc, argv, "motion");
    ros::NodeHandle n;
    extern_n = &n;
    ros::Rate loop_rate(iMotionLoopRate);

    /******************************   MPC control function 初始化***********************************************/
    initMpcOsqp();
    /*                                                                          */
    /*****************************    --------     ******************************/

    /*****************************    发布话题       ******************************/
    /*                                                                          */
#if PUBLISH_SWITCH

    pub_pwm = n.advertise<asrobot::pwm>("pwm_message", 10);
    pub_status = n.advertise<asrobot::status>("status_message", 10); //D kaiguan
    extern_pub_pwm = &pub_pwm;
    extern_pub_status = &pub_status;
    mc_pub_debug = n.advertise<asrobot::mc>("mc_message_debug", 10);

#endif
    /*                                                                          */
    /*****************************    --------     ******************************/

    /*****************************    订阅话题       ******************************/
    /*                                                                          */
#if SUBSCRIBE_SWITCH

    sub_depth = n.subscribe("mc_message", 100, mccallback);
    sub_cmd = n.subscribe("cmd_message", 10, cmdcallback);
    sub_trajectory = n.subscribe("mul_ref_trajectory", 10, Spine_pathCallback);
    sub_multiRobotPosition = n.subscribe("multi_RobotPosition", 10, multiRobotPosition_Callback);
#endif
    /*                                                                          */
    /*****************************    --------     ******************************/

    /*****************************    定时器触发      ******************************/
    /*                                                                          */
#if TIMER_SWITCH

    ros::Timer timer_saveData = n.createTimer(ros::Duration(DURATION), saveDataTimerCallback);

#endif
    /*                                                                          */
    /*****************************     --------     ******************************/

    /*****************************    自定义初始化     ******************************/
    /*                                                                          */
    simMotorForce(0, 0, 0, 0, 0, 0, 0);

    /// PID 控制器参数设置
    setYawPIDPmt(0.3, 0.0015, 0.0015);
    setPitchPIDPmt(0.5, 0.04, 0.15);
    setDepthPIDPmt(0.55, 0.02, 0.02);
    setSpeedPIDPmt(0.015, 0, 0);
    ///

    /// ADRC 控制器参数设置
    setPxADRCPmt(1.5, 0.1, 27, 0);

    setPyADRCPmt(1.5, 0.1, 27, 0);

    setPzADRCPmt(0, 0, 27, 0);
    //    setYawADRCPmt(0.035, 0, 100, 0);
    setYawADRCPmt(0.1, 0.0001, 25, 0.5);
    // setYawADRCPmt(0, 0, 27, 0);//hcy


    ///

    /// S面 控制器参数设置 该控制器参数还未整定
    // setPxSSPmt(0, 0, 0, 0);
    // setVxSSPmt(0, 0, 0, 0);
    // setPySSPmt(0, 0, 0, 0);
    // setVySSPmt(0, 0, 0, 0);
    // setPzSSPmt(0, 0, 0, 0);
    // setYawSSPmt(0, 0, 0, 0);
    ///

    /// ADRC 速度追踪控制器参数设置
    // setSpeedXTDandESOPmt(0.01, 5, 3.3, 1.5);
    // setSpeedXNLSEFPmt(1.5, 0, 0, 100);

    // setSpeedYTDandESOPmt(0.01, 5, 3.3, 1.5);
    // setSpeedYNLSEFPmt(1.5, 0, 0, 100);

    // setSpeedYawTDandESOPmt(0.01, 5, 4.8, 4);
    // setSpeedYawNLSEFPmt(17, 0, 0, 100);

    // setSpeedZTDandESOPmt(0, 0, 0, 0);
    // setSpeedZNLSEFPmt(0, 0, 0, 0);

    // setYawTDandESOPmt(0.1, 0.01, 27, 0);
    ///

    // hcy:实物正在用的控制器 全局坐标版本 2022.8
    /// ADRC 位置调试控制器参数设置
    // setDebugPzADRCPmt_v2(0.1,0.1,10,0.5);
    // setDebugPzADRCPmt_v2(4,10,5,20,30);//85 
    // setDebugYawADRCPmt_v2(4,5,1,3,30);
    // // setDebugYawADRCPmt_v2(20,20,20,20);//50
    // setDebugPxADRCPmt(1, 0.05, 0.05, 1, 5, 1);
    // setDebugPyADRCPmt(1, 0.05, 0.05, 1, 5, 1);


    //hcy:copy自李赞
    // ADRC 控制器参数设置
    // docking用的ADRC参数，对应 DEBUG_H_3D_POS_V2
    // setDebugPxADRCPmt(1, 0.05, 0.05, 1, 5, 1);
    // setDebugPyADRCPmt(1, 0.05, 0.05, 1, 5, 1);
    // setDebugPzADRCPmt(8.5, 0.1, 0.05, 1, 5, 1);
    // setDebugYawADRCPmt(0.15, 0.01, 0.05, 10, 20, 5);


    if (mode == 0)
        LegShape_X();
    /*                                                                          */
    /*****************************    --------     ******************************/

    /***************************************************************************************/
    /*                                 总循环                                               */
    /***************************************************************************************/

    while (ros::ok())
    {
        /*****************************    自定义函数   ******************************/
        /*                                                                           */

        /*                                                                          */
        /*****************************    --------     ******************************/

        /*****************************    循环发布消息    ******************************/
        /*                                                                           */

        /*                                                                          */
        /*****************************    --------     ******************************/

	//hcy
        switch (mode)
        {
        case 1:
            gaitControl();
            break;
        case 2:
            ADRCPosition3DControl_G(exp_p_x, exp_p_y, exp_yaw, exp_p_z, cur_p_x, cur_p_y, cur_yaw, cur_p_z);
            break;
        case 3:
            ADRCYawControl_G(exp_yaw, cur_yaw);
            break;
        case 4:
            ADRCPositionGraspControl_G(exp_p_x, exp_p_y, exp_yaw, exp_p_z, cur_p_x, cur_p_y, cur_yaw, cur_p_z);
            break;
        default:
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    /*-------------------------------------------------------------------------------------*/
    file_robot1_trajectory.close();
    file_robot2_trajectory.close();
    file_robot3_trajectory.close();
    file_vrRobot_trajectory.close();
    return 0;
}
