/**
 * @file aruco_demo.cpp
 * @brief 【参考／次要文件】无人机视觉对准与降落示例，当前未在 CMakeLists.txt 中编译。
 * 与当前机器人检测主链路独立；仅学习检测和坐标转换时，可以整文件暂时跳过。
 * 包含 MAVROS 状态/位姿回调、相机到机体变换、位置设定值发布及降落状态机。
 * 注意：这里使用 geometry_aruco_msgs/mavros_aruco_msgs 类型；接入当前使用
 * geometry_msgs::PoseStamped 的 /aruco/pose 前，应核对 ROS 消息类型兼容性。
 */
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * STcak and tested in Gazebo SITL
 */
// ********************************************************无人机参考程序***************************************************
#include <ros/ros.h>
#include <iostream>
// 话题用到的aruco_msgs的类型
#include <geometry_aruco_msgs/PoseStamped.h>
#include <mavros_aruco_msgs/CommandBool.h>
#include <mavros_aruco_msgs/SetMode.h>
#include <mavros_aruco_msgs/State.h>
#include <mavros_aruco_msgs/PositionTarget.h>
// 矩阵运算
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace Eigen;
using namespace std;

mavros_aruco_msgs::State current_state;

//Twb的意思是body系在world系下的位姿，也是body系到world系的变换矩阵，Twb = Tb->w
Eigen::Isometry3d Twa,Tca,Tbc,Twb,Twc;  //变换矩阵4*4
//Isometry3d Twa=Eigen::Isometry3d::Identity();
//Isometry3d Tca=Eigen::Isometry3d::Identity();
//Isometry3d Tbc=Eigen::Isometry3d::Identity();
//Isometry3d Twb=Eigen::Isometry3d::Identity();
//Isometry3d Twc=Eigen::Isometry3d::Identity();
//Tbc(0,0) =            0, Tbc(0,1) =           -1, Tbc(0,2) =            0, Tbc(0,3) =             0;
//Tbc(1,0) =           -1, Tbc(1,1) =            0, Tbc(1,2) =            0, Tbc(1,3) =             0;
//Tbc(2,0) =            0, Tbc(2,1) =            0, Tbc(2,2) =           -1, Tbc(2,3) =             0;
//Tbc(3,0) =            0, Tbc(3,1) =            0, Tbc(3,2) =            0, Tbc(3,3) =             1;


//Eigen::Matrix3d rotation_matrix_bc = Eigen::Matrix3d::Identity();
Eigen::Matrix3d rotation_matrix_bc,rotation_matrix_wb,rotation_matrix_ca;//旋转矩阵

Eigen::Vector3d t_bc,t_wb,t_ca;//平移向量

Quaterniond q_wb,q_ca;//四元数

//double drone_position_ENU[3]={0,0,0}; //无人机ENU世界系下位置

//double aruco_position_cam[3]={0,0,0}; //aruco二维码检测得到的位置也就是相机系下的位置

//double aruco_position_ENU[3]={0,0,0}; //aruco二维码在东北天世界系下的位置

/**
 * @brief 【示例必需】缓存飞控连接、解锁和模式状态，供 main 判断是否请求切模式/解锁。
 * @param aruco_msg mavros/state 的状态消息（名称虽有 aruco，内容是飞控状态）。
 * @return 无；更新全局 current_state。
 */
void state_cb(const mavros_aruco_msgs::State::ConstPtr& aruco_msg){
    current_state = *aruco_msg;
}

/**
 * @brief 【示例必需】根据无人机本地位姿构建机体系到世界系的变换 Twb。
 * @param aruco_msg /mavros/local_position/pose 的位置和四元数。
 * @return 无；更新 t_wb、q_wb 和 Twb，供到点判断和世界系目标计算使用。
 */
void pos_cb(const geometry_aruco_msgs::PoseStamped::ConstPtr &aruco_msg)
        {
            //drone_position_ENU[0] = aruco_msg->pose.position.x;
            //drone_position_ENU[1] = aruco_msg->pose.position.y;
            //drone_position_ENU[2] = aruco_msg->pose.position.z;

            t_wb << aruco_msg->pose.position.x ,aruco_msg->pose.position.y ,aruco_msg->pose.position.z;
            //在Eigen库中,四元数的存储顺序是w、x、y、z
            q_wb = Eigen::Quaterniond(aruco_msg->pose.orientation.w ,aruco_msg->pose.orientation.x ,aruco_msg->pose.orientation.y ,aruco_msg->pose.orientation.z);

            rotation_matrix_wb = q_wb.toRotationMatrix();//四元数转为旋转矩阵
            //Twb.rotate( rotation_matrix_wb );
            //Twb.pretranslate( t_wb );
            // 得到机体系到世界系的变换矩阵R、T
            Twb.linear() = rotation_matrix_wb;
            Twb.translation() = t_wb;
        }

/**
 * @brief 【示例必需】缓存标记相对相机的位姿，构建标记系到相机系的变换 Tca。
 * @param aruco_msg /aruco/pose，需与该示例声明的消息类型兼容。
 * @return 无；更新 t_ca、q_ca 和 Tca。
 * 注意：本示例未记录有效标志和接收时间，不能通过缓存值判断检测是否仍有效。
 */
void aruco_pos_cb(const geometry_aruco_msgs::PoseStamped::ConstPtr &aruco_msg)
        {
            //aruco_position_cam[0] = aruco_msg->pose.position.x;
            //aruco_position_cam[1] = aruco_msg->pose.position.y;
            //aruco_position_cam[2] = aruco_msg->pose.position.z;

            t_ca << aruco_msg->pose.position.x ,aruco_msg->pose.position.y ,aruco_msg->pose.position.z;

            q_ca = Eigen::Quaterniond(aruco_msg->pose.orientation.w ,aruco_msg->pose.orientation.x ,aruco_msg->pose.orientation.y ,aruco_msg->pose.orientation.z);
            rotation_matrix_ca = q_ca.toRotationMatrix();
            //Tca.rotate( rotation_matrix_ca );
            //Tca.pretranslate( t_ca );
            // 得到aruco到相机系
            Tca.linear() = rotation_matrix_ca;
            Tca.translation() = t_ca;
        }



/**
 * @brief 【参考入口】等待飞控、预发位置目标、请求 OFFBOARD/解锁，并按状态机对准和降落。
 * @param argc/argv ROS 启动参数；auto_arm_offboard 控制自动切模式/解锁分支。
 * @return 正常退出返回 0。
 * 状态：0 初始高度目标 -> 1 飞往搜索点 -> 2 对准 -> 3 逐步下降 -> 4 近地对准 -> 5 请求降落。
 * 这是无人机控制参考，不是当前 aruco_det.launch 的运行路径。
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    bool AUTO_ARM_OFFBOARD;

    nh1.param<bool>("auto_arm_offboard", AUTO_ARM_OFFBOARD, true);

    ros::Subscriber state_sub = nh.subscribe<mavros_aruco_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_aruco_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);  

    ros::Subscriber aruco_pose_sub = nh.subscribe<geometry_aruco_msgs::PoseStamped>("/aruco/pose", 10, aruco_pos_cb);






    // 【次要：未使用接口】local_pos_pub 创建后未 publish；实际使用下面的 raw/local 发布器。
    ros::Publisher local_pos_pub = nh.advertise<geometry_aruco_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_aruco_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_aruco_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_aruco_msgs::SetMode>
            ("mavros/set_mode");

    // 【示例核心】Tbc 表示固定安装关系；Twb*Tbc*Tca 将标记变换到世界坐标系。
    // 相机系到机体系的变换
    t_bc <<  0, 0, 0;
    rotation_matrix_bc << 0, -1, 0,
                         -1 , 0, 0,
                          0, 0, -1;

    Tbc.linear() = rotation_matrix_bc;
    Tbc.translation() = t_bc;
    t_ca << 0, 0, 0;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // 【示例启动流程】持续处理回调，直到收到飞控 connected 状态。
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
// 期望位置
    mavros_aruco_msgs::PositionTarget pos_setpoint;  //创建一个mavros_aruco_msgs::PositionTarget类型的实例化对象
    pos_setpoint.type_mask = 0b100111111000;  //T265 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    pos_setpoint.position.z = 2;
    pos_setpoint.yaw = 0;

    // 【示例启动流程】先连续发布初始设定值，再进入自动切模式和控制阶段。
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }
// 切模式
    mavros_aruco_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_aruco_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_aruco_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    int offboard_flag = 0;//防止切land降落之后又去解锁切offboard
    int land_flag = 0;
    // 【示例核心】state_flag 决定当前阶段；后面多个独立 if 允许同一轮继续进入下一阶段。
    int state_flag = 0;
    ros::Time time_snap;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // 【示例控制】按状态和请求间隔尝试 OFFBOARD/解锁；该开关不包围后面的状态机。
        if(AUTO_ARM_OFFBOARD==true)
        {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();//可以通过这个方式保持当前指点持续几秒钟啊。
            }
        }
        }
        // 【次要：空分支】false 分支没有执行内容。
        if(AUTO_ARM_OFFBOARD==false)
        {
        }

        //ros::Duration(1.0).sleep();
        // 【阶段 0 -> 1】接近初始 (0,0,2) 且满足请求时间间隔后，进入搜索点阶段。
        if((abs(t_wb[0])<0.2&&abs(t_wb[1])<0.2&&abs(t_wb[2]-2)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            state_flag = 1;
        }

        // 【阶段 1】飞向固定搜索点 (1.5,1.5,2)，到点并看到非零相机 Z 后进入对准。
        if( state_flag == 1)
        {
         std::cout << "state 1" << std::endl;
        //  给一定的误差
         pos_setpoint.position.x = 1.5;
         pos_setpoint.position.y = 1.5;
         pos_setpoint.position.z = 2; 
  
         if((abs(t_wb[0]-1.5)<0.2&&(abs(t_wb[1]-1.5)<0.2)&&abs(t_wb[2]-2)<0.2)&&(t_ca[2] != 0))
         {
            state_flag = 2;
         }
        }

        // 【阶段 2】用世界系标记位置更新 XY，高度设为标记上方 2 m；相机 XY 误差小则进入下降。
        if( state_flag == 2)
        {
         std::cout << "state 2" << std::endl;
         //aruco_position_ENU[0] = drone_position_ENU[0] - aruco_position_cam[1];
         //aruco_position_ENU[1] = drone_position_ENU[1] - aruco_position_cam[0];
         //aruco_position_ENU[2] = drone_position_ENU[2] - aruco_position_cam[2];

         //Twc = Twb * Tbc ;
         //Twa =Twc * Tca ;
         //Twa = Tca * Tbc * Twb; //注意这样是错的
         Twa = Twb * Tbc * Tca ;

         /***
         std::cout << "t_wb" << t_wb  << std::endl;
         std::cout << "Twb" << Twb.matrix() << std::endl;
         std::cout << "Tbc" << Tbc.matrix() << std::endl;
         std::cout << "t_ca" << t_wb  << std::endl;
         std::cout << "Tca" << Tca.matrix() << std::endl;
         std::cout << "Twc" << Twc.matrix() << std::endl;
         std::cout << "Twa" << Twa.matrix() << std::endl;
         ***/
         // 获取T中的平移向量
         Vector3d t_wa = Twa.translation();

         pos_setpoint.position.x = t_wa[0];
         pos_setpoint.position.y = t_wa[1];
         pos_setpoint.position.z = t_wa[2] + 2; 

         if((abs(t_ca[0])<0.1)&&(abs(t_ca[1])<0.1))
         {
            state_flag = 3;
            time_snap = ros::Time::now();
         }
        }

        // 【阶段 3】继续跟踪标记 XY，以 0.1 m/s 随时间降低高度目标；机体 Z<0.35 时进入阶段 4。
        if( state_flag == 3)
        {
         std::cout << "state 3" << std::endl;

         ros::Time now_time = ros::Time::now();
         int now_time_sec = now_time.sec;
         int now_time_nsec = now_time.nsec;
         double now_time_sec_sum = now_time_sec + now_time_nsec*(1e-9);

         int time_snap_sec = time_snap.sec;
         int time_snap_nsec = time_snap.nsec;
         double time_snap_sec_sum = time_snap_sec + time_snap_nsec*(1e-9);
         double now_time_relate_from_state2 = now_time_sec_sum - time_snap_sec_sum;


         //aruco_position_ENU[0] = drone_position_ENU[0] - aruco_position_cam[1];
         //aruco_position_ENU[1] = drone_position_ENU[1] - aruco_position_cam[0];
         //aruco_position_ENU[2] = drone_position_ENU[2] - aruco_position_cam[2];

         /***
         pos_setpoint.type_mask = 0b100111000100;
         pos_setpoint.coordinate_frame = 1;
         pos_setpoint.velocity.x = 0;
         pos_setpoint.velocity.y = 0;
         pos_setpoint.velocity.z = -0.1;
         pos_setpoint.position.x = aruco_position_ENU[0];
         pos_setpoint.position.y = aruco_position_ENU[1];
         pos_setpoint.yaw = 0;
         ***/

         //Twc = Twb * Tbc ;
         Twa = Twb * Tbc * Tca ;
         // 获取T中的平移向量
         Vector3d t_wa = Twa.translation();

         pos_setpoint.position.x = t_wa[0];
         pos_setpoint.position.y = t_wa[1];
         pos_setpoint.position.z = t_wa[2] + 2 - 0.1*now_time_relate_from_state2; 

         if(t_wb[2] < 0.35)
         {
            state_flag = 4;
         }
        }


         // 【阶段 4】设为标记上方 0.3 m，再次对准；满足相机 XY 误差条件则请求自动降落。
         if( state_flag == 4)
         {
         std::cout << "state 4" << std::endl;
         //aruco_position_ENU[0] = drone_position_ENU[0] - aruco_position_cam[1];
         //aruco_position_ENU[1] = drone_position_ENU[1] - aruco_position_cam[0];
         //aruco_position_ENU[2] = drone_position_ENU[2] - aruco_position_cam[2];

         //Twc = Twb * Tbc ;
         //Twa =Twc * Tca ;
         Twa = Twb * Tbc * Tca ;
         // 获取T中的平移向量
         Vector3d t_wa = Twa.translation();

         pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
         pos_setpoint.coordinate_frame = 1;

         pos_setpoint.position.x = t_wa[0];
         pos_setpoint.position.y = t_wa[1];
         pos_setpoint.position.z = t_wa[2] + 0.3; 
         pos_setpoint.yaw = 0;
        
         if((abs(t_ca[0])<0.1)&&(abs(t_ca[1])<0.1))
         {
            state_flag = 5;
         } 
        }

         // 【阶段 5】调用模式服务请求 AUTO.LAND，直到飞控状态反馈该模式。
         if( state_flag == 5)
         {
         if( current_state.mode != "AUTO.LAND" )
         {
         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");
             offboard_flag = 1;    
           }  
          }
         }

        // 【示例核心输出】持续发送本轮位置目标，并处理回调更新飞控与视觉信息。
        setpoint_raw_local_pub.publish(pos_setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



