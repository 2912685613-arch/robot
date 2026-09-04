#ifndef C_SAVE_DATA_H
#define C_SAVE_DATA_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <cmath>
#include <sstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace std;

namespace SaveData
{

    class C_SaveData
    {
    public:
        bool write_head_flag;
        ofstream fout;

    public:
        C_SaveData();
        C_SaveData(string _path);

        string gettime();
        void write_head();
        void write_data(); // 包含 write_head 操作
        void write_data(int _self_id, float _robot_pos_x, float _robot_pos_y, float _robot_pos_z, float _robot_yaw);
        void write_data(float _robot_pos_x, float _robot_pos_y, float _robot_velocity_x, float _x_linear_acceleration,
                        float _robot_yaw, float _yaw_velocity, float _force_x, float _force_yaw);
        void write_data(float _robot_pos_x, float _robot_pos_y, float _world_velocity_x, float _world_velocity_y,
                        float _robot_yaw, float _yaw_velocity, float _force_1, float _force_2, float _force_3, float _force_4);
        void write_data(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                        float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                        float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw);

        void write_data(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                        float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                        float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw,
                        float _robot4_pos_x, float _robot4_pos_y, float _robot4_pos_z, float _robot4_yaw,
                        float _robot5_pos_x, float _robot5_pos_y, float _robot5_pos_z, float _robot5_yaw);

        void write_data(int iternum, Eigen::VectorXd X_piao, Eigen::VectorXd force);
        void write_data(int iternum, Eigen::VectorXd x_real_current);

        // hcy
        void write_data_3(float _robot_pos_x, float _robot_pos_y, float _robot_pos_z,
                          float _robot_roll, float _robot_pitch, float _robot_yaw,
                          float _robot_depth_sensor,
                          float _robot_traj_radius,
                          float _robot_ref_pos_x, float _robot_ref_pos_y, float _robot_ref_pos_z, float _robot_dest_yaw,
                          float _force_x, float _torque_yaw, float _force_z,
                          float _thrust_force_1, float _thrust_force_2, float _thrust_force_3, float _thrust_force_4,
                          float _leg_angle_1, float _leg_angle_2, float _leg_angle_3, float _leg_angle_4);
    };

}

#endif // C_SAVE_DATA_H
