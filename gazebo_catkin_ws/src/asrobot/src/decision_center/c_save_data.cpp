#include "../../include/asrobot/c_save_data.h"

namespace SaveData {
    C_SaveData::C_SaveData()
    {
        write_head_flag = true;
        fout = ofstream("/home/catkin_ws/src/data.txt");
    }

    C_SaveData::C_SaveData(string _path )
    {
        write_head_flag = true;
        fout = ofstream(_path);
    }

    string C_SaveData::gettime()
    {
        time_t timep;
        time (&timep);
        char tmp[64];
        strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
        return tmp;

    }

    void C_SaveData::write_head()
    {
        fout << "robot1_pos_x\t" << "robot1_pos_y\t" << "robot1_pos_z\t" << "robot1_depth\t";
        fout << "robot2_pos_x\t" << "robot2_pos_y\t" << "robot2_pos_z\t" << "robot2_depth\t";
        fout << "robot3_pos_x\t" << "robot3_pos_y\t" << "robot3_pos_z\t" << "robot3_depth\t";
        fout << "robot3_roll\t"  << "robot3_yaw\t"   << "robot3_pitch\t";
        fout << "robot4_pos_x\t" << "robot4_pos_y\t" << "robot4_pos_z\t" << "robot4_depth\t";
        fout << "robot5_pos_x\t" << "robot5_pos_y\t" << "robot5_pos_z\t" << "robot5_depth\t";

    }

    void C_SaveData::write_data()
    {
        if(write_head_flag ==  true)
        {
            write_head();
            write_head_flag = false;
        }
        else
        {
            fout << gettime() << "\t";
            fout << "robot1_pos_x\t" << "robot1_pos_y\t" << "robot1_pos_z\t" << "robot1_depth\t";
            fout << "robot2_pos_x\t" << "robot2_pos_y\t" << "robot2_pos_z\t" << "robot2_depth\t";
            fout << "robot3_pos_x\t" << "robot3_pos_y\t" << "robot3_pos_z\t" << "robot3_depth\t";
            fout << "robot3_roll\t"  << "robot3_yaw\t"   << "robot3_pitch\t";
            fout << "robot4_pos_x" << "\t" << "robot4_pos_y" << "\t" << "robot4_pos_z" << "\t" << "robot4_depth" << "\t";
            fout << "robot5_pos_x" << "\t" << "robot5_pos_y" << "\t" << "robot5_pos_z" << "\t" << "robot5_depth" << "\t";
        }
    }


    void C_SaveData::write_data(int _self_id, float _robot_pos_x, float _robot_pos_y, float _robot_pos_z, float _robot_yaw)
    {
	fout << gettime() << "\t";
        switch(_self_id)
        {
        case 1:
        {
            if(write_head_flag == true)
            {
                fout << "robot1_pos_x" <<"      "<<"robot1_pos_y"<< "      "<< "robot1_pos_z"<<"      "<< "robot1_yaw"<<endl;
                write_head_flag = false;
            }
            else
            {
                fout << _robot_pos_x << "       "<<_robot_pos_y << "      " << _robot_pos_z << "      "<<_robot_yaw << endl;
            }
        };break;

        case 2:
        {
            if(write_head_flag == true)
            {
                fout << "robot2_pos_x" <<"      "<<"robot2_pos_y"<< "      "<< "robot2_pos_z"<<"      "<< "robot2_yaw"<<endl;
                write_head_flag = false;
            }
            else
            {
                fout << _robot_pos_x << "       "<<_robot_pos_y << "      " << _robot_pos_z << "      "<<_robot_yaw << endl;
            }

        };break;

        case 3:
        {
            if(write_head_flag == true)
            {
                fout << "robot3_pos_x" <<"      "<<"robot3_pos_y"<< "      "<< "robot3_pos_z"<<"      "<< "robot3_yaw"<<endl;
                write_head_flag = false;
            }
            else
            {
                fout << _robot_pos_x << "       "<<_robot_pos_y << "      " << _robot_pos_z << "      "<<_robot_yaw << endl;
            }
        };break;

        case 4:
        {
            if(write_head_flag == true)
            {
                fout << "robot4_pos_x" <<"      "<<"robot4_pos_y"<< "      "<< "robot4_pos_z"<<"      "<< "robot4_yaw"<<endl;
                write_head_flag = false;
            }
            else
            {
                fout << _robot_pos_x << "       "<<_robot_pos_y << "      " << _robot_pos_z << "      "<<_robot_yaw << endl;
            }
        };break;

        case 5:
        {
            if(write_head_flag == true)
            {
                fout << "robot5_pos_x" <<"      "<<"robot5_pos_y"<< "      "<< "robot5_pos_z"<<"      "<< "robot5_yaw"<<endl;
                write_head_flag = false;
            }
            else
            {
                fout << _robot_pos_x << "       "<<_robot_pos_y << "      " << _robot_pos_z << "      "<<_robot_yaw << endl;
            }
        };break;
        default:
        {
            cout << " unidentified id !!!!!! "<<endl;
        }
        }

}
    void C_SaveData::write_data(float _robot_pos_x, float _robot_pos_y, float _robot_velocity_x, float _x_linear_acceleration,
    float _robot_yaw,float _yaw_velocity,float _force_x,float _force_yaw)
{
    if(write_head_flag == true)
        {
            fout << "pos_x" <<" "<<"pos_y"<<" "<<"velocity_x"<<" "<< "x_linear_acceleration"<<" "<< "yawangle" <<" "<<"yaw_velocity"<<" "<<"_force_x"<<" "<<"_force_yaw"<<endl;
            write_head_flag = false;
        }
        else
        {
            fout << _robot_pos_x <<" "<<_robot_pos_y <<" " << _robot_velocity_x<< " "<<_x_linear_acceleration<<" "<<_robot_yaw<<" "<<_yaw_velocity<<" "<<_force_x<<" "<<_force_yaw<<endl;
                  
        }
}
 void C_SaveData::write_data(float _robot_pos_x, float _robot_pos_y, float _world_velocity_x, float _world_velocity_y,
        float _robot_yaw,float _yaw_velocity,float _force_1,float _force_2,float _force_3,float _force_4)
{
    if(write_head_flag == true)
        {
            fout << "pos_x" <<" "<<"pos_y"<<" "<<"world_velocity_x"<<" "<< "world_velocity_y"<<" "<< "yawangle" <<" "<<
                    "yaw_velocity"<<" "<<"_force_1"<<" "<<"_force_2"<<" "<<"_force_3"<<" "<<"_force_4"<<endl;
            write_head_flag = false;
        }
        else
        {
            fout << _robot_pos_x <<" "<<_robot_pos_y <<" "<< _world_velocity_x<<" "
            <<_world_velocity_y<<" "<<_robot_yaw<<" "<<_yaw_velocity<<" "
            <<_force_1<<" "<<_force_2<<" "<<_force_3<<" "<<_force_4<<endl;
                  
        }
} 


    void C_SaveData::write_data(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                                float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                                float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw)
    {

        if(write_head_flag == true)
        {
            fout << "robot1_pos_x" <<"      "<<"robot1_pos_y"<<"      "<<"robot1_pos_z"<<"      "<< "robot1_yaw"
                 << "robot2_pos_x" <<"      "<<"robot2_pos_y"<<"      "<<"robot2_pos_z"<<"      "<< "robot2_yaw"
                 << "robot3_pos_x" <<"      "<<"robot3_pos_y"<<"      "<<"robot3_pos_z"<<"      "<< "robot3_yaw" << endl;
            write_head_flag = false;
        }
        else
        {
            fout << _robot1_pos_x << "      "<<_robot1_pos_y << "      " << _robot1_pos_z << "      "<<_robot1_yaw
                 << _robot2_pos_x << "      "<<_robot2_pos_y << "      " << _robot2_pos_z << "      "<<_robot2_yaw
                 << _robot3_pos_x << "      "<<_robot3_pos_y << "      " << _robot3_pos_z << "      "<<_robot3_yaw << endl;
        }
    }

    void C_SaveData::write_data(float _robot1_pos_x, float _robot1_pos_y, float _robot1_pos_z, float _robot1_yaw,
                                float _robot2_pos_x, float _robot2_pos_y, float _robot2_pos_z, float _robot2_yaw,
                                float _robot3_pos_x, float _robot3_pos_y, float _robot3_pos_z, float _robot3_yaw,
                                float _robot4_pos_x, float _robot4_pos_y, float _robot4_pos_z, float _robot4_yaw,
                                float _robot5_pos_x, float _robot5_pos_y, float _robot5_pos_z, float _robot5_yaw)
    {
        if(write_head_flag == true)
        {
            fout << "robot1_pos_x" <<"      "<<"robot1_pos_y"<<"      "<<"robot1_pos_z"<<"      "<< "robot1_yaw"
                 << "robot2_pos_x" <<"      "<<"robot2_pos_y"<<"      "<<"robot2_pos_z"<<"      "<< "robot2_yaw"
                 << "robot3_pos_x" <<"      "<<"robot3_pos_y"<<"      "<<"robot3_pos_z"<<"      "<< "robot3_yaw"
                 << "robot4_pos_x" <<"      "<<"robot4_pos_y"<<"      "<<"robot4_pos_z"<<"      "<< "robot4_yaw"
                 << "robot5_pos_x" <<"      "<<"robot5_pos_y"<<"      "<<"robot5_pos_z"<<"      "<< "robot5_yaw" << endl;
            write_head_flag = false;
        }
        else
        {
            fout << _robot1_pos_x << "      "<<_robot1_pos_y << "      " << _robot1_pos_z << "      "<<_robot1_yaw
                 << _robot2_pos_x << "      "<<_robot2_pos_y << "      " << _robot2_pos_z << "      "<<_robot2_yaw
                 << _robot3_pos_x << "      "<<_robot3_pos_y << "      " << _robot3_pos_z << "      "<<_robot3_yaw
                 << _robot4_pos_x << "      "<<_robot4_pos_y << "      " << _robot4_pos_z << "      "<<_robot4_yaw
                 << _robot5_pos_x << "      "<<_robot5_pos_y << "      " << _robot5_pos_z << "      "<<_robot5_yaw << endl;
        }

    }










}
