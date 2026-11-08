#include "../../include/asrobot/c_vectoredthrustcontrol.h"

// 从 motion.cpp扩展过来
extern asrobot::pwm pwm_msg;
extern ros::Publisher *extern_pub_pwm;

// leg shape constant
const int SHAPE_H = 1;
const int SHAPE_H_T = 2;
const int SHAPE_X = 3;
const int SHAPE_G = 4;

/////hcy
const float base_motor_force = 1.6; // 电机推力基值
const float bias_motor_force = 0.6; // 电机推力偏移量

const float min_motor_speed = 240, max_motor_speed = 360; // 最小最大电机转速

// // x模式下电机最小最大转速
// const float x_shape_min_motor_speed1 = 1000, x_shape_max_motor_speed1 = 1500;
// const float x_shape_min_motor_speed2 = 1000, x_shape_max_motor_speed2 = 1500;
// const float x_shape_min_motor_speed3 = 1000, x_shape_max_motor_speed3 = 1500;
// const float x_shape_min_motor_speed4 = 1000, x_shape_max_motor_speed4 = 1500;

// // H模式下电机最小最大转速
// const float h_shape_min_motor_speed1 = 1000, h_shape_max_motor_speed1 = 1500;
// const float h_shape_min_motor_speed2 = 1000, h_shape_max_motor_speed2 = 1500;
// const float h_shape_min_motor_speed3 = 1000, h_shape_max_motor_speed3 = 1500;
// const float h_shape_min_motor_speed4 = 1000, h_shape_max_motor_speed4 = 1500;
// x模式下电机最小最大转速
// const float x_shape_min_motor_speed1 = 100, x_shape_max_motor_speed1 = 250;
// const float x_shape_min_motor_speed2 = 100, x_shape_max_motor_speed2 = 250;
// const float x_shape_min_motor_speed3 = 100, x_shape_max_motor_speed3 = 250;
// const float x_shape_min_motor_speed4 = 100, x_shape_max_motor_speed4 = 250;

// // H模式下电机最小最大转速
// const float h_shape_min_motor_speed1 = 100, h_shape_max_motor_speed1 = 250;
// const float h_shape_min_motor_speed2 = 100, h_shape_max_motor_speed2 = 250;
// const float h_shape_min_motor_speed3 = 100, h_shape_max_motor_speed3 = 250;
// const float h_shape_min_motor_speed4 = 100, h_shape_max_motor_speed4 = 250;
// const float x_shape_min_motor_speed1 = 250, x_shape_max_motor_speed1 = 400;
// const float x_shape_min_motor_speed2 = 250, x_shape_max_motor_speed2 = 400;
// const float x_shape_min_motor_speed3 = 250, x_shape_max_motor_speed3 = 400;
// const float x_shape_min_motor_speed4 = 250, x_shape_max_motor_speed4 = 400;

// // H模式下电机最小最大转速
// const float h_shape_min_motor_speed1 = 250, h_shape_max_motor_speed1 = 400;
// const float h_shape_min_motor_speed2 = 250, h_shape_max_motor_speed2 = 400;
// const float h_shape_min_motor_speed3 = 250, h_shape_max_motor_speed3 = 400;
// const float h_shape_min_motor_speed4 = 250, h_shape_max_motor_speed4 = 400;

//////85
// // x模式下电机最小最大转速
// const float x_shape_min_motor_speed1 = 0, x_shape_max_motor_speed1 = 100;
// const float x_shape_min_motor_speed2 = 0, x_shape_max_motor_speed2 = 100;
// const float x_shape_min_motor_speed3 = 0, x_shape_max_motor_speed3 = 100;
// const float x_shape_min_motor_speed4 = 0, x_shape_max_motor_speed4 = 100;

// // H模式下电机最小最大转速
// const float h_shape_min_motor_speed1 = 0, h_shape_max_motor_speed1 = 100;
// const float h_shape_min_motor_speed2 = 0, h_shape_max_motor_speed2 = 100;
// const float h_shape_min_motor_speed3 = 0, h_shape_max_motor_speed3 = 100;
// const float h_shape_min_motor_speed4 = 0, h_shape_max_motor_speed4 = 100;
///////

const float x_shape_min_motor_speed1 = 0, x_shape_max_motor_speed1 = 100;
const float x_shape_min_motor_speed2 = 0, x_shape_max_motor_speed2 = 100;
const float x_shape_min_motor_speed3 = 0, x_shape_max_motor_speed3 = 100;
const float x_shape_min_motor_speed4 = 0, x_shape_max_motor_speed4 = 100;

// H模式下电机最小最大转速
const float h_shape_min_motor_speed1 = 25, h_shape_max_motor_speed1 = 75;
const float h_shape_min_motor_speed2 = 25, h_shape_max_motor_speed2 = 75;
const float h_shape_min_motor_speed3 = 25, h_shape_max_motor_speed3 = 75;
const float h_shape_min_motor_speed4 = 25, h_shape_max_motor_speed4 = 75;

const float g_shape_min_motor_speed1 = 0, g_shape_max_motor_speed1 = 100;
const float g_shape_min_motor_speed2 = -100, g_shape_max_motor_speed2 = 100;
const float g_shape_min_motor_speed3 = 0, g_shape_max_motor_speed3 = 100;
const float g_shape_min_motor_speed4 = -100, g_shape_max_motor_speed4 = 100;

double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void convertToObliqueCoordinates(
    double force1_direction_deg,  // 斜角坐标系x轴方向（度）
    double object_direction_deg,  // 直角坐标系中向量方向（度）
    double controller_forward,    // 向量模
    double& x_prime,              // 斜角坐标系中的x坐标
    double& y_prime               // 斜角坐标系中的y坐标
) {
    // 将角度转换为弧度
    double force1_dir_rad = deg2rad(force1_direction_deg);
    double object_dir_rad = deg2rad(object_direction_deg);

    // 计算斜角坐标系的基向量
    double e1_x = cos(force1_dir_rad);          // x轴基向量
    double e1_y = sin(force1_dir_rad);
    double e2_dir_rad = force1_dir_rad + deg2rad(130.0);  // y轴方向为x轴方向+130度
    double e2_x = cos(e2_dir_rad);              // y轴基向量
    double e2_y = sin(e2_dir_rad);

    // 计算行列式（det = e1_x * e2_y - e2_x * e1_y）
    double det = e1_x * e2_y - e2_x * e1_y;
    if (std::abs(det) < 1e-10) {
        std::cerr << "Error: 基向量线性相关，无法构造斜角坐标系！" << std::endl;
        x_prime = 0.0;
        y_prime = 0.0;
        return;
    }

    // 计算直角坐标系中向量a的坐标
    double a_x = controller_forward * cos(object_dir_rad);
    double a_y = controller_forward * sin(object_dir_rad);

    // 计算逆矩阵并转换坐标
    x_prime = (e2_y * a_x - e2_x * a_y) / det;
    y_prime = (-e1_y * a_x + e1_x * a_y) / det;
}

namespace VectoredThrustControl
{

    // speed 初始化
    Speed::Speed()
    {
        x = 0;
        y = 0;
        z = 0;
        yaw = 0;
    }

    // Attitude 初始化
    Attitude::Attitude()
    {
        pitch = 0;
        roll = 0;
        yaw = 0;
    }

    Position::Position()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    // 对C_VectoredThrustControl进行初始化

    C_VectoredThrustControl::C_VectoredThrustControl()
    {
        legstatus = FB;
        fmcYawpidError = 0;
        fmcYawpidValue = 0;
        fmcDepthpidError = 0;
        fmcDepthpidValue = 0;
        fmcSpeedpidError = 0;
        fmcSpeedpidValue = 0;

        leg1_angle_filter.setNoiseRatio(50);
        leg2_angle_filter.setNoiseRatio(50);
        leg3_angle_filter.setNoiseRatio(50);
        leg4_angle_filter.setNoiseRatio(50);
    }

    // 驱动方式旧版本
    void C_VectoredThrustControl::motorforce(float _force_x, float _force_y, float _force_z,
                                             float _torque_z,
                                             float _roll, float _yaw, float _pitch)
    {
        pwm_msg.force_x = _force_x;
        pwm_msg.force_y = _force_y;
        pwm_msg.force_z = _force_z;
        pwm_msg.torque_z = _torque_z;
        pwm_msg.attitude_roll = _roll;
        pwm_msg.attitude_yaw = _yaw;
        pwm_msg.attitude_pitch = _pitch;

        extern_pub_pwm->publish(pwm_msg);

        // 腿部实时摆动

        float angle1, angle2, angle3, angle4;                                                                         // 比例因子
        angle1 = 0.25 * atan((_force_z + 0.65) / (sqrt(pow(_force_x, 2) + pow(_force_y, 2)) + 0.0000001)) * 180 / PI; // force_z=0.65合力为零
        angle2 = 0.25 * atan((_force_z + 0.65) / (sqrt(pow(_force_x, 2) + pow(_force_y, 2)) + 0.0000001)) * 180 / PI; // force_z=0.65合力为零
        angle3 = 0.25 * atan((_force_z + 0.65) / (sqrt(pow(_force_x, 2) + pow(_force_y, 2)) + 0.0000001)) * 180 / PI; // force_z=0.65合力为零
        angle4 = 0.25 * atan((_force_z + 0.65) / (sqrt(pow(_force_x, 2) + pow(_force_y, 2)) + 0.0000001)) * 180 / PI; // force_z=0.65合力为零

        angle1 = leg1_angle_filter.CalFilterOutput(angle1);
        angle2 = leg1_angle_filter.CalFilterOutput(angle2);
        angle3 = leg1_angle_filter.CalFilterOutput(angle3);
        angle4 = leg1_angle_filter.CalFilterOutput(angle4);

        Leg1.set_angle(Leg1.A_now, Leg1.B_now, -18 + angle1, 10, 0);
        Leg2.set_angle(Leg2.A_now, Leg2.B_now, -18 + angle2, 10, 0);
        Leg3.set_angle(Leg3.A_now, Leg3.B_now, -18 + angle3, 10, 0);
        Leg4.set_angle(Leg4.A_now, Leg4.B_now, -18 + angle4, 10, 0);
    }

    // H型变换
    void C_VectoredThrustControl::yawLocomMode_FB()
    {
        Leg1.set_angle(45, 0, -10, 500);
        Leg2.set_angle(45, 0, -10, 500);
        Leg3.set_angle(45, 0, -10, 500);
        Leg4.set_angle(45, 0, -10, 500);
    }

    // 工字型
    void C_VectoredThrustControl::yawLocomMode_LR()
    {
        Leg1.set_angle(-40, 0, -18, 500);
        Leg2.set_angle(-40, 0, -18, 500);
        Leg3.set_angle(-40, 0, -18, 500);
        Leg4.set_angle(-40, 0, -18, 500);
    }

    // X型
    void C_VectoredThrustControl::yawLocomMode_X()
    {
        Leg1.set_angle(0, 0, -18, 500);
        Leg2.set_angle(0, 0, -18, 500);
        Leg3.set_angle(0, 0, -18, 500);
        Leg4.set_angle(0, 0, -18, 500);
    }

    void C_VectoredThrustControl::shiftMode_G()
    {
        Leg1.set_angle(0, 110, -110, 500);
        Leg2.set_angle(40, 0, 0, 500);
        Leg3.set_angle(0, 110, -110, 500);
        Leg4.set_angle(40, 0, 0, 500);
    }

    bool C_VectoredThrustControl::judgeDirection(double _expectValue, double _realValue)
    {
        float expectValue;

        if (_expectValue > 0)
        {
            expectValue = _expectValue - 180;
            if (_realValue > expectValue && _realValue < _expectValue)
            {
                fmcYawpidError = -_expectValue + _realValue;
                return 0;
            }
            else
            {
                if (_realValue >= _expectValue)
                    fmcYawpidError = _realValue - _expectValue;
                else

                    fmcYawpidError = 360 - keepPositiveValue(_realValue) - _expectValue;
                return 1;
            }
        }
        else
        {
            expectValue = _expectValue + 180;
            if (_realValue < expectValue && _realValue > _expectValue)
            {
                fmcYawpidError = _realValue - _expectValue;
                return 1;
            }
            else
            {
                if (_realValue <= _expectValue)
                    fmcYawpidError = -_expectValue + _realValue;
                else
                    fmcYawpidError = -360 + keepPositiveValue(_expectValue) + keepPositiveValue(_realValue);
                return 0;
            }
        }
    }

    // 取绝对值
    double C_VectoredThrustControl::keepPositiveValue(double _data)
    {
        return _data > 0 ? _data : (-_data);
    }

    // 全方位矢量推进控制
    void C_VectoredThrustControl::attitudecontrol(Attitude _e_attitude, Speed _e_speed, float _e_depth,
                                                  Attitude _c_attitude, Speed _c_speed, float _c_depth)
    {
        /********************caculate yaw error **********************/
        judgeDirection(_e_attitude.yaw, _c_attitude.yaw);
        if (fmcYawpidError >= 180)
        {
            fmcYawpidError = 0;
        }
        if (fmcYawpidError <= -180)
        {
            fmcYawpidError = 0;
        }
        /************************yaw pid caculate********************/
        mcYawPID.i_limit = 200;
        double yawforce = mcYawPID.pidNewCalc(fmcYawpidError);

        /********************caculate pitch error **********************/
        double PitchError = _c_attitude.pitch - _e_attitude.pitch;
        /********************pitch pid caculate **********************/
        double pitchforce = mcPitchPID.pidNewCalc(PitchError);

        /********************caculate speedx error **********************/
        double SpeedxError = _e_speed.x - _c_speed.x;
        if (fmcYawpidError > 3 || fmcYawpidError < -3)
        {
            SpeedxError = _c_speed.x;
        }
        /********************speedx pid caculate **********************/
        double speedxforce = mcSpeedPID.pidNewCalc(SpeedxError);
        // speedxforce = -1;

        /********************caculate depth error **********************/
        double DepthError = _c_depth - _e_depth;
        /********************depth pid caculate **********************/
        mcYawPID.i_limit = 200;
        double depthforce = mcDepthPID.pidNewCalc(DepthError);
        printf("c_d:%.2f e_d:%.2f\n", _c_depth, _e_depth);

        /*************caculate force and angle of every motor************/
        double force1, force2, force3, force4;
        double angle1, angle2, angle3, angle4;
        double fx1, fx2, fx3, fx4;
        double fz1, fz2, fz3, fz4;
        double fmin, fmax1, fmax2;
        double force_x, force_y, force_z, torque_z;
        // fx
        fx1 = 0.5 * yawforce;
        fx2 = -0.5 * speedxforce;
        fx3 = 0.5 * (yawforce - speedxforce);
        fx4 = 0;

        fmin = fx1;
        if (fmin > fx2)
            fmin = fx2;
        if (fmin > fx3)
            fmin = fx3;
        if (fmin > fx4)
            fmin = fx4;
        fx1 -= (fmin - 0.4);
        fx2 -= (fmin - 0.4);
        fx3 -= (fmin - 0.4);
        fx4 -= (fmin - 0.4);

        // fz
        fz1 = 0.5 * (depthforce - pitchforce);
        fz2 = 0.5 * (pitchforce - 0);
        fz3 = 0.5 * (depthforce + 0);
        fz4 = 0;

        fmax1 = (fz1 > fz3) ? fz1 : fz3;
        fmax2 = (fz2 > fz4) ? fz2 : fz4;
        fz4 = 0.5 * (fmax1 - fmax2);

        fz1 = fz1 - fz4;
        fz2 = fz2 + fz4;
        fz3 = fz3 - fz4;

        // motor force and angle
        force1 = pow(pow(fx1, 2) + pow(fz1, 2), 0.5);
        force2 = pow(pow(fx2, 2) + pow(fz2, 2), 0.5);
        force3 = pow(pow(fx3, 2) + pow(fz3, 2), 0.5);
        force4 = pow(pow(fx4, 2) + pow(fz4, 2), 0.5);

        angle1 = atan(fz1 / (fx1 + 0.0000001)) * 180 / 3.1415926; // make sure denominator is not 0
        angle2 = atan(fz2 / (fx2 + 0.0000001)) * 180 / 3.1415926;
        angle3 = atan(fz3 / (fx3 + 0.0000001)) * 180 / 3.1415926;
        angle4 = atan(fz4 / (fx4 + 0.0000001)) * 180 / 3.1415926;

        /*******************angle limit*************************/
        float angle_min, angle_limit = -30;
        int num = 1;
        angle_min = angle1;
        if (angle2 < angle_min)
        {
            angle_min = angle2;
            num = 2;
        }
        if (angle3 < angle_min)
        {
            angle_min = angle3;
            num = 3;
        }
        if (angle4 < angle_min)
        {
            angle_min = angle4;
            num = 4;
        }

        if (angle_min < angle_limit) // changle angle
        {
            double deta = 0;
            switch (num)
            {
            case 1:
                deta = fz1 / tan(angle_limit * 3.14159265 / 180) - fx1;
                break;
            case 2:
                deta = fz2 / tan(angle_limit * 3.14159265 / 180) - fx2;
                break;
            case 3:
                deta = fz3 / tan(angle_limit * 3.14159265 / 180) - fx3;
                break;
            case 4:
                deta = fz4 / tan(angle_limit * 3.14159265 / 180) - fx4;
                break;
            }
            fx1 = fx1 + deta;
            fx2 = fx2 + deta;
            fx3 = fx3 + deta;
            fx4 = fx4 + deta;

            // recaculate motor force and angle
            force1 = pow(pow(fx1, 2) + pow(fz1, 2), 0.5);
            force2 = pow(pow(fx2, 2) + pow(fz2, 2), 0.5);
            force3 = pow(pow(fx3, 2) + pow(fz3, 2), 0.5);
            force4 = pow(pow(fx4, 2) + pow(fz4, 2), 0.5);

            angle1 = atan(fz1 / (fx1 + 0.0000001)) * 180 / 3.1415926; // make sure denominator is not 0
            angle2 = atan(fz2 / (fx2 + 0.0000001)) * 180 / 3.1415926;
            angle3 = atan(fz3 / (fx3 + 0.0000001)) * 180 / 3.1415926;
            angle4 = atan(fz4 / (fx4 + 0.0000001)) * 180 / 3.1415926;
        }

        /******************force limit**********************/
        double force_max, force_limit = 3.8;
        force_max = force1;
        num = 1;
        if (force2 > force_max)
        {
            force_max = force2;
            num = 2;
        }
        if (force3 > force_max)
        {
            force_max = force3;
            num = 3;
        }
        if (force4 > force_max)
        {
            force_max = force4;
            num = 4;
        }

        if (force_max > force_limit) // change force
        {
            double k;
            switch (num)
            {
            case 1:
                k = force_limit / force1;
                break;
            case 2:
                k = force_limit / force2;
                break;
            case 3:
                k = force_limit / force3;
                break;
            case 4:
                k = force_limit / force4;
                break;
            }

            force1 = force1 * k;
            force2 = force2 * k;
            force3 = force3 * k;
            force4 = force4 * k;
        }

        if (mcYawPID.enable)
        {
            printf("set yaw:%.2f   yaw error:%.2f   yawforce:%.4f\n", _e_attitude.yaw, fmcYawpidError, yawforce);
        }
        if (mcPitchPID.enable)
        {
            printf("pitch error:%.2f    pitchforce:%.4f\n", PitchError, pitchforce);
        }
        if (mcSpeedPID.enable)
        {
            printf("speed error:%.2f    speedxforce:%.4f\n", SpeedxError, speedxforce);
        }
        if (mcDepthPID.enable)
        {
            printf("depth error:%.2f    depthforce:%.4f\n", DepthError, depthforce);
        }
        //    printf("fx1:%.4f     fx2:%.4f     fx3:%.4f     fx4:%.4f  \n", fx1,fx2,fx3,fx4);
        //    printf("fz1:%.4f     fz2:%.4f     fz3:%.4f     fz4:%.4f  \n", fz1,fz2,fz3,fz4);
        //    printf("force1:%.4f  force2:%.4f  force3:%.4f  force4:%.4f  \n", force1,force2,force3,force4);
        //    printf("angle1:%.4f  angle2:%.4f  angle3:%.4f  angle4:%.4f  \n", angle1,angle2,angle3,angle4);

        // force output
        if (legstatus == LR)
        {
            force_y = fx3 + fx4 - fx1 - fx2;
            force_z = fz1 + fz4 + fz2 + fz3;
            torque_z = (fx2 + fx4 - fx1 - fx3) * 0.02758; // caculate resultant force and torque

            motorforce(0, force_y, force_z, torque_z, _c_attitude.roll, _c_attitude.yaw, _c_attitude.pitch);
            Leg1.set_angle(Leg1.A_now, Leg1.B_now, -18 + angle2, 10, 0);
            Leg2.set_angle(Leg2.A_now, Leg2.B_now, -18 + angle3, 10, 0);
            Leg3.set_angle(Leg3.A_now, Leg3.B_now, -18 + angle4, 10, 0);
            Leg4.set_angle(Leg4.A_now, Leg4.B_now, -18 + angle1, 10, 0);
        }
        else if (legstatus == FB)
        {
            force_x = fx1 + fx4 - fx2 - fx3;
            force_z = fz1 + fz4 + fz2 + fz3;
            torque_z = (fx2 + fx4 - fx1 - fx3) * 0.02758;

            motorforce(force_x, 0, force_z, torque_z, _c_attitude.roll, _c_attitude.yaw, _c_attitude.pitch);
            Leg1.set_angle(Leg1.A_now, Leg1.B_now, -18 + angle1, 10, 0);
            Leg2.set_angle(Leg2.A_now, Leg2.B_now, -18 + angle2, 10, 0);
            Leg3.set_angle(Leg3.A_now, Leg3.B_now, -18 + angle3, 10, 0);
            Leg4.set_angle(Leg4.A_now, Leg4.B_now, -18 + angle4, 10, 0);
        }

        printf("\n");
    }

    // 驱动方式新版本

    void C_VectoredThrustControl::thrusterforce2speed(float _force1, float _force2, float _force3, float _force4)
    {

        // conversation function from force to ang_vel
        pwm_msg.rotor_speed1 = _force1 * 25;
        pwm_msg.rotor_speed2 = _force2 * 25;
        pwm_msg.rotor_speed3 = _force3 * 25;
        pwm_msg.rotor_speed4 = _force4 * 25;

        extern_pub_pwm->publish(pwm_msg);
    }

    /*
    //hcy:gazebo原程序使用
    // H shape with leg angle///
    void C_VectoredThrustControl::yawLocomMode_FB(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle)
    {
        if(_leg1_angle > 30)
        {
            _leg1_angle = 30;
        }
        else if(_leg1_angle < -30)
        {
            _leg1_angle = -30;
        }

        if(_leg2_angle > 30)
        {
            _leg2_angle = 30;
        }
        else if(_leg2_angle < -30)
        {
            _leg2_angle = -30;
        }

        if(_leg3_angle > 30)
        {
            _leg3_angle = 30;
        }
        else if(_leg3_angle < -30)
        {
            _leg3_angle = -30;
        }

        if(_leg4_angle > 30)
        {
            _leg4_angle = 30;
        }
        else if(_leg4_angle < -30)
        {
            _leg4_angle = -30;
        }

        // Leg1.set_angle(45, 0, -10 + _leg1_angle, 500);
        // Leg2.set_angle(45, 0, -10 + _leg2_angle, 500);
        // Leg3.set_angle(45, 0, -10 + _leg3_angle, 500);
        // Leg4.set_angle(45, 0, -10 + _leg4_angle, 500);
        Leg1.set_angle(40, 0, 4+_leg1_angle, 500);
        Leg2.set_angle(40, 0, 4+_leg2_angle, 500);
        Leg3.set_angle(40, 0, 4+_leg3_angle, 500);
        Leg4.set_angle(40, 0, 4+_leg4_angle, 500);
    }
    */

    // H_T shape with leg angle
    void C_VectoredThrustControl::yawLocomMode_LR(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle)
    {
        if (_leg1_angle > 30)
        {
            _leg1_angle = 30;
        }
        else if (_leg1_angle < -30)
        {
            _leg1_angle = -30;
        }

        if (_leg2_angle > 30)
        {
            _leg2_angle = 30;
        }
        else if (_leg2_angle < -30)
        {
            _leg2_angle = -30;
        }

        if (_leg3_angle > 30)
        {
            _leg3_angle = 30;
        }
        else if (_leg3_angle < -30)
        {
            _leg3_angle = -30;
        }

        if (_leg4_angle > 30)
        {
            _leg4_angle = 30;
        }
        else if (_leg4_angle < -30)
        {
            _leg4_angle = -30;
        }

        Leg1.set_angle(-40, 0, -18 + _leg1_angle, 500);
        Leg2.set_angle(-40, 0, -18 + _leg2_angle, 500);
        Leg3.set_angle(-40, 0, -18 + _leg3_angle, 500);
        Leg4.set_angle(-40, 0, -18 + _leg4_angle, 500);
    }

    /*
    //hcy:gazebo原程序使用
    // X shape with leg angle///
    void C_VectoredThrustControl::yawLocomMode_X(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle)
    {
        if(_leg1_angle > 30)
        {
            _leg1_angle = 30;
        }
        else if(_leg1_angle < -30)
        {
            _leg1_angle = -30;
        }

        if(_leg2_angle > 30)
        {
            _leg2_angle = 30;
        }
        else if(_leg2_angle < -30)
        {
            _leg2_angle = -30;
        }

        if(_leg3_angle > 30)
        {
            _leg3_angle = 30;
        }
        else if(_leg3_angle < -30)
        {
            _leg3_angle = -30;
        }

        if(_leg4_angle > 30)
        {
            _leg4_angle = 30;
        }
        else if(_leg4_angle < -30)
        {
            _leg4_angle = -30;
        }

        // Leg1.set_angle(0, 0, -18 + _leg1_angle, 500);
        // Leg2.set_angle(0, 0, -18 + _leg2_angle, 500);
        // Leg3.set_angle(0, 0, -18 + _leg3_angle, 500);
        // Leg4.set_angle(0, 0, -18 + _leg4_angle, 500);
        Leg1.set_angle(0, 0, 4+_leg1_angle, 500);//hcy:4度为了平衡重力
        Leg2.set_angle(0, 0, 4+_leg2_angle, 500);
        Leg3.set_angle(0, 0, 4+_leg3_angle, 500);
        Leg4.set_angle(0, 0, 4+_leg4_angle, 500);
    }
    */

    void C_VectoredThrustControl::calcSprayForces_Shape_X(float _X_force, float _Y_force, float _Z_force,
                                                          float *_force1, float *_force2, float *_force3, float *_force4,
                                                          float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float __fx1, __fx2, __fx3, __fx4; // the horizontal force
        float __fz1, __fz2, __fz3, __fz4; // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi; // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate;

        // calculate the horizontal force   the direction of _X_force is forward and back

        if ((_Y_force + _X_force) >= 0)
        {
            __fx1 = 0;
            __fx3 = (_X_force + _Y_force) * sqrt(2) / 2;
        }
        else
        {
            __fx3 = 0;
            __fx1 = -(_X_force + _Y_force) * sqrt(2) / 2;
        }

        if ((_Y_force - _X_force) > 0)
        {
            __fx2 = 0;
            __fx4 = (_Y_force - _X_force) * sqrt(2) / 2;
        }
        else
        {
            __fx4 = 0;
            __fx2 = -(_Y_force - _X_force) * sqrt(2) / 2;
        }

        // calculate the vertical force
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;

        // compensate the horizontal force
        if (__fz1 <= 0)
        {
            __force_compensate = __fz1 / tan(-__leg_angle_limit);
        }
        else
        {
            __force_compensate = __fz1 / tan(__leg_angle_limit);
        }
        __fx1 += __force_compensate;
        __fx2 += __force_compensate;
        __fx3 += __force_compensate;
        __fx4 += __force_compensate;

        // output spray force
        *_force1 = sqrt(__fx1 * __fx1 + __fz1 * __fz1);
        *_force2 = sqrt(__fx2 * __fx2 + __fz2 * __fz2);
        *_force3 = sqrt(__fx3 * __fx3 + __fz3 * __fz3);
        *_force4 = sqrt(__fx4 * __fx4 + __fz4 * __fz4);

        // output leg angle
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001) * 57.3;
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001) * 57.3;
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001) * 57.3;
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001) * 57.3;
    }

    float C_VectoredThrustControl::myabs(float _a)
    {
        if (_a > 0)
            return _a;
        else
            return -_a;
    }
    // 力的线性空间重映射,将输入值 value 按比例缩放
    float C_VectoredThrustControl::mapForce(float _src_left, float _src_right, float _dst_left, float _dst_right, float _input_value)
    {
        // 如果输入的数据在映射空间之外,那就报错
        if (_input_value < _src_left || _input_value > _src_right)
        {
            cout << " map space is wrong " << endl;
            return -1;
        }

        if (myabs(_src_right - _src_left) < 0.001)
        {
            return (_dst_left + _dst_right) / 2; // 原始输入差不多的话,就输出映射的中值
        }

        float __k = (_dst_right - _dst_left) / (_src_right - _src_left);
        float __b = _dst_left - (_src_left * __k);

        float __res = __k * _input_value + __b; // 线性计算

        return __res;
    }

    /*
    //hcy:gazebo原程序使用
    // 另一个x运动方式解算版本///
    void C_VectoredThrustControl::calcSprayForces_Shape_X_2(   float _X_force, float _Y_force, float _Z_force,
                                      float *_force1, float *_force2, float *_force3, float *_force4,
                                      float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)

    {
        float __fx1, __fx2, __fx3, __fx4;  // the horizontal force
        float __fz1, __fz2, __fz3, __fz4;  // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi;          // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate1 = 0, __force_compensate2 = 0,
              __force_compensate3 = 0, __force_compensate4 = 0;

        float __min_force, __max_force;

        // base值 in 2.8 out 1.5 limit 3.1
        __fx1 = __fx2 = __fx3 = __fx4 = 2.8;

        // 对于x方向上
        // 2跟3在向x的正轴推进
        __fx2 += _X_force/4*sqrt(2) ; // X型模式下喷水电机的喷力sqrt(2)/2的喷力
        __fx3 += _X_force/4*sqrt(2) ;

        // 1和4向x的负轴推进
        __fx1 -= _X_force/4*sqrt(2) ;
        __fx4 -= _X_force/4*sqrt(2) ;


        // 对于y方向上
        // 3和4向y的正轴推进
        __fx3 += _Y_force/4*sqrt(2);
        __fx4 += _Y_force/4*sqrt(2);

        // 1和2向y的负轴推进
        __fx1 -= _Y_force/4*sqrt(2);
        __fx2 -= _Y_force/4*sqrt(2);


        // z方向上
        /// dubug
        //_Z_force = 0;//hcy
        ///
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force/4;


        // 判断腿部应该转多少角度 弧度制表示
        *_leg1_angle = atan2( __fz1 , __fx1 + 0.0001);
        *_leg2_angle = atan2( __fz2 , __fx2 + 0.0001);
        *_leg3_angle = atan2( __fz3 , __fx3 + 0.0001);
        *_leg4_angle = atan2( __fz4 , __fx4 + 0.0001);

        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力
        // 1号腿
        if( *_leg1_angle > __leg_angle_limit )
        {
            __force_compensate1 = __fz1/tan(__leg_angle_limit) - __fx1;
        }
        else if( *_leg1_angle < - __leg_angle_limit )
        {
            __force_compensate1 = __fz1/tan(-__leg_angle_limit) - __fx1;
        }

        // 2号腿
        if( *_leg2_angle > __leg_angle_limit )
        {
            __force_compensate2 = __fz2/tan(__leg_angle_limit) - __fx2;
        }
        else if( *_leg2_angle < - __leg_angle_limit )
        {
            __force_compensate2 = __fz2/tan(-__leg_angle_limit) - __fx2;
        }

        // 3号腿
        if( *_leg3_angle > __leg_angle_limit )
        {
            __force_compensate3 = __fz3/tan(__leg_angle_limit) - __fx3;
        }
        else if( *_leg3_angle < - __leg_angle_limit )
        {
            __force_compensate3 = __fz3/tan(-__leg_angle_limit) - __fx3;
        }

        // 4号腿
        if( *_leg4_angle > __leg_angle_limit )
        {
            __force_compensate4 = __fz4/tan(__leg_angle_limit) - __fx4;
        }
        else if( *_leg4_angle < - __leg_angle_limit )
        {
            __force_compensate4 = __fz4/tan(-__leg_angle_limit) - __fx4;
        }

        // 每个腿都加上总补偿
        __fx1 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );
        __fx2 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );
        __fx3 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );
        __fx4 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );


        // 最后再求一遍转角 转为角度制 这回肯定在-30到30之间了
        *_leg1_angle = atan2( __fz1 , __fx1 + 0.0001);
        *_leg2_angle = atan2( __fz2,  __fx2 + 0.0001);
        *_leg3_angle = atan2( __fz3 , __fx3 + 0.0001);
        *_leg4_angle = atan2( __fz4 , __fx4 + 0.0001);

        *_leg1_angle = *_leg1_angle * 57.3;
        *_leg2_angle = *_leg2_angle * 57.3;
        *_leg3_angle = *_leg3_angle * 57.3;
        *_leg4_angle = *_leg4_angle * 57.3;


        // 再求合力
        *_force1 = sqrt(__fx1*__fx1 + __fz1*__fz1);
        *_force2 = sqrt(__fx2*__fx2 + __fz2*__fz2);
        *_force3 = sqrt(__fx3*__fx3 + __fz3*__fz3);
        *_force4 = sqrt(__fx4*__fx4 + __fz4*__fz4);

        // 考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force4, min(*_force3, min(*_force2, *_force1)));
        __max_force = max(*_force4, max(*_force3, max(*_force2, *_force1)));

        /// dubug
        cout <<" X_force: " << _X_force << " Y_force: "<< _Y_force << endl;
        cout << " fx1: "<< __fx1 << " fx2: "<< __fx2
             << " fx3: "<< __fx3 << " fx4: "<< __fx4 << endl;

        cout << " fz1: " << __fz1 << " fz2: " << __fz2
             << " fz3: " << __fz3 << " fz4: " << __fz4 <<endl;


        cout << " min_force: " << __min_force << " max_force: " << __max_force << endl;


        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle <<endl;

        /// 如果力超出区间,才映射到区间内
        if(__min_force < 2.8 - 0.2 || __max_force > 2.8 + 0.5)
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, 2.8 - 0.2, 2.8 + 0.5, *_force1);
            *_force2 = mapForce(__min_force, __max_force, 2.8 - 0.2, 2.8 + 0.5, *_force2);
            *_force3 = mapForce(__min_force, __max_force, 2.8 - 0.2, 2.8 + 0.5, *_force3);
            *_force4 = mapForce(__min_force, __max_force, 2.8 - 0.2, 2.8 + 0.5, *_force4);
        }

    }
    */

    void C_VectoredThrustControl::calcSprayForces_Shape_H(float _X_force, float _Z_force, float _yaw_force,
                                                          float *_force1, float *_force2, float *_force3, float *_force4,
                                                          float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float __fx1, __fx2, __fx3, __fx4; // the horizontal force
        float __fz1, __fz2, __fz3, __fz4; // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi; // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate;

        // calculate the horizontal force   the direction of _X_force is forward and back

        if ((_X_force + _yaw_force) >= 0)
        {
            __fx4 = 0;
            __fx3 = (_X_force + _yaw_force) / 2;
        }
        else
        {
            __fx3 = 0;
            __fx4 = -(_X_force + _yaw_force) / 2;
        }

        if ((_X_force - _yaw_force) > 0)
        {
            __fx1 = 0;
            __fx2 = (_X_force - _yaw_force) / 2;
        }
        else
        {
            __fx2 = 0;
            __fx1 = -(_X_force - _yaw_force) / 2;
        }

        // calculate the vertical force
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;

        // compensate the horizontal force
        if (__fz1 <= 0)
        {
            __force_compensate = __fz1 / tan(-__leg_angle_limit);
        }
        else
        {
            __force_compensate = __fz1 / tan(__leg_angle_limit);
        }
        __fx1 += __force_compensate;
        __fx2 += __force_compensate;
        __fx3 += __force_compensate;
        __fx4 += __force_compensate;

        // output spray force
        *_force1 = sqrt(__fx1 * __fx1 + __fz1 * __fz1);
        *_force2 = sqrt(__fx2 * __fx2 + __fz2 * __fz2);
        *_force3 = sqrt(__fx3 * __fx3 + __fz3 * __fz3);
        *_force4 = sqrt(__fx4 * __fx4 + __fz4 * __fz4);

        // output leg angle
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001) * 57.3;
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001) * 57.3;
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001) * 57.3;
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001) * 57.3;
    }

    /*
    //hcy:gazebo原程序使用
    // 另一个解算策略///
    void C_VectoredThrustControl::calcSprayForces_Shape_H_2(float _X_force, float _Z_force, float _yaw_force,
                                                               float *_force1, float *_force2, float *_force3, float *_force4,
                                                               float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float __fx1, __fx2, __fx3, __fx4;  // the horizontal force
        float __fz1, __fz2, __fz3, __fz4;  // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi;          // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate1 = 0, __force_compensate2 = 0,
              __force_compensate3 = 0, __force_compensate4 = 0;

        float __min_force, __max_force;

        // base值 in 2.8 out 1.5 limit 3.1
        // __fx1 = __fx2 = __fx3 = __fx4 = 2.8;
        __fx1 = __fx2 = __fx3 = __fx4 = 2;//hcy

        // 对于x方向上
        // 2跟3在向x的正轴推进
        __fx2 += _X_force/4 ;
        __fx3 += _X_force/4 ;

        // 1和4向x的负轴推进
        __fx1 -= _X_force/4 ;
        __fx4 -= _X_force/4 ;


        // 对于航向角
        // 1和3朝正航向
        __fx1 += _yaw_force/4;
        __fx3 += _yaw_force/4;

        // 2和4朝负航向
        __fx2 -= _yaw_force/4;
        __fx4 -= _yaw_force/4;


        // z方向上
        /// dubug
        //_Z_force = 0;//hcy
        ///
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force/4;


        // 判断腿部应该转多少角度 弧度制表示
        *_leg1_angle = atan2( __fz1 , __fx1 + 0.0001);
        *_leg2_angle = atan2( __fz2 , __fx2 + 0.0001);
        *_leg3_angle = atan2( __fz3 , __fx3 + 0.0001);
        *_leg4_angle = atan2( __fz4 , __fx4 + 0.0001);

        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力
        // 1号腿
        if( *_leg1_angle > __leg_angle_limit )
        {
            __force_compensate1 = __fz1/tan(__leg_angle_limit) - __fx1;
        }
        else if( *_leg1_angle < - __leg_angle_limit )
        {
            __force_compensate1 = __fz1/tan(-__leg_angle_limit) - __fx1;
        }

        // 2号腿
        if( *_leg2_angle > __leg_angle_limit )
        {
            __force_compensate2 = __fz2/tan(__leg_angle_limit) - __fx2;
        }
        else if( *_leg2_angle < - __leg_angle_limit )
        {
            __force_compensate2 = __fz2/tan(-__leg_angle_limit) - __fx2;
        }

        // 3号腿
        if( *_leg3_angle > __leg_angle_limit )
        {
            __force_compensate3 = __fz3/tan(__leg_angle_limit) - __fx3;
        }
        else if( *_leg3_angle < - __leg_angle_limit )
        {
            __force_compensate3 = __fz3/tan(-__leg_angle_limit) - __fx3;
        }

        // 4号腿
        if( *_leg4_angle > __leg_angle_limit )
        {
            __force_compensate4 = __fz4/tan(__leg_angle_limit) - __fx4;
        }
        else if( *_leg4_angle < - __leg_angle_limit )
        {
            __force_compensate4 = __fz4/tan(-__leg_angle_limit) - __fx4;
        }

        // 每个腿都加上总补偿
        __fx1 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );
        __fx2 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );
        __fx3 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );
        __fx4 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4 );


        // 最后再求一遍转角 转为角度制 这回肯定在-30到30之间了
        *_leg1_angle = atan2( __fz1 , __fx1 + 0.0001);
        *_leg2_angle = atan2( __fz2,  __fx2 + 0.0001);
        *_leg3_angle = atan2( __fz3 , __fx3 + 0.0001);
        *_leg4_angle = atan2( __fz4 , __fx4 + 0.0001);

        *_leg1_angle = *_leg1_angle * 57.3;
        *_leg2_angle = *_leg2_angle * 57.3;
        *_leg3_angle = *_leg3_angle * 57.3;
        *_leg4_angle = *_leg4_angle * 57.3;


        // 再求合力
        *_force1 = sqrt(__fx1*__fx1 + __fz1*__fz1);
        *_force2 = sqrt(__fx2*__fx2 + __fz2*__fz2);
        *_force3 = sqrt(__fx3*__fx3 + __fz3*__fz3);
        *_force4 = sqrt(__fx4*__fx4 + __fz4*__fz4);

        printf("111:_force1:%f _force2:%f _force3:%f _force4:%f\n", *_force1, *_force2 , *_force3, *_force4);//hcy

        // 考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force4, min(*_force3, min(*_force2, *_force1)));
        __max_force = max(*_force4, max(*_force3, max(*_force2, *_force1)));

        /// dubug
        cout <<" X_force: " << _X_force << " yaw_force: "<< _yaw_force << endl;
        cout << " fx1: "<< __fx1 << " fx2: "<< __fx2
             << " fx3: "<< __fx3 << " fx4: "<< __fx4 << endl;

        cout << " fz1: " << __fz1 << " fz2: " << __fz2
             << " fz3: " << __fz3 << " fz4: " << __fz4 <<endl;


        cout << " min_force: " << __min_force << " max_force: " << __max_force << endl;


        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle <<endl;
        ///

        /// 如果力超出区间,才映射到区间内
        // if(__min_force < 2.8 - 0.7 || __max_force > 2.8 + 0.5)
        // {
        //     // 将对应的力映射到限定区间上
        //     *_force1 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force1);
        //     *_force2 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force2);
        //     *_force3 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force3);
        //     *_force4 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force4);
        // }
        // if(__min_force < 1.6 - 0.4 || __max_force > 1.6 + 0.3)
        // {
        //     // 将对应的力映射到限定区间上
        //     *_force1 = mapForce(__min_force, __max_force, 1.6 - 0.4, 1.6 + 0.3, *_force1);
        //     *_force2 = mapForce(__min_force, __max_force, 1.6 - 0.4, 1.6 + 0.3, *_force2);
        //     *_force3 = mapForce(__min_force, __max_force, 1.6 - 0.4, 1.6 + 0.3, *_force3);
        //     *_force4 = mapForce(__min_force, __max_force, 1.6 - 0.4, 1.6 + 0.3, *_force4);
        // }
        // if(__min_force < 2 - 0.5 || __max_force > 2 + 0.5)
        // {
        //     // 将对应的力映射到限定区间上
        //     *_force1 = mapForce(__min_force, __max_force, 2 - 0.5, 2 + 0.5, *_force1);
        //     *_force2 = mapForce(__min_force, __max_force, 2 - 0.5, 2 + 0.5, *_force2);
        //     *_force3 = mapForce(__min_force, __max_force, 2 - 0.5, 2 + 0.5, *_force3);
        //     *_force4 = mapForce(__min_force, __max_force, 2 - 0.5, 2 + 0.5, *_force4);
        // }
        if(__min_force < 2 - 0.7 || __max_force > 2 + 0.7)
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, 2 - 0.7, 2 + 0.7, *_force1);
            *_force2 = mapForce(__min_force, __max_force, 2 - 0.7, 2 + 0.7, *_force2);
            *_force3 = mapForce(__min_force, __max_force, 2 - 0.7, 2 + 0.7, *_force3);
            *_force4 = mapForce(__min_force, __max_force, 2 - 0.7, 2 + 0.7, *_force4);
        }
        printf("222:_force1:%f _force2:%f _force3:%f _force4:%f\n", *_force1, *_force2 , *_force3, *_force4);//hcy

    }
    */

    void C_VectoredThrustControl::calcSprayForces_Shape_H_T(float _Y_force, float _Z_force, float _yaw_force,
                                                            float *_force1, float *_force2, float *_force3, float *_force4,
                                                            float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float __fx1, __fx2, __fx3, __fx4; // the horizontal force
        float __fz1, __fz2, __fz3, __fz4; // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi; // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate;

        // calculate the horizontal force   the direction of _X_force is forward and back

        if ((_Y_force + _yaw_force) >= 0)
        {
            __fx1 = 0;
            __fx4 = (_Y_force + _yaw_force) / 2;
        }
        else
        {
            __fx4 = 0;
            __fx1 = -(_Y_force + _yaw_force) / 2;
        }

        if ((_Y_force - _yaw_force) >= 0)
        {
            __fx2 = 0;
            __fx3 = (_Y_force - _yaw_force) / 2;
        }
        else
        {
            __fx3 = 0;
            __fx2 = -(_Y_force - _yaw_force) / 2;
        }

        // calculate the vertical force
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;

        // compensate the horizontal force
        if (__fz1 <= 0)
        {
            __force_compensate = __fz1 / tan(-__leg_angle_limit);
        }
        else
        {
            __force_compensate = __fz1 / tan(__leg_angle_limit);
        }
        __fx1 += __force_compensate;
        __fx2 += __force_compensate;
        __fx3 += __force_compensate;
        __fx4 += __force_compensate;

        // output spray force
        *_force1 = sqrt(__fx1 * __fx1 + __fz1 * __fz1);
        *_force2 = sqrt(__fx2 * __fx2 + __fz2 * __fz2);
        *_force3 = sqrt(__fx3 * __fx3 + __fz3 * __fz3);
        *_force4 = sqrt(__fx4 * __fx4 + __fz4 * __fz4);

        // output leg angle
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001) * 57.3;
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001) * 57.3;
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001) * 57.3;
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001) * 57.3;
    }

    void C_VectoredThrustControl::calcSprayForces_Shape_H_T_2(float _Y_force, float _Z_force, float _yaw_force,
                                                              float *_force1, float *_force2, float *_force3, float *_force4,
                                                              float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float __fx1, __fx2, __fx3, __fx4; // the horizontal force
        float __fz1, __fz2, __fz3, __fz4; // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi; // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate1 = 0, __force_compensate2 = 0,
              __force_compensate3 = 0, __force_compensate4 = 0;

        float __min_force, __max_force;

        // base值 in 2.8 out 1.5 limit 3.1
        __fx1 = __fx2 = __fx3 = __fx4 = 2.8;

        // 对于y方向上
        // 3跟4在向y的正轴推进
        __fx3 += _Y_force / 4; // X型模式下喷水电机的喷力sqrt(2)/2的喷力
        __fx4 += _Y_force / 4;

        // 1和2向x的负轴推进
        __fx1 -= _Y_force / 4;
        __fx2 -= _Y_force / 4;

        // 对于航向角
        // 2和4朝正航向
        __fx2 += _yaw_force / 4;
        __fx4 += _yaw_force / 4;

        // 1和3朝负航向
        __fx1 -= _yaw_force / 4;
        __fx3 -= _yaw_force / 4;

        // z方向上
        /// dubug
        _Z_force = 0;
        ///
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;

        // 判断腿部应该转多少角度 弧度制表示
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001);
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001);

        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力
        // 1号腿
        if (*_leg1_angle > __leg_angle_limit)
        {
            __force_compensate1 = __fz1 / tan(__leg_angle_limit) - __fx1;
        }
        else if (*_leg1_angle < -__leg_angle_limit)
        {
            __force_compensate1 = __fz1 / tan(-__leg_angle_limit) - __fx1;
        }

        // 2号腿
        if (*_leg2_angle > __leg_angle_limit)
        {
            __force_compensate2 = __fz2 / tan(__leg_angle_limit) - __fx2;
        }
        else if (*_leg2_angle < -__leg_angle_limit)
        {
            __force_compensate2 = __fz2 / tan(-__leg_angle_limit) - __fx2;
        }

        // 3号腿
        if (*_leg3_angle > __leg_angle_limit)
        {
            __force_compensate3 = __fz3 / tan(__leg_angle_limit) - __fx3;
        }
        else if (*_leg3_angle < -__leg_angle_limit)
        {
            __force_compensate3 = __fz3 / tan(-__leg_angle_limit) - __fx3;
        }

        // 4号腿
        if (*_leg4_angle > __leg_angle_limit)
        {
            __force_compensate4 = __fz4 / tan(__leg_angle_limit) - __fx4;
        }
        else if (*_leg4_angle < -__leg_angle_limit)
        {
            __force_compensate4 = __fz4 / tan(-__leg_angle_limit) - __fx4;
        }

        // 每个腿都加上总补偿
        __fx1 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx2 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx3 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx4 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);

        // 最后再求一遍转角 转为角度制 这回肯定在-30到30之间了
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001);
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001);

        *_leg1_angle = *_leg1_angle * 57.3;
        *_leg2_angle = *_leg2_angle * 57.3;
        *_leg3_angle = *_leg3_angle * 57.3;
        *_leg4_angle = *_leg4_angle * 57.3;

        // 再求合力
        *_force1 = sqrt(__fx1 * __fx1 + __fz1 * __fz1);
        *_force2 = sqrt(__fx2 * __fx2 + __fz2 * __fz2);
        *_force3 = sqrt(__fx3 * __fx3 + __fz3 * __fz3);
        *_force4 = sqrt(__fx4 * __fx4 + __fz4 * __fz4);

        // 考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force4, min(*_force3, min(*_force2, *_force1)));
        __max_force = max(*_force4, max(*_force3, max(*_force2, *_force1)));

        /// dubug
        cout << " Y_force: " << _Y_force << " yaw_force: " << _yaw_force << endl;
        cout << " fx1: " << __fx1 << " fx2: " << __fx2
             << " fx3: " << __fx3 << " fx4: " << __fx4 << endl;

        cout << " fz1: " << __fz1 << " fz2: " << __fz2
             << " fz3: " << __fz3 << " fz4: " << __fz4 << endl;

        cout << " min_force: " << __min_force << " max_force: " << __max_force << endl;

        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle << endl;

        /// 如果力超出区间,才映射到区间内 否则不用映射
        if (__min_force < 2.8 - 0.7 || __max_force > 2.8 + 0.5)
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force1);
            *_force2 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force2);
            *_force3 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force3);
            *_force4 = mapForce(__min_force, __max_force, 2.8 - 0.7, 2.8 + 0.5, *_force4);
        }
    }

    void C_VectoredThrustControl::PxControlWithPxForce(float _Px_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(_Px_force, 0, 0,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::VxControlWithVxForce(float _Vx_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(_Vx_force, 0, 0,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::PyControlWithPyForce(float _Py_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(0, _Py_force, 0,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::VyControlWithVyForce(float _Vy_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(0, _Vy_force, 0,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::yawControlWithYawForce(float _yaw_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_H(0, 0, _yaw_force,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::depthControlWithPzForce(float _Pz_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(0, 0, _Pz_force,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::doubleClosedLoopControlPx(float _Vx_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(_Vx_force, 0, 0,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    void C_VectoredThrustControl::doubleClosedLoopControlPy(float _Vy_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        calcSprayForces_Shape_X(0, _Vy_force, 0,
                                &__force1, &__force2, &__force3, &__force4,
                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
        yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
        thrusterforce2speed(__force1, __force2, __force3, __force4);
    }

    /*
    //hcy:gazebo原程序使用
    // 三维控制 输入腿型  x, y, z, yaw 四个力,自动解算///
    void C_VectoredThrustControl::space3DControl(int _leg_shape, float _X_force, float _Y_force, float _Z_force, float _yaw_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        switch(_leg_shape)
        {
        case SHAPE_H:
        {
    //        calcSprayForces_Shape_H(_X_force, _Z_force, _yaw_force,
    //                                &__force1, &__force2, &__force3, &__force4,
    //                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
            calcSprayForces_Shape_H_2(_X_force, _Z_force, _yaw_force,
                                    &__force1, &__force2, &__force3, &__force4,
                                    &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
            get_thrust_force(__force1,__force2,__force3,__force4);

            yawLocomMode_FB(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed(__force1, __force2, __force3, __force4);
            // thrusterforce2speed_H(__force1, __force2, __force3, __force4);//hcy
        };break;

        case SHAPE_H_T:
        {
    //        calcSprayForces_Shape_H_T(_Y_force, _Z_force, _yaw_force,
    //                                &__force1, &__force2, &__force3, &__force4,
    //                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
            calcSprayForces_Shape_H_T_2(_Y_force, _Z_force, _yaw_force,
                                      &__force1, &__force2, &__force3, &__force4,
                                      &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);

            yawLocomMode_LR(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed(__force1, __force2, __force3, __force4);
        };break;

        case SHAPE_X:
        {
    //        calcSprayForces_Shape_X(_X_force, _Y_force, _Z_force,
    //                                &__force1, &__force2, &__force3, &__force4,
    //                                &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
            calcSprayForces_Shape_X_2(_X_force, _Y_force, _Z_force,
                                    &__force1, &__force2, &__force3, &__force4,
                                    &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);

            yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed(__force1, __force2, __force3, __force4);
        };break;
        default:
        {
            cout << "please choose a correct leg mode!!!"<<endl;
        };break;
        }

        cout << " force1: " << __force1  << " force2: " << __force2
             << " force3: " << __force3  << " force4: " << __force4 <<endl;

        cout << " angle1: " << __leg_angle1  << " angle2: " << __leg_angle2
             << " angle3: " << __leg_angle3  << " angle4: " << __leg_angle4 <<endl;
    }
    */

    void C_VectoredThrustControl::get_thrust_force(float force1, float force2, float force3, float force4)
    {

        thrust1 = force1;
        thrust2 = force2;
        thrust3 = force3;
        thrust4 = force4;
    }

    /*****************************************hcy*****************************************/

    // hcy:H型新版本解算策略,（重要）H型新版本，计算力和角度
    void C_VectoredThrustControl::calcSprayForces_Shape_H_2(float _X_force, float _Z_force, float _yaw_force,
                                                            float *_force1, float *_force2, float *_force3, float *_force4,
                                                            float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float __fx1, __fx2, __fx3, __fx4; // the horizontal force
        float __fz1, __fz2, __fz3, __fz4; // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi; // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate1 = 0, __force_compensate2 = 0,
              __force_compensate3 = 0, __force_compensate4 = 0;

        float __min_force, __max_force;

        //
        __fx1 = __fx2 = __fx3 = __fx4 = base_motor_force;

        // 对于x方向上
        // 2跟3在向x的正轴推进
        __fx2 += _X_force / 4;
        __fx3 += _X_force / 4;

        // 1和4向x的负轴推进
        __fx1 -= _X_force / 4;
        __fx4 -= _X_force / 4;

        // 对于航向角
        // 1和3朝正航向
        __fx1 += _yaw_force / 4;
        __fx3 += _yaw_force / 4;

        // 2和4朝负航向
        __fx2 -= _yaw_force / 4;
        __fx4 -= _yaw_force / 4;

        // z方向上
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;

        // 判断腿部应该转多少角度 弧度制表示
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001);
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001);

        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力
        // 1号腿
        if (*_leg1_angle > __leg_angle_limit)
        {
            __force_compensate1 = __fz1 / tan(__leg_angle_limit) - __fx1;
        }
        else if (*_leg1_angle < -__leg_angle_limit)
        {
            __force_compensate1 = __fz1 / tan(-__leg_angle_limit) - __fx1;
        }

        // 2号腿
        if (*_leg2_angle > __leg_angle_limit)
        {
            __force_compensate2 = __fz2 / tan(__leg_angle_limit) - __fx2;
        }
        else if (*_leg2_angle < -__leg_angle_limit)
        {
            __force_compensate2 = __fz2 / tan(-__leg_angle_limit) - __fx2;
        }

        // 3号腿
        if (*_leg3_angle > __leg_angle_limit)
        {
            __force_compensate3 = __fz3 / tan(__leg_angle_limit) - __fx3;
        }
        else if (*_leg3_angle < -__leg_angle_limit)
        {
            __force_compensate3 = __fz3 / tan(-__leg_angle_limit) - __fx3;
        }

        // 4号腿
        if (*_leg4_angle > __leg_angle_limit)
        {
            __force_compensate4 = __fz4 / tan(__leg_angle_limit) - __fx4;
        }
        else if (*_leg4_angle < -__leg_angle_limit)
        {
            __force_compensate4 = __fz4 / tan(-__leg_angle_limit) - __fx4;
        }

        // 每个腿都加上总补偿
        __fx1 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx2 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx3 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx4 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);

        // 最后再求一遍转角 转为角度制 这回肯定在-30到30之间了
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001);
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001);

        *_leg1_angle = *_leg1_angle * 57.3;
        *_leg2_angle = *_leg2_angle * 57.3;
        *_leg3_angle = *_leg3_angle * 57.3;
        *_leg4_angle = *_leg4_angle * 57.3;

        // 再求合力
        *_force1 = sqrt(__fx1 * __fx1 + __fz1 * __fz1);
        *_force2 = sqrt(__fx2 * __fx2 + __fz2 * __fz2);
        *_force3 = sqrt(__fx3 * __fx3 + __fz3 * __fz3);
        *_force4 = sqrt(__fx4 * __fx4 + __fz4 * __fz4);

        printf("111:_force1:%f _force2:%f _force3:%f _force4:%f\n", *_force1, *_force2, *_force3, *_force4); // hcy

        // 考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force4, min(*_force3, min(*_force2, *_force1)));
        __max_force = max(*_force4, max(*_force3, max(*_force2, *_force1)));

        /// dubug
        cout << " X_force: " << _X_force << " yaw_force: " << _yaw_force << endl;
        cout << " fx1: " << __fx1 << " fx2: " << __fx2
             << " fx3: " << __fx3 << " fx4: " << __fx4 << endl;

        cout << " fz1: " << __fz1 << " fz2: " << __fz2
             << " fz3: " << __fz3 << " fz4: " << __fz4 << endl;

        cout << " min_force: " << __min_force << " max_force: " << __max_force << endl;

        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle << endl;
        ///

        /// 如果力超出区间,才映射到区间内
        if (__min_force < (base_motor_force - bias_motor_force) || __max_force > (base_motor_force + bias_motor_force))
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force1);
            *_force2 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force2);
            *_force3 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force3);
            *_force4 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force4);
        }

        printf("222:_force1:%f _force2:%f _force3:%f _force4:%f\n", *_force1, *_force2, *_force3, *_force4); // hcy
    }

    // hcy:H shape with leg angle,（重要）设置H型时动态角度
    void C_VectoredThrustControl::yawLocomMode_FB(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle,
                                                bool manual_yaw)
    {
        if (_leg1_angle > 30)
        {
            _leg1_angle = 30;
        }
        else if (_leg1_angle < -30)
        {
            _leg1_angle = -30;
        }

        if (_leg2_angle > 30)
        {
            _leg2_angle = 30;
        }
        else if (_leg2_angle < -30)
        {
            _leg2_angle = -30;
        }

        if (_leg3_angle > 30)
        {
            _leg3_angle = 30;
        }
        else if (_leg3_angle < -30)
        {
            _leg3_angle = -30;
        }

        if (_leg4_angle > 30)
        {
            _leg4_angle = 30;
        }
        else if (_leg4_angle < -30)
        {
            _leg4_angle = -30;
        }

        // Leg1.set_angle(40, 0, _leg1_angle + 10, 500);
        // Leg2.set_angle(40, 0, _leg2_angle + 10, 500);
        // Leg3.set_angle(40, 0, _leg3_angle + 10, 500);
        // Leg4.set_angle(40, 0, _leg4_angle + 10, 500);
        Leg1.set_angle(40, 0, _leg1_angle, 2000);
        Leg2.set_angle(manual_yaw ? -40 : 40, 0, _leg2_angle, 2000);
        Leg3.set_angle(40, 0, _leg3_angle, 2000);
        Leg4.set_angle(manual_yaw ? -40 : 40, 0, _leg4_angle, 2000);
    }

    // 1/3 号提供正偏航，2/4 号提供负偏航；不改变自动控制的原 H 型姿态。
    void C_VectoredThrustControl::manualYawDepthControl(float depth_force, float yaw_force)
    {
        float forces[4], angles[4];
        calcSprayForces_Shape_H_2(0, depth_force, yaw_force,
            &forces[0], &forces[1], &forces[2], &forces[3],
            &angles[0], &angles[1], &angles[2], &angles[3]);
        yawLocomMode_FB(angles[0], angles[1], angles[2], angles[3], true);
        thrusterforce2speed_H(forces[0], forces[1], forces[2], forces[3]);
        thrust_force_1 = forces[0]; thrust_force_2 = forces[1];
        thrust_force_3 = forces[2]; thrust_force_4 = forces[3];
        ROS_INFO_STREAM("[MANUAL YAW ALLOCATION] H_deg=40,-40,40,-40 yaw_request=" << yaw_force
            << " depth_request=" << depth_force << " force=" << forces[0] << "," << forces[1]
            << "," << forces[2] << "," << forces[3]);
    }

    // hcy:对电机进行针对性的力解算 H型
    void C_VectoredThrustControl::thrusterforce2speed_H(float _force1, float _force2, float _force3, float _force4)
    {
        // 推力在1到3之间，转速在100到500之间
        int __speed1, __speed2, __speed3, __speed4;
        float __k_1, __b_1;
        float __k_2, __b_2;
        float __k_3, __b_3;
        float __k_4, __b_4;

        // 推力限幅
        if (_force1 < (base_motor_force - bias_motor_force))
            _force1 = (base_motor_force - bias_motor_force);
        else if (_force1 > (base_motor_force + bias_motor_force))
            _force1 = (base_motor_force + bias_motor_force);

        if (_force2 < (base_motor_force - bias_motor_force))
            _force2 = (base_motor_force - bias_motor_force);
        else if (_force2 > (base_motor_force + bias_motor_force))
            _force2 = (base_motor_force + bias_motor_force);

        if (_force3 < (base_motor_force - bias_motor_force))
            _force3 = (base_motor_force - bias_motor_force);
        else if (_force3 > (base_motor_force + bias_motor_force))
            _force3 = (base_motor_force + bias_motor_force);

        if (_force4 < (base_motor_force - bias_motor_force))
            _force4 = (base_motor_force - bias_motor_force);
        else if (_force4 > (base_motor_force + bias_motor_force))
            _force4 = (base_motor_force + bias_motor_force);

        printf("333:_force1:%f _force2:%f _force3:%f _force4:%f\n", _force1, _force2, _force3, _force4);

        __k_1 = (h_shape_max_motor_speed1 - h_shape_min_motor_speed1) / (2 * bias_motor_force);
        __b_1 = h_shape_min_motor_speed1 - __k_1 * (base_motor_force - bias_motor_force);

        __k_2 = (h_shape_max_motor_speed2 - h_shape_min_motor_speed2) / (2 * bias_motor_force);
        __b_2 = h_shape_min_motor_speed2 - __k_2 * (base_motor_force - bias_motor_force);

        __k_3 = (h_shape_max_motor_speed3 - h_shape_min_motor_speed3) / (2 * bias_motor_force);
        __b_3 = h_shape_min_motor_speed3 - __k_3 * (base_motor_force - bias_motor_force);

        __k_4 = (h_shape_max_motor_speed4 - h_shape_min_motor_speed4) / (2 * bias_motor_force);
        __b_4 = h_shape_min_motor_speed4 - __k_4 * (base_motor_force - bias_motor_force);

        __speed1 = __k_1 * _force1 + __b_1;
        __speed2 = __k_2 * _force2 + __b_2;
        __speed3 = __k_3 * _force3 + __b_3;
        __speed4 = __k_4 * _force4 + __b_4;

        printf("!!!speed1:%d speed2:%d speed3:%d speed4:%d\n", __speed1, __speed2, __speed3, __speed4);

        // 将转速锁定在对应电机的区间内部

        if (__speed1 < h_shape_min_motor_speed1)
            __speed1 = h_shape_min_motor_speed1;
        else if (__speed1 > h_shape_max_motor_speed1)
            __speed1 = h_shape_max_motor_speed1;

        if (__speed2 < h_shape_min_motor_speed2)
            __speed2 = h_shape_min_motor_speed2;
        else if (__speed2 > h_shape_max_motor_speed2)
            __speed2 = h_shape_max_motor_speed2;

        if (__speed3 < h_shape_min_motor_speed3)
            __speed3 = h_shape_min_motor_speed3;
        else if (__speed3 > h_shape_max_motor_speed3)
            __speed3 = h_shape_max_motor_speed3;

        if (__speed4 < h_shape_min_motor_speed4)
            __speed4 = h_shape_min_motor_speed4;
        else if (__speed4 > h_shape_max_motor_speed4)
            __speed4 = h_shape_max_motor_speed4;

        pwm_msg.rotor_speed1 = __speed1; // hcy
        pwm_msg.rotor_speed2 = __speed2;
        pwm_msg.rotor_speed3 = __speed3;
        pwm_msg.rotor_speed4 = __speed4;

        extern_pub_pwm->publish(pwm_msg); // hcy

        // motorspeed(__speed1, __speed2, __speed3, __speed4);
        printf("speed1:%d speed2:%d speed3:%d speed4:%d\n", __speed1, __speed2, __speed3, __speed4);
    }

    // hcy:另一个x运动方式解算版本 新版本,（重要）X型新版本，控制深度会用到
    void C_VectoredThrustControl::calcSprayForces_Shape_X_2(float _X_force, float _Y_force, float _Z_force,
                                                            float *_force1, float *_force2, float *_force3, float *_force4,
                                                            float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)

    {
        float __fx1, __fx2, __fx3, __fx4; // the horizontal force//斜45度的在x方向的力
        float __fz1, __fz2, __fz3, __fz4; // the vertical force

        float __leg_angle_limit = 30.0 / 180.0 * pi;            // the limit angle of leg ( -30 ~ 30 )
        float __force_compensate1 = 0, __force_compensate2 = 0, // hy:力补偿
            __force_compensate3 = 0, __force_compensate4 = 0;

        float __min_force, __max_force;

        // base value
        __fx1 = __fx2 = __fx3 = __fx4 = base_motor_force; // hy:基础力为2N

        // 对于x方向上
        // 2跟3在向x的正轴推进
        __fx2 += _X_force / 4 * sqrt(2); // X型模式下喷水电机的喷力sqrt(2)/2的喷力
        __fx3 += _X_force / 4 * sqrt(2);

        // 1和4向x的负轴推进
        __fx1 -= _X_force / 4 * sqrt(2);
        __fx4 -= _X_force / 4 * sqrt(2);

        // 对于y方向上
        // 3和4向y的正轴推进
        __fx3 += _Y_force / 4 * sqrt(2);
        __fx4 += _Y_force / 4 * sqrt(2);

        // 1和2向y的负轴推进
        __fx1 -= _Y_force / 4 * sqrt(2);
        __fx2 -= _Y_force / 4 * sqrt(2);

        // hy:对于每个腿而言的推进器z方向的力
        __fz1 = __fz2 = __fz3 = __fz4 = _Z_force / 4;

        // 判断腿部应该转多少角度 弧度制表示
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001); // 避免x方向为0
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001);

        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力
        // 1号腿
        if (*_leg1_angle > __leg_angle_limit)
        {
            __force_compensate1 = __fz1 / tan(__leg_angle_limit) - __fx1;
        }
        else if (*_leg1_angle < -__leg_angle_limit)
        {
            __force_compensate1 = __fz1 / tan(-__leg_angle_limit) - __fx1;
        }

        // 2号腿
        if (*_leg2_angle > __leg_angle_limit)
        {
            __force_compensate2 = __fz2 / tan(__leg_angle_limit) - __fx2;
        }
        else if (*_leg2_angle < -__leg_angle_limit)
        {
            __force_compensate2 = __fz2 / tan(-__leg_angle_limit) - __fx2;
        }

        // 3号腿
        if (*_leg3_angle > __leg_angle_limit)
        {
            __force_compensate3 = __fz3 / tan(__leg_angle_limit) - __fx3;
        }
        else if (*_leg3_angle < -__leg_angle_limit)
        {
            __force_compensate3 = __fz3 / tan(-__leg_angle_limit) - __fx3;
        }

        // 4号腿
        if (*_leg4_angle > __leg_angle_limit)
        {
            __force_compensate4 = __fz4 / tan(__leg_angle_limit) - __fx4;
        }
        else if (*_leg4_angle < -__leg_angle_limit)
        {
            __force_compensate4 = __fz4 / tan(-__leg_angle_limit) - __fx4;
        }

        // 每个腿都加上总补偿,每个腿都加上是为了维持力的平衡
        __fx1 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx2 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx3 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);
        __fx4 += (__force_compensate1 + __force_compensate2 + __force_compensate3 + __force_compensate4);

        // 最后再求一遍转角 转为角度制 这回肯定在-30到30之间了
        *_leg1_angle = atan2(__fz1, __fx1 + 0.0001);
        *_leg2_angle = atan2(__fz2, __fx2 + 0.0001);
        *_leg3_angle = atan2(__fz3, __fx3 + 0.0001);
        *_leg4_angle = atan2(__fz4, __fx4 + 0.0001);

        // hy:转换为角度制
        *_leg1_angle = *_leg1_angle * 57.3;
        *_leg2_angle = *_leg2_angle * 57.3;
        *_leg3_angle = *_leg3_angle * 57.3;
        *_leg4_angle = *_leg4_angle * 57.3;

        // hy:对于每个腿而言的x,z方向的合力
        //  再求合力
        *_force1 = sqrt(__fx1 * __fx1 + __fz1 * __fz1);
        *_force2 = sqrt(__fx2 * __fx2 + __fz2 * __fz2);
        *_force3 = sqrt(__fx3 * __fx3 + __fz3 * __fz3);
        *_force4 = sqrt(__fx4 * __fx4 + __fz4 * __fz4);

        // hy:对于机器人而言的四个推进器中的最小力和最大力
        //  考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force4, min(*_force3, min(*_force2, *_force1)));
        __max_force = max(*_force4, max(*_force3, max(*_force2, *_force1)));

        /// dubug
        // hy:由adrcoutput算出的controller_x_position和controller_y_position
        cout << "**************************X型力的解算**************************" << endl;
        cout << " X_force: " << _X_force << " Y_force: " << _Y_force << endl;

        cout << " fx1: " << __fx1 << " fx2: " << __fx2
             << " fx3: " << __fx3 << " fx4: " << __fx4 << endl;
        cout << " fz1: " << __fz1 << " fz2: " << __fz2
             << " fz3: " << __fz3 << " fz4: " << __fz4 << endl;

        cout << " min_force: " << __min_force << " max_force: " << __max_force << endl;

        // hy:角度变化量
        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle << endl;

        /// 如果力超出区间,才映射到区间内
        if (__min_force < (base_motor_force - bias_motor_force) || __max_force > (base_motor_force + bias_motor_force))
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force1);
            *_force2 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force2);
            *_force3 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force3);
            *_force4 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force4);
        }
    }

    // hcy:X shape with leg angle
    void C_VectoredThrustControl::yawLocomMode_X(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle)
    {
        if (_leg1_angle > 30)
        {
            _leg1_angle = 30;
        }
        else if (_leg1_angle < -30)
        {
            _leg1_angle = -30;
        }

        if (_leg2_angle > 30)
        {
            _leg2_angle = 30;
        }
        else if (_leg2_angle < -30)
        {
            _leg2_angle = -30;
        }

        if (_leg3_angle > 30)
        {
            _leg3_angle = 30;
        }
        else if (_leg3_angle < -30)
        {
            _leg3_angle = -30;
        }

        if (_leg4_angle > 30)
        {
            _leg4_angle = 30;
        }
        else if (_leg4_angle < -30)
        {
            _leg4_angle = -30;
        }

        // Leg1.set_angle(0, 0, _leg1_angle + 10, 500);
        // Leg2.set_angle(0, 0, _leg2_angle + 10, 500);
        // Leg3.set_angle(0, 0, _leg3_angle + 10, 500);
        // Leg4.set_angle(0, 0, _leg4_angle + 10, 500);
        Leg1.set_angle(0, 0, _leg1_angle, 500);
        Leg2.set_angle(0, 0, _leg2_angle, 500);
        Leg3.set_angle(0, 0, _leg3_angle, 500);
        Leg4.set_angle(0, 0, _leg4_angle, 500);
    }

    void C_VectoredThrustControl::yawLocomMode_G(float _leg1_angle, float _leg2_angle, float _leg3_angle, float _leg4_angle)
    {
        Leg1.set_angle(0, 110, _leg1_angle, 500);
        Leg2.set_angle(40, 0, 0, 500);
        Leg3.set_angle(0, 110, _leg3_angle, 500);
        Leg4.set_angle(40, 0, 0, 500);
    }


    // hcy:对电机进行针对性的力解算 X型
    void C_VectoredThrustControl::thrusterforce2speed_X(float _force1, float _force2, float _force3, float _force4)
    {

        // 推力在1到3之间，转速在100到500之间
        int __speed1, __speed2, __speed3, __speed4;
        float __k_1, __b_1;
        float __k_2, __b_2;
        float __k_3, __b_3;
        float __k_4, __b_4;

        // 推力限幅
        if (_force1 < (base_motor_force - bias_motor_force))
            _force1 = (base_motor_force - bias_motor_force);
        else if (_force1 > (base_motor_force + bias_motor_force))
            _force1 = (base_motor_force + bias_motor_force);

        if (_force2 < (base_motor_force - bias_motor_force))
            _force2 = (base_motor_force - bias_motor_force);
        else if (_force2 > (base_motor_force + bias_motor_force))
            _force2 = (base_motor_force + bias_motor_force);

        if (_force3 < (base_motor_force - bias_motor_force))
            _force3 = (base_motor_force - bias_motor_force);
        else if (_force3 > (base_motor_force + bias_motor_force))
            _force3 = (base_motor_force + bias_motor_force);

        if (_force4 < (base_motor_force - bias_motor_force))
            _force4 = (base_motor_force - bias_motor_force);
        else if (_force4 > (base_motor_force + bias_motor_force))
            _force4 = (base_motor_force + bias_motor_force);

        __k_1 = (x_shape_max_motor_speed1 - x_shape_min_motor_speed1) / (2 * bias_motor_force);
        __b_1 = x_shape_min_motor_speed1 - __k_1 * (base_motor_force - bias_motor_force);

        __k_2 = (x_shape_max_motor_speed2 - x_shape_min_motor_speed1) / (2 * bias_motor_force);
        __b_2 = x_shape_min_motor_speed2 - __k_2 * (base_motor_force - bias_motor_force);

        __k_3 = (x_shape_max_motor_speed3 - x_shape_min_motor_speed3) / (2 * bias_motor_force);
        __b_3 = x_shape_min_motor_speed3 - __k_3 * (base_motor_force - bias_motor_force);

        __k_4 = (x_shape_max_motor_speed4 - x_shape_min_motor_speed4) / (2 * bias_motor_force);
        __b_4 = x_shape_min_motor_speed4 - __k_4 * (base_motor_force - bias_motor_force);

        __speed1 = __k_1 * _force1 + __b_1;
        __speed2 = __k_2 * _force2 + __b_2;
        __speed3 = __k_3 * _force3 + __b_3;
        __speed4 = __k_4 * _force4 + __b_4;

        // 将转速锁定在对应电机的区间内部

        if (__speed1 < x_shape_min_motor_speed1)
            __speed1 = x_shape_min_motor_speed1;
        else if (__speed1 > x_shape_max_motor_speed1)
            __speed1 = x_shape_max_motor_speed1;

        if (__speed2 < x_shape_min_motor_speed2)
            __speed2 = x_shape_min_motor_speed2;
        else if (__speed2 > x_shape_max_motor_speed2)
            __speed2 = x_shape_max_motor_speed2;

        if (__speed3 < x_shape_min_motor_speed3)
            __speed3 = x_shape_min_motor_speed3;
        else if (__speed3 > x_shape_max_motor_speed3)
            __speed3 = x_shape_max_motor_speed3;

        if (__speed4 < x_shape_min_motor_speed4)
            __speed4 = x_shape_min_motor_speed4;
        else if (__speed4 > x_shape_max_motor_speed4)
            __speed4 = x_shape_max_motor_speed4;

        pwm_msg.rotor_speed1 = __speed1; // hcy
        pwm_msg.rotor_speed2 = __speed2;
        pwm_msg.rotor_speed3 = __speed3;
        pwm_msg.rotor_speed4 = __speed4;

        extern_pub_pwm->publish(pwm_msg); // hcy
        cout << "**************************推进器力转速度**************************" << endl;
        // motorspeed(__speed1, __speed2, __speed3, __speed4);
        printf("speed1:%d speed2:%d speed3:%d speed4:%d\n", __speed1, __speed2, __speed3, __speed4);
    }

    void C_VectoredThrustControl::calcSprayForces_Shape_G(float _controller_forward, float _controller_vertical, float _controller_torque,
                                                          float *_force1, float *_force2, float *_force3, float *_force4,
                                                          float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle)
    {
        float forceBack, forceForward, forceVertical, forceToque;

        float __force_compensate1 = 0, __force_compensate2 = 0, // hy:力补偿
              __force_compensate3 = 0, __force_compensate4 = 0;
        float __min_force, __max_force;
        float deta_angle1, deta_angle3;

        forceBack = base_motor_force - _controller_forward/2;
        forceForward = base_motor_force + _controller_forward/2;
        forceVertical = _controller_vertical/2;
        forceToque = _controller_torque/0.3;
        if(forceToque > 4) forceToque = 4;
        if(forceToque < -4) forceToque = -4;
        
        deta_angle1 = atan2(forceVertical, forceBack+0.0001);
        deta_angle3 = atan2(forceVertical, forceForward+0.0001);
       
        if(deta_angle1 > 30) deta_angle1 = 30;
        if(deta_angle1 < -30) deta_angle1 = -30;
        if(deta_angle3 > 30) deta_angle3 = 30;
        if(deta_angle3 < -30) deta_angle3 = -30;


        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力

        // hy:转换为角度制
        *_leg1_angle = -110 + deta_angle1 * 57.3;
        *_leg3_angle = -110 + deta_angle3 * 57.3;

        // hy:对于每个腿而言的x,z方向的合力
        //  再求合力
        *_force1 = sqrt(forceBack * forceBack + forceVertical * forceVertical);
        *_force2 = forceToque;
        *_force3 = sqrt(forceForward * forceForward + forceVertical * forceVertical);
        *_force4 = forceToque;

        // hy:对于机器人而言的四个推进器中的最小力和最大力
        //  考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force3, *_force1);
        __max_force = max(*_force3, *_force1);

        /// dubug
        // hy:由adrcoutput算出的controller_x_position和controller_y_position
        cout << "**************************G型力的解算**************************" << endl;
        cout << " _controller_forward: " << _controller_forward
             << " _controller_vertical: " << _controller_vertical
             << "_controller_torque: " << _controller_torque
             << endl;

        cout << " forceBack: " << forceBack << " forceForward: " << forceForward
             << " forceVertical: " << forceVertical << " forceToque: " << forceToque << endl;
        cout << " _force1: " << *_force1 << " _force2: " << *_force2
             << " _force3: " << *_force3 << " _force4: " << *_force4 << endl;


        // hy:角度变化量
        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle << endl;

        /// 如果力超出区间,才映射到区间内
        if (__min_force < (base_motor_force - bias_motor_force) || __max_force > (base_motor_force + bias_motor_force))
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force1);
            *_force3 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force3);
        }
    }
    void C_VectoredThrustControl::calcSprayForcesGrasp_Shape_G(float _controller_forward, float _controller_horizontal, float _controller_vertical, float _controller_torque,
                                                               float *_force1, float *_force2, float *_force3, float *_force4,
                                                               float *_leg1_angle, float *_leg2_angle, float *_leg3_angle, float *_leg4_angle,
                                                               float _force1_direction, float _object_direction)
    {
        float forceBack, forceForward, forceVertical, forceToque, forceHorizontal;
        double force1, force2;
        

        float __min_force, __max_force;
        float deta_angle1, deta_angle3;

        convertToObliqueCoordinates(_force1_direction, _object_direction, _controller_forward, force1, force2);

        forceBack = base_motor_force - force1/2;
        forceForward = base_motor_force + force1/2;
        forceVertical = _controller_vertical/2;        
        deta_angle1 = atan2(forceVertical, forceBack+0.0001);
        deta_angle3 = atan2(forceVertical, forceForward+0.0001);       
        if(deta_angle1 > 30) deta_angle1 = 30;
        if(deta_angle1 < -30) deta_angle1 = -30;
        if(deta_angle3 > 30) deta_angle3 = 30;
        if(deta_angle3 < -30) deta_angle3 = -30;

        forceHorizontal = force2/2;
        forceToque = _controller_torque/0.3;        
        if(forceToque > 4) forceToque = 4;
        if(forceToque < -4) forceToque = -4;


        // 超过了角度范围,说明z方向的力比较大,为了保证z轴的这个力,要增加x方向的力

        // hy:转换为角度制
        *_leg1_angle = -110 + deta_angle1 * 57.3;
        *_leg3_angle = -110 + deta_angle3 * 57.3;

        // hy:对于每个腿而言的x,z方向的合力
        //  再求合力
        *_force1 = sqrt(forceBack * forceBack + forceVertical * forceVertical);
        *_force2 = forceToque - forceHorizontal;
        *_force3 = sqrt(forceForward * forceForward + forceVertical * forceVertical);
        *_force4 = forceToque + forceHorizontal;

        // hy:对于机器人而言的四个推进器中的最小力和最大力
        //  考虑到合外力会超过力的限制 将力映射到对应的区间内
        __min_force = min(*_force3, *_force1);
        __max_force = max(*_force3, *_force1);

        /// dubug
        // hy:由adrcoutput算出的controller_x_position和controller_y_position
        cout << "**************************G型力的解算**************************" << endl;
        // cout << " _controller_forward: " << _controller_forward
        //      << " _controller_vertical: " << _controller_vertical
        //      << "_controller_torque: " << _controller_torque
        //      << endl;

        cout << " forceBack: " << forceBack << " forceForward: " << forceForward
             << " forceVertical: " << forceVertical << " forceToque: " << forceToque << endl
             <<"  forceHorizontal: "<<forceHorizontal<<endl;;
        cout << " _force1: " << *_force1 << " _force2: " << *_force2
             << " _force3: " << *_force3 << " _force4: " << *_force4 << endl;


        // hy:角度变化量
        cout << " *leg1_angle: " << *_leg1_angle << " *leg2_angle: " << *_leg2_angle
             << " *leg3_angle: " << *_leg3_angle << " *leg4_angle: " << *_leg4_angle << endl;

        /// 如果力超出区间,才映射到区间内
        if (__min_force < (base_motor_force - bias_motor_force) || __max_force > (base_motor_force + bias_motor_force))
        {
            // 将对应的力映射到限定区间上
            *_force1 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force1);
            *_force3 = mapForce(__min_force, __max_force, base_motor_force - bias_motor_force, base_motor_force + bias_motor_force, *_force3);
        }
    }

    void C_VectoredThrustControl::thrusterforce2speed_G(float _force1, float _force2, float _force3, float _force4)
    {

        // 推力在1到3之间，转速在100到500之间
        int __speed1, __speed2, __speed3, __speed4;
        float __k_1, __b_1;
        // float __k_2, __b_2;
        float __k_3, __b_3;
        // float __k_4, __b_4;

        __k_1 = (g_shape_max_motor_speed1 - g_shape_min_motor_speed1) / (2 * bias_motor_force);
        __b_1 = g_shape_min_motor_speed1 - __k_1 * (base_motor_force - bias_motor_force);

        // __k_2 = (g_shape_max_motor_speed2 - g_shape_min_motor_speed2) / (2 * bias_motor_force);
        // __b_2 = g_shape_min_motor_speed2 - __k_2 * (base_motor_force - bias_motor_force);

        __k_3 = (g_shape_max_motor_speed3 - g_shape_min_motor_speed3) / (2 * bias_motor_force);
        __b_3 = g_shape_min_motor_speed3 - __k_3 * (base_motor_force - bias_motor_force);

        // __k_4 = (g_shape_max_motor_speed4 - g_shape_min_motor_speed4) / (2 * bias_motor_force);
        // __b_4 = g_shape_min_motor_speed4 - __k_4 * (base_motor_force - bias_motor_force);

        __speed1 = __k_1 * _force1 + __b_1;
        // __speed2 = __k_2 * _force2 + __b_2;
        __speed3 = __k_3 * _force3 + __b_3;
        // __speed4 = __k_4 * _force4 + __b_4;
        __speed1 = _force1*25;
        __speed2 = _force2*25;
        __speed3 = _force3*25;
        __speed4 = _force4*25;

        // 将转速锁定在对应电机的区间内部

        if (__speed1 < g_shape_min_motor_speed1)
            __speed1 = g_shape_min_motor_speed1;
        else if (__speed1 > g_shape_max_motor_speed1)
            __speed1 = g_shape_max_motor_speed1;

        if (__speed2 < g_shape_min_motor_speed2)
            __speed2 = g_shape_min_motor_speed2;
        else if (__speed2 > g_shape_max_motor_speed2)
            __speed2 = g_shape_max_motor_speed2;

        if (__speed3 < g_shape_min_motor_speed3)
            __speed3 = g_shape_min_motor_speed3;
        else if (__speed3 > g_shape_max_motor_speed3)
            __speed3 = g_shape_max_motor_speed3;

        if (__speed4 < g_shape_min_motor_speed4)
            __speed4 = g_shape_min_motor_speed4;
        else if (__speed4 > g_shape_max_motor_speed4)
            __speed4 = g_shape_max_motor_speed4;
        
        pwm_msg.rotor_speed1 = __speed1;
        pwm_msg.rotor_speed2 = __speed2;
        pwm_msg.rotor_speed3 = __speed3;
        pwm_msg.rotor_speed4 = __speed4;

        extern_pub_pwm->publish(pwm_msg); // hcy
        cout << "**************************推进器力转速度**************************" << endl;
        // motorspeed(__speed1, __speed2, __speed3, __speed4);
        printf("speed1:%d speed2:%d speed3:%d speed4:%d\n", __speed1, __speed2, __speed3, __speed4);
    }

    // hy:（重要）运动最底层的总函数
    //  3维运动控制解算 总函数
    /**
     * @description: 3维运动控制解算
     * @param {int} _leg_shape        形状
     * @param {float} _X_force          controller_x_position
     * @param {float} _Y_force          controller_y_position
     * @param {float} _Z_force          controller_z_position
     * @param {float} _yaw_force    controller_yaw
     * @return {*}
     * @author: LJL
     */
    void C_VectoredThrustControl::space3DControl(int _leg_shape, float _X_force, float _Y_force, float _Z_force, float _yaw_force)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        switch (_leg_shape)
        {
        case SHAPE_H:
        {

            calcSprayForces_Shape_H_2(_X_force, _Z_force, _yaw_force,
                                      &__force1, &__force2, &__force3, &__force4,
                                      &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);
            yawLocomMode_FB(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed_H(__force1, __force2, __force3, __force4);
        };
        break;

        case SHAPE_H_T:
        {
            calcSprayForces_Shape_H_T_2(_Y_force, _Z_force, _yaw_force,
                                        &__force1, &__force2, &__force3, &__force4,
                                        &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);

            yawLocomMode_LR(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed(__force1, __force2, __force3, __force4);
        };
        break;

        case SHAPE_X:
        {
            calcSprayForces_Shape_X_2(_X_force, _Y_force, _Z_force,
                                      &__force1, &__force2, &__force3, &__force4,
                                      &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);

            yawLocomMode_X(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed_X(__force1, __force2, __force3, __force4);
        };
        break;
        case SHAPE_G:
            calcSprayForces_Shape_G(_X_force, _Z_force, _yaw_force,
                                    &__force1, &__force2, &__force3, &__force4,
                                    &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4);

            yawLocomMode_G(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed_G(__force1, __force2, __force3, __force4);
            break;
        default:
        {
            cout << "please choose a correct leg mode!!!" << endl;
        };
        break;
        }

        thrust_force_1 = __force1;
        thrust_force_2 = __force2;
        thrust_force_3 = __force3;
        thrust_force_4 = __force4;
        cout << "**************************腿部力和角度**************************" << endl;
        cout << "force1: " << __force1 << "    force2: " << __force2
             << "    force3: " << __force3 << "    force4: " << __force4 << endl;

        cout << "angle1: " << __leg_angle1 << "    angle2: " << __leg_angle2
             << "    angle3: " << __leg_angle3 << "    angle4: " << __leg_angle4 << endl;
    }

    void C_VectoredThrustControl::space3DGraspControl(int _leg_shape, float _forward_force, float _horizontal_force, float _vertical_force, float torque, float _force1_direction, float _object_direction)
    {
        float __force1, __force2, __force3, __force4;
        float __leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4;

        switch (_leg_shape)
        {
        case SHAPE_G:
            calcSprayForcesGrasp_Shape_G(_forward_force, _horizontal_force, _vertical_force, torque,
                                    &__force1, &__force2, &__force3, &__force4,
                                    &__leg_angle1, &__leg_angle2, &__leg_angle3, &__leg_angle4, _force1_direction, _object_direction);

            yawLocomMode_G(__leg_angle1, __leg_angle2, __leg_angle3, __leg_angle4);
            thrusterforce2speed_G(__force1, __force2, __force3, __force4);
            break;
        default:
        {
            cout << "please choose a correct leg mode!!!" << endl;
        };
        break;
        }

        thrust_force_1 = __force1;
        thrust_force_2 = __force2;
        thrust_force_3 = __force3;
        thrust_force_4 = __force4;
        cout << "**************************腿部力和角度**************************" << endl;
        cout << "force1: " << __force1 << "    force2: " << __force2
             << "    force3: " << __force3 << "    force4: " << __force4 << endl;

        cout << "angle1: " << __leg_angle1 << "    angle2: " << __leg_angle2
             << "    angle3: " << __leg_angle3 << "    angle4: " << __leg_angle4 << endl;
    }
}
