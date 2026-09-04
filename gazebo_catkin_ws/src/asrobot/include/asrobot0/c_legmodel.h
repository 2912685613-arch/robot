#ifndef C_LEGMODEL_H
#define C_LEGMODEL_H

#include "ros/ros.h"
#include "asrobot/pwm.h"
#include "std_msgs/String.h"

#include <iostream>
#include <stdint.h>
#include <math.h>
#include <stdio.h>


#define PI     3.1415926535
#define COXA   99.43
#define FEMUR  92.00
//#define TIBIA  32.75
#define TIBIA  102.00

#define GAIT_STOP    0
#define GAIT_FORWARD 1
#define GAIT_BACK    2
#define GAIT_LEFT    3
#define GAIT_RIGHT   4



// 常量
//PWM channel
//const int leg1_pwm_channel[3] = {16,15,14};//leg1_pwm_channel
//const int leg2_pwm_channel[3] = {4,1,20};//leg2_pwm_channel
//const int leg3_pwm_channel[3] = {18,19,17};//leg3_pwm_channel
//const int leg4_pwm_channel[3] = {13,12,11};//leg4_pwm_channel

const int leg1_pwm_channel[3] = {0,1,2};//leg1_pwm_channel
const int leg2_pwm_channel[3] = {3,4,5};//leg2_pwm_channel
const int leg3_pwm_channel[3] = {6,7,8};//leg3_pwm_channel
const int leg4_pwm_channel[3] = {9,10,11};//leg4_pwm_channel

//The real angle when gazebo angle is 0
// const int leg1_duty_0[3] = {0,0,10};
// const int leg2_duty_0[3] = {0,0,10};
// const int leg3_duty_0[3] = {0,-2,10};
// const int leg4_duty_0[3] = {0,0,10};
const int leg1_duty_0[3] = {0,-2,-14};//hcy
const int leg2_duty_0[3] = {0,0,-14};
const int leg3_duty_0[3] = {0,0,-14};
const int leg4_duty_0[3] = {0,0,-14};
// const int leg1_duty_0[3] = {0,0,0};//hcy
// const int leg2_duty_0[3] = {0,0,0};
// const int leg3_duty_0[3] = {0,0,0};
// const int leg4_duty_0[3] = {0,0,0};

//angle factor
const float leg1_k[3] = {1,1,1};
const float leg2_k[3] = {1,1,1};
const float leg3_k[3] = {1,1,1};
const float leg4_k[3] = {1,1,1};

//const float leg1_k[3] = {11.3,-11.53,11.2};
//const float leg2_k[3] = {-11.3,-11.53,11.2};
//const float leg3_k[3] = {11.3,-11.53,11.2};
//const float leg4_k[3] = {-11.3,-11.53,11.2};

//Left limit
//const int leg1_left[3] = {1100,750,500};
//const int leg2_left[3] = {1030,650,500};
//const int leg3_left[3] = {1000,610,500};
//const int leg4_left[3] = {1040,500,500};

//Right limit
//const int leg1_right[3] = {1984,2400,2500};
//const int leg2_right[3] = {1964,2400,2480};
//const int leg3_right[3] = {1950,2320,2480};
//const int leg4_right[3] = {1950,2400,2480};

//const int leg1_right[3] = {1984,2400,2500};
//const int leg2_right[3] = {1964,2400,2480};
//const int leg3_right[3] = {1950,2400,2480};
//const int leg4_right[3] = {1950,2400,2480};

namespace LegModel {

class leg
{

// 变量
private:
    int A0_duty, B0_duty, C0_duty; //the duty when angle is 0
    float A_k, B_k, C_k;
    int servo_A, servo_B, servo_C; //the PWM channel of each servo
//    int A_duty_left, B_duty_left, C_duty_left; //left limit
//    int A_duty_right, B_duty_right, C_duty_right; //right limit
    void cartesian_to_polar(float &A, float &B, float &C, float x, float y, float z);//position to angle
    void polar_to_cartesian(float A, float B, float C, float &x, float &y, float &z);//angle to position


// 函数
public:
    leg(const int pwm_channel[], const int duty_0[], const float duty_k[]);

    float A_now, B_now, C_now;//servo angle
    float x_now, y_now, z_now;//leg position
    void set_position(float x, float y, float z, int time = 500, uint8_t type = 0);
    void set_angle(float A, float B, float C, int time = 500, uint8_t type = 0);


};

}// namespace 后面的括号



#endif // C_LEGMODEL_H
