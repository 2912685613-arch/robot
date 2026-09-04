#ifndef LEG_H
#define LEG_H

#define PI     3.1415926
#define COXA   99.43
#define FEMUR  92.00
//#define TIBIA  32.75
#define TIBIA  102.00

#define GAIT_STOP    0
#define GAIT_FORWARD 1
#define GAIT_BACK    2
#define GAIT_LEFT    3
#define GAIT_RIGHT   4

//PWM channel

//const int leg1_pwm_channel[3] = {16,15,14};//leg1_pwm_channel
//const int leg2_pwm_channel[3] = {4,1,20};//leg2_pwm_channel
//const int leg3_pwm_channel[3] = {18,19,17};//leg3_pwm_channel
//const int leg4_pwm_channel[3] = {13,12,11};//leg4_pwm_channel

const int leg1_pwm_channel[3] = {0,1,2};//leg1_pwm_channel
const int leg2_pwm_channel[3] = {3,4,5};//leg2_pwm_channel
const int leg3_pwm_channel[3] = {6,7,8};//leg3_pwm_channel
const int leg4_pwm_channel[3] = {9,10,11};//leg4_pwm_channel

//The duty when angle is 0
const int leg1_duty_0[3] = {1420,1920,1560};
const int leg2_duty_0[3] = {1470,2040,1500};
const int leg3_duty_0[3] = {1440,2020,1600};
const int leg4_duty_0[3] = {1530,1910,1420};

//Duty_k
const float leg1_k[3] = {12,-12,12};
const float leg2_k[3] = {-12,-12,12};
const float leg3_k[3] = {12,-12,12};
const float leg4_k[3] = {-12,-12,12};

//Left limit
const int leg1_left[3] = {820,510,620};
const int leg2_left[3] = {860,540,660};
const int leg3_left[3] = {840,550,720};
const int leg4_left[3] = {890,520,1000};

//Right limit
const int leg1_right[3] = {2020,2500,2500};
const int leg2_right[3] = {2060,2500,2500};
const int leg3_right[3] = {2020,2500,2500};
const int leg4_right[3] = {2090,2500,2500};

////Left limit
//int const leg1_left[3] = {966,960,1240};
//int const leg2_left[3] = {1010,1040,1190};
//int const leg3_left[3] = {1070,1090,1000};
//int const leg4_left[3] = {1059,1013,1200};

////Right limit
//int const leg1_right[3] = {1876,2420,2260};
//int const leg2_right[3] = {1830,2410,2179};
//int const leg3_right[3] = {1920,2460,2050};
//int const leg4_right[3] = {1959,2413,2145};


class leg
{
public:
    leg(const int pwm_channel[], const int duty_0[], const float duty_k[], const int left[], const int right[]);

    float A_now, B_now, C_now;//servo angle
    float x_now, y_now, z_now;           //leg position
    void set_position(float x, float y, float z, int time);
    void set_angle(float A, float B, float C, int time);
    
private:
    int A0_duty, B0_duty, C0_duty; //the duty when angle is 0
    float A_k, B_k, C_k;
    int servo_A, servo_B, servo_C; //the PWM channel of each servo
    int A_duty_left, B_duty_left, C_duty_left; //left limit
    int A_duty_right, B_duty_right, C_duty_right; //right limit
    void cartesian_to_polar(float &A, float &B, float &C, float x, float y, float z);//position to angle
    void polar_to_cartesian(float A, float B, float C, float &x, float &y, float &z);//angle to position
};


#endif // LEG_H
