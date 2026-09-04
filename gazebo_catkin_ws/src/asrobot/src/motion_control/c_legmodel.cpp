#include "../../include/asrobot/c_legmodel.h"

extern asrobot::pwm pwm_msg;
extern ros::Publisher* extern_pub_pwm;

namespace LegModel {

// 初始化舵机
leg::leg(const int pwm_channel[], const int duty_0[], const float duty_k[])
{

        servo_A = pwm_channel[0]; servo_B = pwm_channel[1]; servo_C = pwm_channel[2];
             A0_duty = duty_0[0];      B0_duty = duty_0[1];      C0_duty = duty_0[2];
                 A_k = duty_k[0];          B_k = duty_k[1];          C_k = duty_k[2];
//           A_duty_left = left[0];    B_duty_left = left[1];    C_duty_left = left[2];
//         A_duty_right = right[0];  B_duty_right = right[1];  C_duty_right = right[2];
        A_now =-100; B_now = -100; C_now = -100;
        x_now = COXA + FEMUR + TIBIA;
        //set_angle(0, 45, 0, 100);

}

// 直角转极坐标
void leg::cartesian_to_polar(float &A, float &B, float &C, float x, float y, float z)//position to angle
{
    float alpha, beta, gamma;
    float nx, ny;
    //calculate w-z degree
    float v, w;
    nx = 0.70710678*x + 0.70710678*y;
    ny = -0.70710678*x + 0.70710678*y;
    x = nx; y = ny;
    w = (x >= 0 ? 1 : -1)*(sqrt(pow(x, 2) + pow(y, 2)));
    v = w - COXA;
    alpha = atan2(z, v) + acos((pow(FEMUR, 2) - pow(TIBIA, 2) + pow(v, 2) + pow(z, 2)) / 2 / FEMUR / sqrt(pow(v, 2) + pow(z, 2)));
    beta = acos((pow(FEMUR, 2) + pow(TIBIA, 2) - pow(v, 2) - pow(z, 2)) / 2 / FEMUR / TIBIA);
    //calculate x-y-z degree
    gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
    //trans degree pi->180#include "../../include/asrobot/legmodel.h"
    alpha = alpha / PI * 180;
    beta = beta / PI * 180;
    gamma = gamma / PI * 180;
    A = gamma; B = -alpha; C = 180 - beta;
}

// 极坐标转直角坐标
void leg::polar_to_cartesian(float A, float B, float C, float &x, float &y, float &z)//angle to position
{
    float alpha, beta, gamma;
    float nx, ny;
    alpha = -B; beta = 180 - C; gamma = A;
    //trans degree 180->pi
    alpha = alpha / 180 * PI;
    beta = beta / 180 * PI;
    gamma = gamma / 180 * PI;
    //calculate w-z position
    float v, w;
    v = FEMUR*cos(alpha) - TIBIA*cos(alpha + beta);
    z = FEMUR*sin(alpha) - TIBIA*sin(alpha + beta);
    w = v + COXA;
    //calculate x-y-z position
    x = w*cos(gamma);
    y = w*sin(gamma);
    nx = 0.70710678*x - 0.70710678*y;
    ny = 0.70710678*x + 0.70710678*y;
    x = nx; y = ny;
}

// 设置舵机的位置
void leg::set_position(float x, float y, float z, int time, uint8_t type)
{
    float A, B, C;
    int A_duty, B_duty, C_duty;
    cartesian_to_polar(A, B, C, x, y, z);
    //modify angle
    A_duty = A0_duty + A*A_k;
    B_duty = B0_duty + B*B_k;
    C_duty = C0_duty + C*C_k;
    //check limit
//    if(A_duty > A_duty_right) A_duty = A_duty_right;
//        else if(A_duty < A_duty_left) A_duty = A_duty_left;

//    if(B_duty > B_duty_right) B_duty = B_duty_right;
//        else if(B_duty < B_duty_left) B_duty = B_duty_left;

//    if(C_duty > C_duty_right) C_duty = C_duty_right;
//        else if(C_duty < C_duty_left) C_duty = C_duty_left;
    //output by serialport
//    dA = A - A_now;
//    dB = B - B_now;
//    dC = C - C_now;
//    if(dA < 0) dA = -dA;
//    if(dB < 0) dB = -dB;
//    if(dC < 0) dC = -dC;
//    speedA = (short)(dA*1000/time/9);
//    if(speedA<1) speedA = 1;
//    speedB = (short)(dB*1000/time/9);
//    if(speedB<1) speedB = 1;
//    speedC = (short)(dC*1000/time/9);
//    if(speedC<1) speedC = 1;


    pwm_msg.channel.push_back(servo_A);
    pwm_msg.type.push_back(type);
    pwm_msg.duty.push_back(A_duty);//gazebo_output angle
    pwm_msg.time.push_back(time);

    pwm_msg.channel.push_back(servo_B);
    pwm_msg.type.push_back(type);
    pwm_msg.duty.push_back(B_duty);
    pwm_msg.time.push_back(time);

    pwm_msg.channel.push_back(servo_C);
    pwm_msg.type.push_back(type);
    pwm_msg.duty.push_back(C_duty);
    pwm_msg.time.push_back(time);

    extern_pub_pwm->publish(pwm_msg);
    pwm_msg.channel.clear();
    pwm_msg.type.clear();
    pwm_msg.duty.clear();
    pwm_msg.time.clear();



    //update the realtime position and angle
    A_now = A; B_now = B; C_now = C;
    x_now = x; y_now = y; z_now = z;
   printf("set angle:%.2f %.2f %.2f     ", A,B,C);
   printf("set position:%.2f %.2f %.2f\n", x,y,z);
}


//设置舵机转角

void leg::set_angle(float A, float B, float C, int time, uint8_t type)
{
    float x, y, z;
    int A_duty, B_duty, C_duty;
//    float dA, dB, dC;
//    short speedA, speedB, speedC;
//    int A_change=1, B_change=1, C_change=1;
//    /*********************/
//   if(A-A_now<0.1&&A-A_now>-0.1)
//    {
//        A = A_now;
//        A_change=0;
//    }
//    if(B-B_now<0.1&&B-B_now>-0.1)
//    {
//        B = B_now;
//        B_change=0;
//    }
//    if(C-C_now<0.1&&C-C_now>-0.1)
//    {
//        C = C_now;
//        C_change=0;
//    }
    /*********************/
    polar_to_cartesian(A, B, C, x, y, z);
    //caculate duty by angle
    A_duty = A0_duty + A*A_k;
    B_duty = B0_duty + B*B_k;
    C_duty = C0_duty + C*C_k;
//    //check limit
//    if(A_duty > A_duty_right) A_duty = A_duty_right;
//        else if(A_duty < A_duty_left) A_duty = A_duty_left;

//    if(B_duty > B_duty_right) B_duty = B_duty_right;
//        else if(B_duty < B_duty_left) B_duty = B_duty_left;

//    if(C_duty > C_duty_right) C_duty = C_duty_right;
//        else if(C_duty < C_duty_left) C_duty = C_duty_left;
//    //output by serialport
//    dA = A - A_now;
//    dB = B - B_now;
//    dC = C - C_now;
//    if(dA < 0) dA = -dA;
//    if(dB < 0) dB = -dB;
//    if(dC < 0) dC = -dC;
//    speedA = (short)(dA*1000/time/9);
//    if(speedA<1) speedA = 1;
//    speedB = (short)(dB*1000/time/9);
//    if(speedB<1) speedB = 1;
//    speedC = (short)(dC*1000/time/9);
//    if(speedC<1) speedC = 1;



    pwm_msg.channel.push_back(servo_A);
    pwm_msg.type.push_back(type);
    pwm_msg.duty.push_back(A_duty);
    pwm_msg.time.push_back(time);

    pwm_msg.channel.push_back(servo_B);
    pwm_msg.type.push_back(type);
    pwm_msg.duty.push_back(B_duty);
    pwm_msg.time.push_back(time);

    pwm_msg.channel.push_back(servo_C);
    pwm_msg.type.push_back(type);
    pwm_msg.duty.push_back(C_duty);
    pwm_msg.time.push_back(time);

    extern_pub_pwm->publish(pwm_msg);
    pwm_msg.channel.clear();
    pwm_msg.type.clear();
    pwm_msg.duty.clear();
    pwm_msg.time.clear();
    //update the realtime position and angle
    A_now = A; B_now = B; C_now = C;
    x_now = x; y_now = y; z_now = z;
    printf("**************************极坐标和直角坐标下参数*******************\n");
    printf("set angle:%.2f %.2f %.2f     ", A, B, C);
    printf("set position:%.2f %.2f %.2f\n", x, y, z);
}





} // namespace后面的括号
