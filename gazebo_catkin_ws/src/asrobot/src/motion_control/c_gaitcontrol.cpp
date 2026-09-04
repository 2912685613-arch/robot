#include "../../include/asrobot/c_gaitcontrol.h"

namespace GaitControl {

leg Leg1(leg1_pwm_channel, leg1_duty_0, leg1_k);
leg Leg2(leg2_pwm_channel, leg2_duty_0, leg2_k);
leg Leg3(leg3_pwm_channel, leg3_duty_0, leg3_k);
leg Leg4(leg4_pwm_channel, leg4_duty_0, leg4_k);

// 定义步态枚举
GaitStatu gaitstatu = Stop;

C_GaitControl::C_GaitControl()
{
    cout << "gait control is initialized !!!!!"<<endl;
}

// 前进
void C_GaitControl::to_forward()
{
    Leg1.set_position(GAIT_X, 67, 30, 800);
    Leg2.set_position(GAIT_X, 160, 30, 800);
    Leg3.set_position(GAIT_X, 86, 30, 800);
    Leg4.set_position(GAIT_X, 141, 30, 800);
    sleep(1);
    Leg1.set_position(GAIT_X, 67, -150, 1500);
    Leg2.set_position(GAIT_X, 160, -150, 1500);
    Leg3.set_position(GAIT_X, 86, -150, 1500);
    Leg4.set_position(GAIT_X, 141, -150, 1500);
    usleep(1500000);

    Leg1.set_position(GAIT_X, 30, -150, 2*GAITTIME/8, 0);
    Leg3.set_position(GAIT_X, 160, -150, 4*GAITTIME/8, 0);
    Leg4.set_position(GAIT_X, 30, -150, 6*GAITTIME/8, 0);

    printf("change to forward!\n\n");

}

// 后退

void C_GaitControl::to_back()
{
    Leg1.set_position(GAIT_X, 160, 30, 800);
    Leg2.set_position(GAIT_X, 67, 30, 800);
    Leg3.set_position(GAIT_X, 141, 30, 800);
    Leg4.set_position(GAIT_X, 86, 30, 800);
    sleep(1);
    Leg1.set_position(GAIT_X, 160, -150, 1500);
    Leg2.set_position(GAIT_X, 67, -150, 1500);
    Leg3.set_position(GAIT_X, 141, -150, 1500);
    Leg4.set_position(GAIT_X, 86, -150, 1500);
    usleep(1500000);

    Leg2.set_position(GAIT_X, 30, -150, 2*GAITTIME/8, 0);
    Leg4.set_position(GAIT_X, 160, -150, 4*GAITTIME/8, 0);
    Leg3.set_position(GAIT_X, 30, -150, 6*GAITTIME/8, 0);

    printf("change to back!\n\n");
}

// 左转
void C_GaitControl::to_left(float _sita1, float _sita2)
{
    Leg1.set_position(GAIT_X - 65*sin(_sita1), 95 - 65*cos(_sita1), 30, 800, 0);
    Leg2.set_position(GAIT_X - 65*sin(_sita1) + 2*65*sin(_sita1)*5/7, 95 - 65*cos(_sita1) + 2*65*cos(_sita1)*5/7, 30, 800, 0);
    Leg3.set_position(GAIT_X + 65*sin(_sita2) - 2*65*sin(_sita2)*3/7, 95 - 65*cos(_sita2) + 2*65*cos(_sita2)*3/7, 30, 800, 0);
    Leg4.set_position(GAIT_X - 65*sin(_sita2) + 2*65*sin(_sita2)*1/7, 95 + 65*cos(_sita2) - 2*65*cos(_sita2)*1/7, 30, 800, 0);
    sleep(1);

    Leg1.set_position(GAIT_X - 65*sin(_sita1), 95 - 65*cos(_sita1), -150, 1500, 0);
    Leg2.set_position(GAIT_X - 65*sin(_sita1) + 2*65*sin(_sita1)*5/7, 95 - 65*cos(_sita1) + 2*65*cos(_sita1)*5/7, -150, 1500, 0);
    Leg3.set_position(GAIT_X + 65*sin(_sita2) - 2*65*sin(_sita2)*3/7, 95 - 65*cos(_sita2) + 2*65*cos(_sita2)*3/7, -150, 1500, 0);
    Leg4.set_position(GAIT_X - 65*sin(_sita2) + 2*65*sin(_sita2)*1/7, 95 + 65*cos(_sita2) - 2*65*cos(_sita2)*1/7, -150, 1500, 0);
    usleep(1500000);

    Leg2.set_position(GAIT_X + 65*sin(_sita1), 95 + 65*cos(_sita1), -150, 2*GAITTIME/8, 0);
    Leg3.set_position(GAIT_X - 65*sin(_sita2), 95 + 65*cos(_sita2), -150, 4*GAITTIME/8, 0);
    Leg4.set_position(GAIT_X + 65*sin(_sita2), 95 - 65*cos(_sita2), -150, 6*GAITTIME/8, 0);

    printf("change to left!\n\n");
}


// 右转
void C_GaitControl::to_right(float _sita1, float _sita2)
{
    Leg2.set_position(GAIT_X + 65*sin(_sita1), 95 + 65*cos(_sita1), 30, 800, 0);
    Leg1.set_position(GAIT_X + 65*sin(_sita1) - 2*65*sin(_sita1)*5/7, 95 + 65*cos(_sita1) - 2*65*cos(_sita1)*5/7, 30, 800, 0);
    Leg3.set_position(GAIT_X + 65*sin(_sita2) - 2*65*sin(_sita2)*1/7, 95 - 65*cos(_sita2) + 2*65*cos(_sita2)*1/7, 30, 800, 0);
    Leg4.set_position(GAIT_X - 65*sin(_sita2) + 2*65*sin(_sita2)*3/7, 95 + 65*cos(_sita2) - 2*65*cos(_sita2)*3/7, 30, 800, 0);
    sleep(1);

    Leg2.set_position(GAIT_X + 65*sin(_sita1), 95 + 65*cos(_sita1), -150, 1500, 0);
    Leg1.set_position(GAIT_X + 65*sin(_sita1) - 2*65*sin(_sita1)*5/7, 95 + 65*cos(_sita1) - 2*65*cos(_sita1)*5/7, -150, 1500, 0);
    Leg3.set_position(GAIT_X + 65*sin(_sita2) - 2*65*sin(_sita2)*1/7, 95 - 65*cos(_sita2) + 2*65*cos(_sita2)*1/7, -150, 1500, 0);
    Leg4.set_position(GAIT_X - 65*sin(_sita2) + 2*65*sin(_sita2)*3/7, 95 + 65*cos(_sita2) - 2*65*cos(_sita2)*3/7, -150, 1500, 0);
    usleep(1500000);

    Leg1.set_position(GAIT_X - 65*sin(_sita1), 95 - 65*cos(_sita1), -150, 2*GAITTIME/8, 0);
    Leg4.set_position(GAIT_X + 65*sin(_sita2), 95 - 65*cos(_sita2), -150, 4*GAITTIME/8, 0);
    Leg3.set_position(GAIT_X - 65*sin(_sita2), 95 + 65*cos(_sita2), -150, 6*GAITTIME/8, 0);

    printf("change to right!\n\n");
}


// 拐弯前进
void C_GaitControl::forward4(float _r)
{
    float R = abs(_r);
    if(R < 200) R=0;//if R is very small, R=0
    float L = 2 * (GAIT_YHEAD - GAIT_YTAIL) + 53;
    float W = 2 * GAIT_X + 53;

    float l3 = pow(pow(R, 2) + pow(L/2, 2), 0.5);
    float sita3 = atan(L/2/R);

    float R1 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(sita3)*W/2*l3), 0.5);
    float R2 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(3.14159265 - sita3)*W/2*l3), 0.5);


    float value = (pow(R1, 2) + pow(W, 2) - pow(R2, 2))/(2*R1*W);
    if(value > 1)
    {
        printf("warning! acos1() input value > 1\n");
    }
    float sita1 = 3.14159265 - acos(value);

    value = (pow(R2, 2) + pow(W, 2) - pow(R1, 2))/(2*R2*W);
    if(value > 1)
    {
        printf("warning! acos2() input value > 1\n");
    }
    float sita2 = acos(value);

    float kspeed = R1/R2;

    if( _r<0 )
    {
        float temp = sita2;
        sita2 = -sita1;
        sita1 = -temp;

        temp = R1;
        R1 = R2;
        R2 = temp;
    }

    printf("R1=%.2f R2=%.2f sita1=%.2f sita2=%.2f kspeed=%.4f \n", R1, R2, sita1*180/3.1415, sita2*180/3.1415, kspeed);

    if(_r >= 200) //left and forward
    {
        if(gaitstatu != Forward) to_forward();
        gaitstatu = Forward;

	//hcy
        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1)*kspeed, 95 + 65*cos(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1)*kspeed, 95 - 65*cos(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2), 95 - 65*cos(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2), 95 + 65*cos(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2), 95 + 65*cos(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2), 95 - 65*cos(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1)*kspeed, 95 - 65*cos(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1)*kspeed, 95 + 65*cos(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);
        
/*
        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1)*kspeed, 95 - 65*cos(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1)*kspeed, 95 + 65*cos(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1)*kspeed, 95 + 65*cos(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1)*kspeed, 95 - 65*cos(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2), 95 - 65*cos(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2), 95 + 65*cos(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2), 95 + 65*cos(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2), 95 - 65*cos(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

*/
        printf("left and forward\n");
    }
    else if(_r <= -200)  //right and forward
    {
        if(gaitstatu != Forward) to_forward();
        gaitstatu = Forward;


        //hcy
        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

/*
        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

*/
        printf("right and forward\n");
    }
    else if(_r >= 0 && _r < 200)//left
    {
        if(gaitstatu != Left) to_left(sita1, sita2);
        gaitstatu = Left;

        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);
        printf("left\n");
    }
    else if((_r < 0) && (_r > -200))//right
    {
        if(gaitstatu != Right) to_right(sita1, sita2);
        gaitstatu = Right;

        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);
        printf("right\n");
    }

}


void C_GaitControl::right()
{
    float sita1 = -3.14159265/2, sita2 = -3.14159265/2;
    float kp = 0.8;
    Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg2.set_position(GAIT_X + 65*sin(sita1)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg2.set_position(GAIT_X - 65*sin(sita1)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);

    Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg3.set_position(GAIT_X - 65*sin(sita2)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg3.set_position(GAIT_X + 65*sin(sita2)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);


    Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg1.set_position(GAIT_X + 65*sin(sita1)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg1.set_position(GAIT_X - 65*sin(sita1)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);

    Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg4.set_position(GAIT_X - 65*sin(sita2)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg4.set_position(GAIT_X + 65*sin(sita2)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);
}



void C_GaitControl::left()
{
    float sita1 = 3.14159265/2, sita2 = 3.14159265/2;
    float kp=0.8;
    Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg3.set_position(GAIT_X - 65*sin(sita2)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg3.set_position(GAIT_X + 65*sin(sita2)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);

    Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg2.set_position(GAIT_X + 65*sin(sita1)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg2.set_position(GAIT_X - 65*sin(sita1)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);

    Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg4.set_position(GAIT_X - 65*sin(sita2)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg4.set_position(GAIT_X + 65*sin(sita2)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);

    Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg1.set_position(GAIT_X + 65*sin(sita1)*kp, 95, -150, GAITTIME/16, 1);
    usleep(1000*GAITTIME/16);
    Leg1.set_position(GAIT_X - 65*sin(sita1)*kp, 95, -150, 7*GAITTIME/8, 0);
    usleep(1000*GAITTIME/8);
}


/***********************dui jiao***************************/
/**
 * @brief forward2
 *          Trotting gait
 */
void C_GaitControl::forward2()
{
    Leg2.set_position(GAIT_X, 95, -50, GAITTIME/4, 1);
    Leg4.set_position(GAIT_X, 95, -50, GAITTIME/4, 1);
    usleep(1000*GAITTIME/4);
    Leg2.set_position(GAIT_X, 95 - 65, -150, GAITTIME/4, 1);
    Leg4.set_position(GAIT_X, 95 + 65, -150, GAITTIME/4, 1);
    usleep(1000*GAITTIME/4);
    Leg2.set_position(GAIT_X, 95 + 65, -150, GAITTIME/2, 0);
    Leg4.set_position(GAIT_X, 95 - 65, -150, GAITTIME/2, 0);

    Leg1.set_position(GAIT_X, 95, -50, GAITTIME/4, 1);
    Leg3.set_position(GAIT_X, 95, -50, GAITTIME/4, 1);
    usleep(1000*GAITTIME/4);
    Leg1.set_position(GAIT_X, 95 + 65, -150, GAITTIME/4, 1);
    Leg3.set_position(GAIT_X, 95 - 65, -150, GAITTIME/4, 1);
    usleep(1000*GAITTIME/4);
    Leg1.set_position(GAIT_X, 95 - 65, -150, GAITTIME/2, 0);
    Leg3.set_position(GAIT_X, 95 + 65, -150, GAITTIME/2, 0);
}


void C_GaitControl::round_left(float _r)
{
    float R = abs(_r);
    if(R < 200) R=0;//if R is very small, R=0
    float L = 2*(GAIT_YHEAD - GAIT_YTAIL) + 53;
    float W = 2*GAIT_X + 53;

    float l3 = pow(pow(R, 2) + pow(L/2, 2), 0.5);
    float sita3 = atan(L/2/R);

    float R1 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(sita3)*W/2*l3), 0.5);
    float R2 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(3.14159265 - sita3)*W/2*l3), 0.5);


    float value = (pow(R1, 2) + pow(W, 2) - pow(R2, 2))/(2*R1*W);
    if(value > 1)
    {
        printf("warning! acos1() input value > 1\n");
    }
    float sita1 = 3.14159265 - acos(value);

    value = (pow(R2, 2) + pow(W, 2) - pow(R1, 2))/(2*R2*W);
    if(value > 1)
    {
        printf("warning! acos2() input value > 1\n");
    }
    float sita2 = acos(value);

    float kspeed = R1/R2;

    if( _r<0 )
    {
        float temp = sita2;
        sita2 = -sita1;
        sita1 = -temp;

        temp = R1;
        R1 = R2;
        R2 = temp;
    }

    //printf("R1=%.2f R2=%.2f sita1=%.2f sita2=%.2f kspeed=%.4f \n", R1, R2, sita1*180/3.1415, sita2*180/3.1415, kspeed);

    if(_r>0)//left and forward
    {
        Leg3.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 - 65*cos(sita1)*kspeed, GAIT_X - 65*sin(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 + 65*cos(sita1)*kspeed, GAIT_X + 65*sin(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 + 65*cos(sita1)*kspeed, GAIT_X + 65*sin(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 - 65*cos(sita1)*kspeed, GAIT_X - 65*sin(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 - 65*cos(sita2), GAIT_X + 65*sin(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 + 65*cos(sita2), GAIT_X - 65*sin(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg1.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 + 65*cos(sita2), GAIT_X - 65*sin(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 - 65*cos(sita2), GAIT_X + 65*sin(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);
        printf("left and forward\n");
    }
    else if(_r<=0)//right and forward
    {
        Leg3.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 - 65*cos(sita1), GAIT_X - 65*sin(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 + 65*cos(sita1), GAIT_X + 65*sin(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 + 65*cos(sita1), GAIT_X + 65*sin(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 - 65*cos(sita1), GAIT_X - 65*sin(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 - 65*cos(sita2)*kspeed, GAIT_X + 65*sin(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 + 65*cos(sita2)*kspeed, GAIT_X - 65*sin(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg1.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 + 65*cos(sita2)*kspeed, GAIT_X - 65*sin(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 - 65*cos(sita2)*kspeed, GAIT_X + 65*sin(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);
        printf("right and forward\n");
    }
}


void C_GaitControl::round_right(float _r)
{
    float R = abs(_r);
    if(R < 200) R=0;//if R is very small, R=0
    float L = 2*(GAIT_YHEAD - GAIT_YTAIL) + 53;
    float W = 2*GAIT_X + 53;

    float l3 = pow(pow(R, 2) + pow(L/2, 2), 0.5);
    float sita3 = atan(L/2/R);

    float R1 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(sita3)*W/2*l3), 0.5);
    float R2 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(3.14159265 - sita3)*W/2*l3), 0.5);


    float value = (pow(R1, 2) + pow(W, 2) - pow(R2, 2))/(2*R1*W);
    if(value > 1)
    {
        printf("warning! acos1() input value > 1\n");
    }
    float sita1 = 3.14159265 - acos(value);

    value = (pow(R2, 2) + pow(W, 2) - pow(R1, 2))/(2*R2*W);
    if(value > 1)
    {
        printf("warning! acos2() input value > 1\n");
    }
    float sita2 = acos(value);

    float kspeed = R1/R2;

    if( _r<0 )
    {
        float temp = sita2;
        sita2 = -sita1;
        sita1 = -temp;

        temp = R1;
        R1 = R2;
        R2 = temp;
    }
    sita2 += 3.14159265;
    sita1 += 3.14159265;


    //printf("R1=%.2f R2=%.2f sita1=%.2f sita2=%.2f kspeed=%.4f \n", R1, R2, sita1*180/3.1415, sita2*180/3.1415, kspeed);

    if(_r>0)//left and forward
    {
        Leg1.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 + 65*cos(sita2), GAIT_X - 65*sin(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 - 65*cos(sita2), GAIT_X + 65*sin(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 - 65*cos(sita2), GAIT_X + 65*sin(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 + 65*cos(sita2), GAIT_X - 65*sin(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 + 65*cos(sita1)*kspeed, GAIT_X + 65*sin(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 - 65*cos(sita1)*kspeed, GAIT_X - 65*sin(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 - 65*cos(sita1)*kspeed, GAIT_X - 65*sin(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 + 65*cos(sita1)*kspeed, GAIT_X + 65*sin(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        printf("left and forward\n");
    }
    else if(_r<=0)//right and forward
    {
        Leg1.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 + 65*cos(sita2)*kspeed, GAIT_X - 65*sin(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(95 - 65*cos(sita2)*kspeed, GAIT_X + 65*sin(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 - 65*cos(sita2)*kspeed, GAIT_X + 65*sin(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(95 + 65*cos(sita2)*kspeed, GAIT_X - 65*sin(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 + 65*cos(sita1), GAIT_X + 65*sin(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(95 - 65*cos(sita1), GAIT_X - 65*sin(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(95, GAIT_X, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 - 65*cos(sita1), GAIT_X - 65*sin(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(95 + 65*cos(sita1), GAIT_X + 65*sin(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);



        printf("right and forward\n");
    }
}






void C_GaitControl::back(float _r)
{
    float R = abs(_r);
    if(R < 200) R=0;//if R is very small, R=0
    float L = 2*(GAIT_YHEAD - GAIT_YTAIL) + 53;
    float W = 2*GAIT_X + 53;

    float l3 = pow(pow(R, 2) + pow(L/2, 2), 0.5);
    float sita3 = atan(L/2/R);

    float R1 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(sita3)*W/2*l3), 0.5);
    float R2 = pow((pow(l3, 2) + pow(W/2, 2) - 2*cos(3.14159265 - sita3)*W/2*l3), 0.5);


    float value = (pow(R1, 2) + pow(W, 2) - pow(R2, 2))/(2*R1*W);
    if(value > 1)
    {
        printf("warning! acos1() input value > 1\n");
    }
    float sita1 = 3.14159265 - acos(value);

    value = (pow(R2, 2) + pow(W, 2) - pow(R1, 2))/(2*R2*W);
    if(value > 1)
    {
        printf("warning! acos2() input value > 1\n");
    }
    float sita2 = acos(value);

    float kspeed = R1/R2;

    if( _r<0 )
    {
        float temp = sita2;
        sita2 = -sita1;
        sita1 = -temp;

        temp = R1;
        R1 = R2;
        R2 = temp;
    }
    sita2 += 3.14159265;
    sita1 += 3.14159265;
    //printf("R1=%.2f R2=%.2f sita1=%.2f sita2=%.2f kspeed=%.4f \n", R1, R2, sita1*180/3.1415, sita2*180/3.1415, kspeed);

    if(gaitstatu != Back) to_back();
    gaitstatu = Back;

    if(_r>=200)//left and forward
    {
        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1)*kspeed, 95 + 65*cos(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1)*kspeed, 95 - 65*cos(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1)*kspeed, 95 - 65*cos(sita1)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1)*kspeed, 95 + 65*cos(sita1)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2), 95 + 65*cos(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2), 95 - 65*cos(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2), 95 - 65*cos(sita2), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2), 95 + 65*cos(sita2), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        printf("left and forward\n");
    }
    else if(_r<=-200)//right and forward
    {
        Leg1.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg1.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg2.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X - 65*sin(sita1), 95 - 65*cos(sita1), -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg2.set_position(GAIT_X + 65*sin(sita1), 95 + 65*cos(sita1), -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg4.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg4.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        Leg3.set_position(GAIT_X, 95, -50, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X + 65*sin(sita2)*kspeed, 95 - 65*cos(sita2)*kspeed, -150, GAITTIME/16, 1);
        usleep(1000*GAITTIME/16);
        Leg3.set_position(GAIT_X - 65*sin(sita2)*kspeed, 95 + 65*cos(sita2)*kspeed, -150, 7*GAITTIME/8, 0);
        usleep(1000*GAITTIME/8);

        printf("right and forward\n");
    }
}



void C_GaitControl::gait_stop()
{
    Leg1.set_position(GAIT_X, 67, 0, 3000);
    Leg2.set_position(GAIT_X, 160, 0, 3000);
    Leg3.set_position(GAIT_X, 86, 0, 3000);
    Leg4.set_position(GAIT_X, 141, 0, 3000);
    sleep(3);
}


void C_GaitControl::legsetposition(float _x, float _y, float _z)
{
    Leg1.set_position(_x, _y, _z, 500);
    Leg2.set_position(_x, _y, _z, 500);
    Leg3.set_position(_x, _y, _z, 500);
    Leg4.set_position(_x, _y, _z, 500);
}


void C_GaitControl::legsetangle(float _A, float _B, float _C)
{
    Leg1.set_angle(_A, _B, _C, 500);
    Leg2.set_angle(_A, _B, _C, 500);
    Leg3.set_angle(_A, _B, _C, 500);
    Leg4.set_angle(_A, _B, _C, 500);
}



//hcy
void C_GaitControl::get_sun(float pitch,float roll)
{
    float angle_dx, angle_dy;
    float dz_x, dz_y;
    float dz_x_e, dz_y_e;
    float z=5;


    angle_dx = pitch;// - 180;// - 180;
    angle_dy = roll;

    dz_x_e = sin(angle_dx*PI/180)*(27.5+74);
    dz_y_e = sin(angle_dy*PI/180)*(27.5+100);
    dz_x_i += dz_x_e;
    dz_y_i += dz_y_e;

    if(dz_x_i>150) dz_x_i = 150;
    if(dz_y_i>150) dz_y_i = 150;
    if(dz_x_i<-150) dz_x_i = -150;
    if(dz_y_i<-150) dz_y_i = -150;

    dz_x = 13*dz_x_e/10 + 3*dz_x_i/10;
   dz_y = 13*dz_y_e/10 + 3*dz_y_i/10;

    if(dz_x>100) dz_x=100;
    if(dz_y>100) dz_y=100;
    if(dz_x<-100) dz_x=-100;
    if(dz_y<-100) dz_y=-100;
/*
    Leg1.set_position(GAIT_X, 95, -dz_y, 200);//-dz_y
    Leg2.set_position(GAIT_X, 95, -100-dz_x+dz_y, 200);//+dz_y
    Leg3.set_position(GAIT_X, 95, -100-dz_x+dz_y, 200);//+dz_y
    Leg4.set_position(GAIT_X, 95, -dz_y, 200);//-dz_y
*/
    Leg1.set_position(GAIT_X, 95, -100+dz_x/2-dz_y/2, 200);//-dz_y
    Leg2.set_position(GAIT_X, 95, -100-dz_x/2-dz_y/2, 200);//+dz_y
    Leg3.set_position(GAIT_X, 95, -100-dz_x/2+dz_y/2, 200);//+dz_y
    Leg4.set_position(GAIT_X, 95, -100+dz_x/2+dz_y/2, 200);//-dz_y

    usleep(200000);

}











}
