#ifndef C_GAITCONTROL_H
#define C_GAITCONTROL_H

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>

#include "c_legmodel.h"

// 各种宏定义 如步态周期 运行速度等
#define GAITTIME 5000
#define SPEED 1400

/*********gait*************/
#define GAIT_YHEAD   160
#define GAIT_YTAIL   30
#define GAIT_X       134
#define GAIT_Z      -150

using namespace std;
using namespace LegModel;

namespace GaitControl{

// 行走状态枚举 左右前后停
enum GaitStatu
{
    Stop, Left, Right, Forward, Back
};

extern GaitStatu gaitstatu;

extern leg Leg1;
extern leg Leg2;
extern leg Leg3;
extern leg Leg4;


class C_GaitControl
{
public: float dz_x_i, dz_y_i;//hcy

public:
    C_GaitControl();
    void to_forward();
    void to_back();
    void to_left(float _sita1, float _sita2);
    void to_right(float _sita1, float _sita2);
    void forward4(float _r);
    void right();    // at original
    void left();
    void forward2();
    void round_left(float _r);
    void round_right(float _r);
    void back(float _r);
    void gait_stop();
    void get_sun(float pitch,float roll);//hcy
    // 正解与反解
    void legsetposition(float _x, float _y, float _z);
    void legsetangle(float _A, float _B, float _C);

};

}

























#endif // C_GAITCONTROL_H
