#include "asrobot/c_fuzzy_pid.h"

#include <cassert>
#include <cmath>

using ControllerGroup::FuzzyPID;

bool close(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main()
{
    FuzzyPID pid;
    const float kp = 1.0f, ki = 0.2f, kd = 0.1f;

    pid.calculate(0.05f, kp, ki, kd);
    assert(close(pid.getCurrentKp(), 0.85f));
    assert(close(pid.getCurrentKi(), 0.24f));

    pid.reset();
    pid.calculate(0.8f, kp, ki, kd);
    assert(close(pid.getCurrentKp(), 1.0f));
    pid.calculate(1.0f, kp, ki, kd);
    const float growing_kp = pid.getCurrentKp();
    pid.calculate(0.8f, kp, ki, kd);
    assert(pid.getCurrentKp() < growing_kp);

    pid.reset();
    const float first = pid.calculate(2.0f, kp, 0.0f, kd);
    assert(close(pid.getCurrentKi(), 0.0f));
    pid.reset();
    pid.calculate(11.0f, kp, ki, kd);
    const float capped_kp = pid.getCurrentKp();
    pid.reset();
    pid.calculate(-20.0f, kp, ki, kd);
    assert(close(pid.getCurrentKp(), capped_kp));
    pid.reset();
    assert(close(first, pid.calculate(2.0f, kp, 0.0f, kd)));

    FuzzyPID velocity;
    assert(!velocity.setErrorBands(0.08f, 0.01f, 0.5f, 1.0f, 0.01f));
    assert(velocity.setErrorBands(0.01f, 0.08f, 0.5f, 1.0f, 0.01f));
    velocity.calculate(0.05f, kp, ki, kd);
    assert(velocity.getCurrentKp() > 0.85f && velocity.getCurrentKp() < 1.0f);
}
