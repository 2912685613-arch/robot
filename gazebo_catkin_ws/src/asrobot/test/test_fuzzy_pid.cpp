#include "asrobot/c_fuzzy_pid.h"

#include <cassert>
#include <cmath>
#include <limits>

using ControllerGroup::FuzzyPID;

bool close(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main()
{
    FuzzyPID pid;
    const float kp = 1.0f, ki = 0.2f, kd = 0.1f;

    pid.calculate(0.05f, kp, ki, kd);
    assert(close(pid.getLastErrorChange(), 0.0f));
    assert(close(pid.getCurrentKp(), 0.85f));
    assert(close(pid.getCurrentKi(), 0.2f));

    pid.reset();
    pid.calculate(0.3f, kp, ki, kd);
    assert(close(pid.getCurrentKp(), 1.05f));
    pid.calculate(0.8f, kp, ki, kd);
    pid.calculate(1.0f, kp, ki, kd);
    assert(close(pid.getLastErrorChange(), 0.2f));
    const float growing_kp = pid.getCurrentKp();
    pid.calculate(0.8f, kp, ki, kd);
    assert(close(pid.getLastErrorChange(), -0.2f));
    assert(pid.getCurrentKp() < growing_kp);

    pid.reset();
    assert(close(pid.getLastErrorChange(), 0.0f));
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

    // 暂停积分应保持 I 输出，同时继续更新 P、D 和两拍平滑。
    ControllerGroup::PID gated(2.0f, 0.5f, 1.0f);
    assert(close(gated.pidNewCalc(0.2f), 0.35f));
    assert(close(gated.pidNewCalcWithIntegral(0.4f, false), 0.9f));
    assert(close(gated.getIntegralSum(), 0.2f));
    assert(close(gated.getIntegralTerm(), 0.1f));
    assert(close(gated.pidNewCalc(0.1f), 0.575f));

    // 重现上浮：先积累负补偿，再以大正误差运动，积分不能被积成正值。
    FuzzyPID depth;
    assert(depth.setIntegralLimit(5.0f));
    assert(depth.setIntegralSeparation(0.1f));
    for (int i = 0; i < 120; ++i) depth.calculate(-0.05f, 1.6f, 0.02f, 0.1f);
    assert(close(depth.getIntegralSum(), -5.0f));
    depth.resetErrorHistory();
    for (int i = 0; i < 40; ++i) {
        depth.calculate(0.5f, 1.6f, 0.02f, 0.1f);
        assert(!depth.isIntegralAccumulating());
        assert(close(depth.getIntegralSum(), -5.0f));
        assert(close(depth.getIntegralTerm(), -0.1f));
    }

    // 阈值内及边界恢复累加，两种误差方向均适用。
    depth.calculate(0.1f, 1.6f, 0.02f, 0.1f);
    assert(depth.isIntegralAccumulating());
    assert(close(depth.getIntegralSum(), -4.9f));
    depth.calculate(-0.1f, 1.6f, 0.02f, 0.1f);
    assert(close(depth.getIntegralSum(), -5.0f));
    depth.calculate(-0.1001f, 1.6f, 0.02f, 0.1f);
    assert(!depth.isIntegralAccumulating());
    assert(close(depth.getIntegralSum(), -5.0f));

    // 无效配置不能覆盖原阈值，关闭后恢复原来的全程积分。
    assert(!depth.setIntegralSeparation(-0.1f));
    assert(!depth.setIntegralSeparation(std::numeric_limits<float>::infinity()));
    assert(!depth.setIntegralSeparation(std::numeric_limits<float>::quiet_NaN()));
    assert(close(depth.getIntegralSeparation(), 0.1f));
    assert(depth.setIntegralSeparation(0.0f));
    depth.calculate(0.5f, 1.6f, 0.02f, 0.1f);
    assert(depth.isIntegralAccumulating());
    assert(close(depth.getIntegralSum(), -4.5f));

    // 全量重置清空积分状态，但保留积分分离阈值和积分限幅。
    assert(depth.setIntegralSeparation(0.1f));
    depth.reset();
    assert(close(depth.getIntegralSum(), 0.0f));
    assert(close(depth.getIntegralTerm(), 0.0f));
    assert(close(depth.getIntegralSeparation(), 0.1f));
    assert(close(depth.getIntegralLimit(), 5.0f));
    depth.calculate(-0.5f, 1.6f, 0.02f, 0.1f);
    assert(!depth.isIntegralAccumulating());
    assert(close(depth.getIntegralSum(), 0.0f));

    FuzzyPID default_pid;
    default_pid.calculate(0.5f, 1.6f, 0.02f, 0.1f);
    assert(default_pid.isIntegralAccumulating());
    assert(close(default_pid.getIntegralSum(), 0.5f));

    // 周期微分：179 -> -179 是 +2 度，反向是 -2 度，而不是 ±358 度。
    ControllerGroup::PID angular(0.0f, 0.0f, 1.0f);
    assert(close(angular.pidNewCalcWithIntegral(179.0f, true, 360.0f), 0.0f));
    assert(close(angular.pidNewCalcWithIntegral(-179.0f, true, 360.0f), 1.0f));
    assert(close(angular.pidNewCalcWithIntegral(179.0f, true, 360.0f), 0.0f));
    angular.pidReset();
    assert(close(angular.pidNewCalcWithIntegral(-179.0f, true, 360.0f), 0.0f));

    FuzzyPID fuzzy_angle;
    assert(fuzzy_angle.setErrorPeriod(360.0f));
    assert(!fuzzy_angle.setErrorPeriod(-1.0f));
    assert(!fuzzy_angle.setErrorPeriod(std::numeric_limits<float>::quiet_NaN()));
    assert(close(fuzzy_angle.calculate(179.0f, 0.0f, 0.0f, 1.0f), 0.0f));
    assert(std::fabs(fuzzy_angle.calculate(-179.0f, 0.0f, 0.0f, 1.0f)) < 2.0f);
    fuzzy_angle.resetErrorHistory();
    // 保留上一拍平滑值，但不能因新目标产生大微分。
    assert(std::fabs(fuzzy_angle.calculate(90.0f, 0.0f, 0.0f, 1.0f)) < 2.0f);
}
