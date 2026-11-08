#ifndef C_FUZZY_PID_H
#define C_FUZZY_PID_H

#include "c_controller_group.h"

namespace ControllerGroup {

// Fuzzy gain scheduling for position error (metres), using the existing PID.
class FuzzyPID {
public:
    FuzzyPID();
    // Four ascending error breakpoints and a positive per-update change scale.
    // The default is tuned for per-axis position errors in metres.
    bool setErrorBands(float zero, float small, float medium, float large, float change);
    float calculate(float error, float kp, float ki, float kd);
    bool setIntegralLimit(float limit);
    // |误差| 超过阈值时暂停积分累加；0 表示关闭积分分离。
    bool setIntegralSeparation(float threshold);
    // 航向用 360 度周期微分；0 表示普通位置/速度误差。
    bool setErrorPeriod(float period);
    void resetErrorHistory();
    void reset();

    float getCurrentKp() { return pid_.getKp(); }
    float getCurrentKi() { return pid_.getKi(); }
    float getCurrentKd() { return pid_.getKd(); }
    float getLastErrorChange() const { return last_error_change_; }
    float getIntegralSum() const { return pid_.getIntegralSum(); }
    float getIntegralTerm() const { return pid_.getIntegralTerm(); }
    float getIntegralLimit() const { return pid_.i_limit; }
    float getIntegralSeparation() const { return integral_separation_; }
    bool isIntegralAccumulating() const { return integral_accumulating_; }

private:
    PID pid_;
    float previous_error_;
    float last_error_change_;
    bool has_previous_error_;
    float error_centers_[4];
    float change_scale_;
    float integral_separation_;
    bool integral_accumulating_;
    float error_period_ = 0.0f;
};

} // namespace ControllerGroup

#endif
