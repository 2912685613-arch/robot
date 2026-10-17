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
    void reset();

    float getCurrentKp() { return pid_.getKp(); }
    float getCurrentKi() { return pid_.getKi(); }
    float getCurrentKd() { return pid_.getKd(); }

private:
    PID pid_;
    float previous_error_;
    bool has_previous_error_;
    float error_centers_[4];
    float change_scale_;
};

} // namespace ControllerGroup

#endif
