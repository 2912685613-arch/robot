#include "asrobot/c_fuzzy_pid.h"

#include <algorithm>
#include <cmath>

namespace ControllerGroup {
namespace {

struct Gains { float p, i, d; };

// Rows: zero, small, medium, large error. Columns: closing, steady, growing.
// Multipliers apply to the operator's base gains, preserving zero Ki.
const Gains rules[4][3] = {
    {{0.80f, 1.20f, 0.85f}, {0.85f, 1.20f, 0.85f}, {0.95f, 1.10f, 1.00f}},
    {{0.85f, 1.10f, 1.10f}, {1.00f, 1.00f, 1.00f}, {1.15f, 0.85f, 1.15f}},
    {{0.90f, 0.80f, 1.15f}, {1.05f, 0.75f, 1.05f}, {1.20f, 0.65f, 1.20f}},
    {{0.90f, 0.60f, 1.20f}, {1.10f, 0.60f, 1.10f}, {1.25f, 0.50f, 1.25f}}
};

// Adjacent memberships overlap and sum to one, avoiding gain jumps at bands.
void memberships(float value, const float* centers, int count, float* weights)
{
    for (int i = 0; i < count; ++i) weights[i] = 0.0f;
    if (value <= centers[0]) { weights[0] = 1.0f; return; }
    for (int i = 1; i < count; ++i) {
        if (value <= centers[i]) {
            const float blend = (value - centers[i - 1]) / (centers[i] - centers[i - 1]);
            weights[i - 1] = 1.0f - blend;
            weights[i] = blend;
            return;
        }
    }
    weights[count - 1] = 1.0f;
}

} // namespace

FuzzyPID::FuzzyPID() : previous_error_(0.0f), has_previous_error_(false),
    error_centers_{0.1f, 0.8f, 5.0f, 10.0f}, change_scale_(0.1f) {}

bool FuzzyPID::setErrorBands(float zero, float small, float medium, float large, float change)
{
    if (!(zero > 0.0f && zero < small && small < medium && medium < large && change > 0.0f))
        return false;
    error_centers_[0] = zero;
    error_centers_[1] = small;
    error_centers_[2] = medium;
    error_centers_[3] = large;
    change_scale_ = change;
    reset();
    return true;
}

float FuzzyPID::calculate(float error, float kp, float ki, float kd)
{
    // <0.1 near zero, 0.1..0.8 small, 0.8..5 medium, 5..10 large.
    // At >10 the large-error rule saturates. Units are metres per axis.
    // ec is change in absolute error per control update, not a derivative.
    const float change_centers[] = {-change_scale_, 0.0f, change_scale_};
    float e_weights[4], ec_weights[3];
    memberships(std::fabs(error), error_centers_, 4, e_weights);
    const float change = has_previous_error_ ?
        std::fabs(error) - std::fabs(previous_error_) : 0.0f;
    memberships(change, change_centers, 3, ec_weights);

    Gains multiplier = {0.0f, 0.0f, 0.0f};
    for (int e = 0; e < 4; ++e) {
        for (int ec = 0; ec < 3; ++ec) {
            const float weight = e_weights[e] * ec_weights[ec];
            multiplier.p += weight * rules[e][ec].p;
            multiplier.i += weight * rules[e][ec].i;
            multiplier.d += weight * rules[e][ec].d;
        }
    }

    pid_.setKpPara(std::max(0.0f, kp) * multiplier.p);
    pid_.setKiPara(std::max(0.0f, ki) * multiplier.i);
    pid_.setKdPara(std::max(0.0f, kd) * multiplier.d);
    previous_error_ = error;
    has_previous_error_ = true;
    return pid_.pidNewCalc(error);
}

void FuzzyPID::reset()
{
    pid_.pidReset();
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

} // namespace ControllerGroup
