#include "asrobot/c_fuzzy_pid.h"

#include <algorithm>
#include <cmath>

namespace ControllerGroup {
namespace {

struct Gains { float p, i, d; };

// 行对应零、小、中、大误差；列对应误差缩小、不变、增大。
// 表中数值乘以基础 PID 参数，因此基础 Ki 为 0 时不会凭空产生积分作用。
const Gains rules[4][3] = {
    {{0.90f, 1.00f, 1.05f}, {0.85f, 1.00f, 1.15f}, {0.80f, 1.00f, 1.25f}},
    {{1.00f, 1.00f, 1.00f}, {1.05f, 1.00f, 1.05f}, {1.10f, 1.00f, 1.10f}},
    {{1.10f, 1.00f, 0.95f}, {1.15f, 1.00f, 1.00f}, {1.20f, 1.00f, 1.05f}},
    {{1.15f, 1.00f, 0.90f}, {1.20f, 1.00f, 0.95f}, {1.25f, 1.00f, 1.00f}}
};

// 计算输入值对各模糊集合的隶属度：在相邻中心点之间线性分配权重，
// 两个权重之和为 1；超出中心点范围时完全归入首档或末档。
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

} // 匿名命名空间

// 初始化位置误差分档（单位：米）和每次更新的误差变化尺度。
// 首次计算没有历史误差，不能将当前误差误判为从 0 突然变化。
FuzzyPID::FuzzyPID() : previous_error_(0.0f), last_error_change_(0.0f), has_previous_error_(false),
    error_centers_{0.05f, 0.3f, 1.0f, 2.0f}, change_scale_(0.04f),
    integral_separation_(0.0f), integral_accumulating_(false) {}

// 设置模糊集合的中心点及误差变化尺度，例如供速度环使用。
// 参数必须严格递增且为正；无效时返回 false 且保持原配置，
// 成功时更新配置并清除旧的 PID 与误差历史。
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

// 将误差绝对值和每次更新的误差变化模糊化，按隶属度加权规则表中的增益倍率。
// 倍率乘以传入的基础 Kp/Ki/Kd，再由现有 PID 计算控制输出；
// 保存本次误差，供下一次计算变化量使用。
float FuzzyPID::calculate(float error, float kp, float ki, float kd)
{
    // 默认位置误差 <0.05 视为零附近，0.05~0.3 为小，0.3~1 为中，1~2 为大；
    // 超过末档时规则使用末档，但传给 PID 的误差仍是实际值。
    // ec 是每次控制更新的误差绝对值变化量，不是按时间求得的导数。
    const float change_centers[] = {-change_scale_, 0.0f, change_scale_};
    float e_weights[4], ec_weights[3];
    memberships(std::fabs(error), error_centers_, 4, e_weights);
    const float change = has_previous_error_ ?
        std::fabs(error) - std::fabs(previous_error_) : 0.0f;
    last_error_change_ = change;
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
    // 大误差阶段仅保留原积分补偿；接近目标后再累加，避免上浮途中积累过多正积分。
    integral_accumulating_ = integral_separation_ == 0.0f || std::fabs(error) <= integral_separation_;
    return pid_.pidNewCalcWithIntegral(error, integral_accumulating_, error_period_);
}

// 设置积分累加量的限幅（PID 内部对误差累加和限幅，非对输出限幅）。
bool FuzzyPID::setIntegralLimit(float limit)
{
    if (!(limit > 0.0f)) return false;
    pid_.i_limit = limit;
    return true;
}

// 设置积分分离阈值，单位与控制误差一致；拒绝负值及非有限值。
// 配置变化不清空积分，0 可恢复原来的全程积分行为。
bool FuzzyPID::setIntegralSeparation(float threshold)
{
    if (!std::isfinite(threshold) || threshold < 0.0f) return false;
    integral_separation_ = threshold;
    return true;
}

// 只清除上一次误差，保留积分：切换目标点时避免误判误差突变，
// 同时让积分继续抵消浮力等恒定偏置。
void FuzzyPID::resetErrorHistory()
{
    pid_.resetDerivativeHistory();
    previous_error_ = 0.0f;
    last_error_change_ = 0.0f;
    has_previous_error_ = false;
}

// 周期只影响微分差值，保留现有积分和比例/平滑状态。
bool FuzzyPID::setErrorPeriod(float period)
{
    if (!std::isfinite(period) || period < 0.0f) return false;
    error_period_ = period;
    pid_.resetDerivativeHistory();
    return true;
}

// 清除 PID 的积分、平滑状态及上一次误差；保留已配置的分档和变化尺度。
void FuzzyPID::reset()
{
    pid_.pidReset();
    integral_accumulating_ = false;
    previous_error_ = 0.0f;
    last_error_change_ = 0.0f;
    has_previous_error_ = false;
}

} // namespace ControllerGroup
