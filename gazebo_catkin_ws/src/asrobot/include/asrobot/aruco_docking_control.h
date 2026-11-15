#ifndef ASROBOT_ARUCO_DOCKING_CONTROL_H
#define ASROBOT_ARUCO_DOCKING_CONTROL_H

#include <algorithm>
#include <cmath>
#include <string>

namespace aruco_docking {
inline double limit(double value, double lo, double hi) { return std::max(lo, std::min(hi, value)); }

// 所有速度为机体系 m/s，yaw 为世界系度数，yaw_rate 为机体系 rad/s。
// 本演示由 Gazebo ModelStates 提供独立速度/航向反馈；不把视觉位置差分当作速度真值。
struct Input {
    bool vision_valid = false, feedback_valid = false;
    double now = 0, wall = 0, vision_stamp = 0, feedback_stamp = 0;
    double x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0, yaw = 0, yaw_rate = 0;
};
struct Config {
    double target_z = -0.15, timeout = 0.5, wait_timeout = 3.0;
    double max_speed = 0.02, near_speed = 0.005, near_distance = 0.10, acceleration = 0.02;
    double position_kp = 0.3, velocity_kp = 16, velocity_ki = 0.5, z_force_limit = 0.6;
    double xy_kp = 0.6, xy_kd = 0.8, xy_force_limit = 0.20;
    double yaw_kp = 0.003, yaw_kd = 0.04, yaw_limit = 0.03;
    double xy_tolerance = 0.03, xy_hysteresis = 0.01, z_tolerance = 0.01;
    double velocity_tolerance = 0.005, yaw_tolerance = 3, settle_time = 1;
    bool valid() const {
        const double values[] = {timeout,wait_timeout,max_speed,near_speed,near_distance,acceleration,
            position_kp,velocity_kp,velocity_ki,z_force_limit,xy_kp,xy_kd,xy_force_limit,
            yaw_kp,yaw_kd,yaw_limit,xy_tolerance,xy_hysteresis,z_tolerance,
            velocity_tolerance,yaw_tolerance,settle_time};
        if (!std::isfinite(target_z) || target_z >= -0.14) return false;
        for (double v : values) if (!std::isfinite(v) || v <= 0) return false;
        return near_speed <= max_speed && timeout <= 0.5 && wait_timeout > 1.0 && timeout <= wait_timeout;
    }
};
enum class State { Idle, Wait, Align, Approach, Hold, Lost };
inline const char* name(State state) {
    switch(state) {
        case State::Idle:return "IDLE"; case State::Wait:return "WAIT_VISION";
        case State::Align:return "XY_ALIGN"; case State::Approach:return "APPROACH";
        case State::Hold:return "HOLD"; default:return "LOST_STOP";
    }
}
struct Output {
    State state = State::Idle;
    bool active = false;
    double fx = 0, fy = 0, fz = 0, mz = 0;
    double target_vz = 0, z_error = 0, yaw_error = 0, yaw_target = 0;
    double xy_threshold = 0;
};

class Controller {
public:
    explicit Controller(const Config& config = Config()) : cfg(config) {}
    void start(double wall, double yaw, bool yaw_valid) {
        stop(); state_ = State::Wait; start_wall = wall;
        heading = yaw; heading_captured = yaw_valid && std::isfinite(yaw);
    }
    void stop() {
        state_ = State::Idle; initialized = false; heading_captured = false;
        last_wall = last_feedback = last_vision = 0; speed = integral = 0; settle_start = -1;
    }
    State state() const { return state_; }
    static bool fresh(const Input& i, double timeout) {
        const double numbers[] = {i.now,i.wall,i.vision_stamp,i.feedback_stamp,i.x,i.y,i.z,
                                  i.vx,i.vy,i.vz,i.yaw,i.yaw_rate};
        for(double v:numbers) if(!std::isfinite(v)) return false;
        return i.vision_valid && i.feedback_valid && i.vision_stamp > 0 && i.feedback_stamp > 0 &&
            i.now-i.vision_stamp >= 0 && i.now-i.vision_stamp <= timeout &&
            i.now-i.feedback_stamp >= 0 && i.now-i.feedback_stamp <= timeout && i.z < -0.12;
    }
    Output lost() {
        state_ = State::Lost; speed = integral = 0; settle_start = -1;
        Output o; o.state = state_; return o;
    }
    Output update(const Input& i) {
        Output o; o.state = state_;
        if(state_ == State::Idle || state_ == State::Lost) return o;
        if(!fresh(i,cfg.timeout)) {
            if(initialized || i.wall-start_wall > cfg.wait_timeout) return lost();
            return o; // 首条普通 mc/全零视觉只等待，不误判“到位”，不自动退出 dock。
        }
        if(initialized && (i.feedback_stamp < last_feedback || i.vision_stamp < last_vision ||
                            i.wall-last_wall > cfg.timeout || i.wall < last_wall)) return lost();
        if(!heading_captured) { heading = i.yaw; heading_captured = true; }
        if(!initialized) {
            initialized = true; state_ = State::Align; align_z = i.z;
            last_wall = i.wall; last_feedback = i.feedback_stamp; last_vision = i.vision_stamp;
        }
        // 数值控制按反馈的ROS采样间隔更新；墙钟只用于通信/暂停看门狗。
        double dt = limit(i.feedback_stamp-last_feedback, 0.001, 0.25);
        // 近处相机视野缩小：不要用远处3cm阈值直接下降到相机3cm距离。
        const double camera_depth = std::max(0.0, -i.z-0.12);
        const double enter_xy = std::min(cfg.xy_tolerance, std::max(0.004, camera_depth*0.12));
        o.xy_threshold = enter_xy;
        const double xy = std::max(std::fabs(i.x),std::fabs(i.y));
        if(state_ == State::Approach && xy > enter_xy+cfg.xy_hysteresis) {
            state_ = State::Align; align_z = i.z; settle_start = -1;
        }
        if(state_ == State::Align && xy < enter_xy) state_ = State::Approach;
        // HOLD偏离允许带后回到接近/对齐；目标保持不变，避免在边界反复切换。
        if(state_ == State::Hold && (xy > enter_xy+cfg.xy_hysteresis ||
                                    std::fabs(i.z-cfg.target_z)>2*cfg.z_tolerance)) {
            state_ = State::Approach; settle_start=-1;
            if(xy > enter_xy+cfg.xy_hysteresis) {state_=State::Align;align_z=i.z;}
        }
        // HOLD 不会关闭电机或重置控制历史；偏离时继续保持同一15cm目标。
        const double target_z = state_ == State::Align ? align_z : cfg.target_z;
        o.z_error = i.z-target_z;
        double vmax = cfg.max_speed;
        if(std::fabs(o.z_error) < cfg.near_distance || state_ == State::Hold) vmax = cfg.near_speed;
        double desired_v = limit(cfg.position_kp*o.z_error, -vmax, vmax);
        if(state_ == State::Align) desired_v = limit(desired_v,-cfg.near_speed,cfg.near_speed);
        speed += limit(desired_v-speed, -cfg.acceleration*dt, cfg.acceleration*dt);
        const double ve = speed-i.vz;
        const double candidate = limit(integral+cfg.velocity_ki*ve*dt,-cfg.z_force_limit,cfg.z_force_limit);
        const double raw = cfg.velocity_kp*ve+candidate;
        if(std::fabs(raw)<=cfg.z_force_limit || raw*ve<0) integral=candidate;
        o.fz = limit(cfg.velocity_kp*ve+integral,-cfg.z_force_limit,cfg.z_force_limit);
        o.fx = limit(cfg.xy_kp*i.x-cfg.xy_kd*i.vx,-cfg.xy_force_limit,cfg.xy_force_limit);
        o.fy = limit(cfg.xy_kp*i.y-cfg.xy_kd*i.vy,-cfg.xy_force_limit,cfg.xy_force_limit);
        o.yaw_target = heading;
        o.yaw_error = std::remainder(heading-i.yaw,360.0);
        o.mz = limit(cfg.yaw_kp*o.yaw_error-cfg.yaw_kd*i.yaw_rate,-cfg.yaw_limit,cfg.yaw_limit);
        const bool settled = xy<enter_xy && std::fabs(i.z-cfg.target_z)<cfg.z_tolerance &&
            std::fabs(i.vz)<cfg.velocity_tolerance && std::hypot(i.vx,i.vy)<cfg.velocity_tolerance &&
            std::fabs(o.yaw_error)<cfg.yaw_tolerance;
        if(settled) {
            if(settle_start<0) settle_start=i.wall;
            // 仅新的图像能确认稳定，不能拿同一缓存帧确认 HOLD。
            if(i.wall-settle_start>=cfg.settle_time && i.vision_stamp>last_vision) state_=State::Hold;
        } else settle_start=-1;
        last_feedback=i.feedback_stamp;last_vision=i.vision_stamp;last_wall=i.wall;
        o.state=state_;o.active=true;o.target_vz=speed;return o;
    }
private:
    Config cfg;
    State state_ = State::Idle;
    bool initialized = false, heading_captured = false;
    double start_wall=0,last_wall=0,last_feedback=0,last_vision=0;
    double heading=0,align_z=0,speed=0,integral=0,settle_start=-1;
};
} // namespace aruco_docking
#endif
