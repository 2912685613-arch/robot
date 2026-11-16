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
    bool sim_time = false;
    double now = 0, wall = 0, vision_stamp = 0, feedback_stamp = 0;
    double x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0, yaw = 0, yaw_rate = 0;
};
struct Config {
    double target_z = -0.15, timeout = 0.5, wait_timeout = 3.0;
    double recovery_timeout = 1.5;
    double transport_timeout = 2.0;
    int reacquire_frames = 3;
    double max_speed = 0.02, near_speed = 0.005, near_distance = 0.10, acceleration = 0.02;
    double position_kp = 0.3, velocity_kp = 16, velocity_ki = 0.5, z_force_limit = 0.6;
    double xy_kp = 0.6, xy_kd = 0.8, xy_force_limit = 0.20;
    double yaw_kp = 0.003, yaw_kd = 0.04, yaw_limit = 0.03;
    double xy_tolerance = 0.03, xy_hysteresis = 0.01, z_tolerance = 0.01;
    double velocity_tolerance = 0.005, yaw_tolerance = 3, settle_time = 1;
    bool valid() const {
        const double values[] = {timeout,wait_timeout,recovery_timeout,transport_timeout,max_speed,near_speed,near_distance,acceleration,
            position_kp,velocity_kp,velocity_ki,z_force_limit,xy_kp,xy_kd,xy_force_limit,
            yaw_kp,yaw_kd,yaw_limit,xy_tolerance,xy_hysteresis,z_tolerance,
            velocity_tolerance,yaw_tolerance,settle_time};
        if (!std::isfinite(target_z) || target_z >= -0.14) return false;
        for (double v : values) if (!std::isfinite(v) || v <= 0) return false;
        return near_speed <= max_speed && timeout <= 0.5 && wait_timeout > 1.0 && timeout <= wait_timeout &&
               recovery_timeout >= timeout && transport_timeout >= recovery_timeout && reacquire_frames >= 1 && reacquire_frames <= 30;
    }
};
enum class State { Idle, Wait, Align, Approach, Hold, Recover, Transport, Lost };
inline const char* name(State state) {
    switch(state) {
        case State::Idle:return "IDLE"; case State::Wait:return "WAIT_VISION";
        case State::Align:return "XY_ALIGN"; case State::Approach:return "APPROACH";
        case State::Hold:return "HOLD"; case State::Recover:return "REACQUIRE_BRAKE"; case State::Transport:return "WAIT_TRANSPORT"; default:return "LOST_STOP";
    }
}
struct Output {
    State state = State::Idle;
    bool active = false;
    double fx = 0, fy = 0, fz = 0, mz = 0;
    double target_vz = 0, z_error = 0, yaw_error = 0, yaw_target = 0;
    double xy_threshold = 0;
    const char* reason = "none";
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
        recovery_start = recovery_vision = 0; recovery_frames = 0; failure_reason = "none";
        transport_wall = transport_ros = 0;
    }
    State state() const { return state_; }
    static bool feedbackFresh(const Input& i, double timeout) {
        const double numbers[] = {i.now,i.wall,i.feedback_stamp,i.vx,i.vy,i.vz,i.yaw,i.yaw_rate};
        for(double v:numbers) if(!std::isfinite(v)) return false;
        return i.feedback_valid && i.feedback_stamp > 0 &&
               i.now-i.feedback_stamp >= 0 && i.now-i.feedback_stamp <= timeout;
    }
    static bool visionFresh(const Input& i, double timeout) {
        const double numbers[] = {i.vision_stamp,i.x,i.y,i.z};
        for(double v:numbers) if(!std::isfinite(v)) return false;
        return i.vision_valid && i.vision_stamp > 0 &&
               i.now-i.vision_stamp >= 0 && i.now-i.vision_stamp <= timeout && i.z < -0.12;
    }
    static bool fresh(const Input& i, double timeout) {
        return feedbackFresh(i,timeout) && visionFresh(i,timeout);
    }
    Output lost(const char* reason = "transport_timeout") {
        state_ = State::Lost; speed = integral = 0; settle_start = -1; failure_reason = reason;
        Output o; o.state = state_; o.reason = failure_reason; return o;
    }
    // 0.5s通信间隔先停电机，不立即锁死；真正超时用独立阈值。
    // 仅仿真时钟停顿可延后锁死；电机始终关闭，不把旧数据延寿。
    Output transportGap(double wall, double now, bool sim_time) {
        Output o; o.state = state_; o.reason = failure_reason;
        if(state_ == State::Idle || state_ == State::Lost) return o;
        if(!std::isfinite(wall) || !std::isfinite(now)) return lost("invalid_transport_clock");
        if(state_ != State::Transport) {
            state_ = State::Transport; transport_wall = wall; transport_ros = now;
            recovery_frames=0; recovery_vision=0; speed=0; settle_start=-1;
        }
        if(now < transport_ros || wall < transport_wall) return lost("transport_clock_reversed");
        // ROS时间继续前进：最多2秒后锁死。暂停时物理未推进，保持零输出等待。
        if((now-transport_ros >= cfg.transport_timeout) ||
           (!sim_time && wall-transport_wall >= cfg.transport_timeout)) return lost("mc_transport_timeout");
        o.state=state_;o.active=false;o.reason="mc_gap_outputs_zero";return o;
    }

    Output update(const Input& i) {
        Output o; o.state = state_; o.reason = failure_reason;
        if(state_ == State::Idle || state_ == State::Lost) return o;
        // 处理恢复的mc前先执行相同的通信间隔逻辑；防止绕过看门狗直接恢复旧输出。
        if(initialized && i.wall < last_wall) return lost("steady_clock_reversed");
        if(initialized && i.wall-last_wall > cfg.timeout && state_ != State::Transport) {
            auto paused = transportGap(i.wall,i.now,i.sim_time);
            if(paused.state==State::Lost) return paused;
        }
        if(state_ == State::Transport) {
            auto paused = transportGap(i.wall,i.now,i.sim_time);
            if(paused.state==State::Lost) return paused;
            const bool new_sources = fresh(i,cfg.timeout) &&
                i.vision_stamp>transport_ros && i.feedback_stamp>transport_ros;
            if(new_sources) {
                if(i.vision_stamp>recovery_vision) {++recovery_frames;recovery_vision=i.vision_stamp;}
            } else {recovery_frames=0;recovery_vision=0;}
            if(recovery_frames<cfg.reacquire_frames) return paused;
            // 只允许停顿之后产生的新图像和反馈，连续确认后从零速重新对齐。
            state_=State::Align;align_z=i.z;speed=0;settle_start=-1;
            last_wall=i.wall;last_feedback=i.feedback_stamp;last_vision=i.vision_stamp;
        }
        const bool body_ok = feedbackFresh(i,cfg.timeout);
        const bool vision_ok = body_ok && visionFresh(i,cfg.timeout);
        if(!initialized) {
            if(!body_ok || !vision_ok) {
                if(i.wall-start_wall > cfg.wait_timeout) return lost("initial_feedback_timeout");
                o.reason = body_ok ? "waiting_vision" : "waiting_body_feedback";
                return o; // 首条无效/零值只能等待，不误判到位。
            }
        } else {
            if(!body_ok) return lost("body_feedback_invalid_or_stale");
            if(i.feedback_stamp < last_feedback) return lost("body_clock_reversed");
            if(vision_ok && i.vision_stamp < last_vision) return lost("image_clock_reversed");
            if(!vision_ok && state_ != State::Recover) {
                state_ = State::Recover; recovery_start = i.wall;
                recovery_frames = 0; recovery_vision = 0; speed = 0; settle_start = -1;
            }
            if(state_ == State::Recover) {
                if(i.wall-recovery_start >= cfg.recovery_timeout) return lost("vision_recovery_timeout");
                if(vision_ok) {
                    if(i.vision_stamp > recovery_vision) { ++recovery_frames; recovery_vision = i.vision_stamp; }
                } else { recovery_frames = 0; recovery_vision = 0; }
                if(recovery_frames >= cfg.reacquire_frames) {
                    state_ = State::Align; align_z = i.z; speed = 0; settle_start = -1;
                } else {
                    // 仅实时速度/航向制动：不使用旧视觉XY/Z，不搜索、不继续向下。
                    const double dt = limit(i.feedback_stamp-last_feedback,0.001,0.25);
                    o = velocityOutput(i,dt,0);
                    o.state = State::Recover; o.reason = "vision_invalid_or_confirming";
                    last_wall=i.wall;last_feedback=i.feedback_stamp;
                    if(vision_ok)last_vision=i.vision_stamp;
                    return o;
                }
            }
        }
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
        o = velocityOutput(i,dt,speed);
        o.xy_threshold = enter_xy;
        o.z_error = i.z-target_z;
        o.fx = limit(cfg.xy_kp*i.x-cfg.xy_kd*i.vx,-cfg.xy_force_limit,cfg.xy_force_limit);
        o.fy = limit(cfg.xy_kp*i.y-cfg.xy_kd*i.vy,-cfg.xy_force_limit,cfg.xy_force_limit);
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
    Output velocityOutput(const Input& i, double dt, double target_vz) {
        Output o; o.active = true; o.target_vz = target_vz;
        const double ve = target_vz-i.vz;
        const double candidate = limit(integral+cfg.velocity_ki*ve*dt,-cfg.z_force_limit,cfg.z_force_limit);
        const double raw = cfg.velocity_kp*ve+candidate;
        if(std::fabs(raw)<=cfg.z_force_limit || raw*ve<0) integral=candidate;
        o.fz = limit(cfg.velocity_kp*ve+integral,-cfg.z_force_limit,cfg.z_force_limit);
        o.fx = limit(-cfg.xy_kd*i.vx,-cfg.xy_force_limit,cfg.xy_force_limit);
        o.fy = limit(-cfg.xy_kd*i.vy,-cfg.xy_force_limit,cfg.xy_force_limit);
        o.yaw_target = heading;
        o.yaw_error = std::remainder(heading-i.yaw,360.0);
        o.mz = limit(cfg.yaw_kp*o.yaw_error-cfg.yaw_kd*i.yaw_rate,-cfg.yaw_limit,cfg.yaw_limit);
        return o;
    }
    Config cfg;
    State state_ = State::Idle;
    bool initialized = false, heading_captured = false;
    double start_wall=0,last_wall=0,last_feedback=0,last_vision=0;
    double heading=0,align_z=0,speed=0,integral=0,settle_start=-1;
    double recovery_start=0,recovery_vision=0;
    double transport_wall=0,transport_ros=0;
    int recovery_frames=0;
    const char* failure_reason="none";
};
} // namespace aruco_docking
#endif
