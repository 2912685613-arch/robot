#include "asrobot/aruco_docking_control.h"
#include "asrobot/aruco_docking_allocation.h"
#include "asrobot/aruco_target_selector.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace aruco_docking;

Input measurement(double t=10) {
    Input i;i.now=i.wall=i.vision_stamp=i.feedback_stamp=t;
    i.vision_valid=i.feedback_valid=true;i.z=-.9;return i;
}
void advance(Input& i,double dt=.1) { i.now+=dt;i.wall+=dt;i.vision_stamp=i.feedback_stamp=i.now; }
int main(int argc,char** argv) {
    std::string test=argc>1?argv[1]:"";
    Config cfg;
    if(test=="validity") {
        Controller c;c.start(10,0,false);Input i=measurement();i.vision_valid=false;i.z=0;
        auto o=c.update(i);assert(!o.active && o.state==State::Wait);
        advance(i);i=measurement(i.now);o=c.update(i);assert(o.active);
        advance(i);i.vision_valid=false;assert(c.update(i).state==State::Recover);
        for(int n=0;n<20;++n){advance(i);c.update(i);}
        assert(c.state()==State::Lost);
        advance(i);i.vision_valid=true;assert(!c.update(i).active); // 超时后不自行重启
        c.start(i.wall,0,true);assert(c.update(i).active);
        advance(i);i.vision_stamp=i.now-1;assert(c.update(i).state==State::Recover);
    } else if(test=="numeric") {
        for(int n=0;n<5;++n) {
            Controller c;c.start(10,0,false);auto i=measurement();
            if(n==0)i.z=0;if(n==1)i.x=std::numeric_limits<double>::quiet_NaN();
            if(n==2)i.vision_stamp=i.now+1;if(n==3)i.feedback_valid=false;
            if(n==4)i.yaw=std::numeric_limits<double>::infinity();
            assert(!c.update(i).active);
        }
        Controller c;c.start(10,0,false);auto i=measurement();i.vision_valid=false;
        i.wall=14;assert(c.update(i).state==State::Lost);
        assert(cfg.valid());cfg.near_speed=.3;assert(!cfg.valid());
    } else if(test=="speed") {
        Controller c;c.start(10,0,true);auto i=measurement();double last=0;
        for(int n=0;n<60;++n) {
            auto o=c.update(i);assert(o.active);assert(o.target_vz>=-cfg.max_speed-1e-9);
            if(n>0) assert(std::fabs(o.target_vz-last)<=cfg.acceleration*.1+1e-9);
            assert(std::fabs(o.fz)<=cfg.z_force_limit);last=o.target_vz;advance(i);
        }
        i.z=-.19;
        for(int n=0;n<20;++n){c.update(i);advance(i);}
        auto o=c.update(i);assert(std::fabs(o.target_vz)<=cfg.near_speed+1e-9);
        i.z=-.14;advance(i);
        for(int n=0;n<20;++n){o=c.update(i);advance(i);}
        assert(o.target_vz>0); // 距离过近，允许向上修正
        i.vz=-.05;advance(i);assert(c.update(i).fz>0); // 向下过快，速度内环制动
    } else if(test=="hold") {
        Controller c;c.start(10,20,true);auto i=measurement();i.z=-.15;i.yaw=20;
        Output o;
        for(int n=0;n<14;++n){o=c.update(i);advance(i);}
        assert(o.state==State::Hold && o.active);
        i.z=-.17;advance(i);o=c.update(i);assert(o.active && o.target_vz<0);
        i.z=-.14;advance(i);
        for(int n=0;n<20;++n){o=c.update(i);advance(i);}
        assert(o.active && o.target_vz>0); // HOLD主动反馈，不是关电机后的漂浮
        i.vision_valid=false;assert(c.update(i).state==State::Recover);
    } else if(test=="align") {
        Controller c;c.start(10,0,true);auto i=measurement();i.x=.10;
        auto o=c.update(i);assert(o.state==State::Align && o.z_error==0);
        i.z=-.89;advance(i);o=c.update(i);assert(o.target_vz>0); // 对齐时保持捕获高度
        i.x=0;advance(i);assert(c.update(i).state==State::Approach);
        i.x=.045;advance(i);assert(c.update(i).state==State::Align);
        i.z=-.16;i.x=.006;advance(i);o=c.update(i);
        assert(o.xy_threshold<.006); // 近处缩小XY阈值，不盲目继续下潜
    } else if(test=="heading") {
        Controller c;c.start(10,179,true);auto i=measurement();i.yaw=-179;
        auto o=c.update(i);assert(std::fabs(o.yaw_error+2)<1e-9);assert(o.mz<0);
        i.yaw=170;advance(i);o=c.update(i);assert(o.mz>0);
        i.yaw=179;i.yaw_rate=.1;advance(i);assert(c.update(i).mz<0);
        assert(std::fabs(c.update(i).mz)<=cfg.yaw_limit);
        Controller late;late.start(i.wall,0,false);i.yaw=37;
        assert(late.update(i).yaw_target==37); // 无反馈的命令时刻，首帧捕获航向
    } else if(test=="clocks") {
        Controller c;c.start(10,0,true);auto i=measurement();c.update(i);
        i.wall+=1;i.sim_time=true;assert(c.update(i).state==State::Transport); // /clock暂停先零输出等待
        c.start(11,0,true);i=measurement(11);c.update(i);
        i=measurement(9);assert(c.update(i).state==State::Lost); // 时钟回退
    } else if(test=="cached") {
        Controller c;c.start(10,0,true);auto i=measurement();i.z=-.15;c.update(i);
        for(int n=0;n<3;++n){i.now+=.1;i.wall+=.1;i.feedback_stamp=i.now;assert(c.update(i).state!=State::Hold);}
        i.now+=.3;i.wall+=.3;i.feedback_stamp=i.now;assert(c.update(i).state==State::Recover);
    } else if(test=="allocation") {
        for(auto v:{-.03,0.,.03}) {
            auto a=allocate(.1,-.05,-.3,v);assert(a.valid);
            assert((a.achieved-Eigen::Vector3d(.1,-.05,v)).norm()<1e-8);
            for(int n=0;n<4;++n){assert(a.speed[n]>=0&&a.speed[n]<=100);assert(std::fabs(a.tilt[n])<=30.00001);}
        }
        auto saturated=allocate(20,20,-10,10);assert(saturated.valid && saturated.horizontal_scale<1);
        assert((saturated.achieved-Eigen::Vector3d(20,20,10)*saturated.horizontal_scale).norm()<1e-8);
        assert(!allocate(NAN,0,0,0).valid);
    } else if(test=="plant") {
        // 简化质量/阻尼模型仅验证反馈符号和稳态能力，不替代Gazebo动力学。
        Controller c;c.start(10,0,true);auto i=measurement();
        double world_z=-.1,world_x=-.08,world_y=.03,yaw_rate=0;
        i.yaw=8;bool held=false;
        for(int n=0;n<2500;++n) {
            i.z=-1-world_z;i.x=-world_x;i.y=-world_y;i.yaw_rate=yaw_rate;
            auto o=c.update(i);assert(o.active);
            i.vz+=(o.fz+.02-1.0*i.vz)/3.2*.1;
            i.vx+=(o.fx-1.0*i.vx)/3.2*.1;
            i.vy+=(o.fy-1.0*i.vy)/3.2*.1;
            yaw_rate+=(o.mz-.04*yaw_rate)/.03*.1;
            world_z+=i.vz*.1;world_x+=i.vx*.1;world_y+=i.vy*.1;
            i.yaw+=yaw_rate*.1*180/M_PI;
            held=held||o.state==State::Hold;
            advance(i);
        }
        assert(held);assert(std::fabs((-1-world_z)-cfg.target_z)<.01);
        assert(std::fabs(i.yaw)<cfg.yaw_tolerance);
        assert(std::hypot(world_x,world_y)<.004);
    } else if(test=="recovery") {
        Controller c;c.start(10,0,true);auto i=measurement();c.update(i);
        advance(i);i.vision_valid=false;i.z=0;i.x=100;i.y=-100;
        i.vz=-.02;i.yaw=5;i.yaw_rate=.05;
        auto o=c.update(i);assert(o.active && o.state==State::Recover);
        assert(o.target_vz==0 && o.fz>0 && o.mz<0);
        assert(o.fx==0 && o.fy==0); // 不追失效位置，只用实时速度/航向制动
        advance(i);i.vision_valid=true;i.x=i.y=0;i.z=-.3;
        o=c.update(i);assert(o.state==State::Recover && o.target_vz==0);
        i.now+=.1;i.wall+=.1;i.feedback_stamp=i.now; // 同一张图不能重复确认
        o=c.update(i);assert(o.state==State::Recover);
        advance(i);o=c.update(i);assert(o.state==State::Recover);
        advance(i);o=c.update(i);assert(o.active && o.state==State::Approach);
        assert(o.target_vz>=-cfg.acceleration*.1-1e-9); // 重获从零速平滑恢复
        advance(i);i.vision_valid=false;c.update(i);
        advance(i);i.feedback_valid=false;o=c.update(i);
        assert(!o.active && o.state==State::Lost);
        assert(std::string(o.reason)=="body_feedback_invalid_or_stale");
    } else if(test=="selector") {
        TargetSelector selector;
        for(int n=0;n<10;++n)assert(!selector.selectInner(true,.49,true,.5,13));
        for(int n=0;n<2;++n)assert(!selector.selectInner(true,.25,true,.25,25));
        assert(selector.selectInner(true,.25,true,.25,25));
        assert(!selector.selectInner(true,.25,false,-1,0)); // 大码仍在，直接回退
        assert(!selector.selectInner(true,.25,true,.25,25));
        assert(!selector.selectInner(false,-1,false,-1,0)); // 空帧打断确认
        for(int n=0;n<2;++n)assert(!selector.selectInner(false,-1,true,.1,60));
        assert(selector.selectInner(false,-1,true,.1,60)); // 外码已出画面也可选小码
        assert(selector.selectInner(false,-1,true,.03,200));
    } else if(test=="latency_trace") {
        // 重放日志中正常约0.33s的图像延迟与一次切码短缺，不应永久退出。
        Controller c;c.start(10,0,true);auto i=measurement();i.vision_stamp=i.now-.33;
        c.update(i);
        for(int n=0;n<15;++n) {
            advance(i,.2);i.vision_stamp=i.now-.33;
            if(n==5 || n==6){i.vision_valid=false;i.z=0;}
            else {i.vision_valid=true;i.z=-.6;}
            auto o=c.update(i);assert(o.state!=State::Lost);
            if(n==5||n==6)assert(o.target_vz==0 && o.state==State::Recover);
        }
        assert(c.state()==State::Approach);
    } else if(test=="transport_stall") {
        Controller c;c.start(10,0,true);auto i=measurement();i.sim_time=true;
        c.update(i);
        // 本次真实日志：ROS=30.573不动，墙钟停顿2秒，先关电机而非永久退出。
        auto o=c.transportGap(10.6,10.268,true);
        assert(!o.active && o.state==State::Transport);
        o=c.transportGap(12.6,10.268,true);
        assert(!o.active && o.state==State::Transport);
        // 恢复后先收到旧缓存，不应立即启动；只有停顿后产生的新图像可以确认。
        i.wall=12.7;i.now=10.48;i.feedback_stamp=10.48;i.vision_stamp=10.1;
        o=c.update(i);assert(!o.active && o.state==State::Transport);
        for(int n=0;n<3;++n){advance(i,.2);o=c.update(i);if(n<2)assert(!o.active);}
        assert(o.active && o.state==State::Approach);
        assert(std::fabs(o.target_vz)<=cfg.acceleration*.001+1e-9);
        // 时间仍前进但MC不来，超过独立阈值后必须锁存故障。
        c.transportGap(14,11.5,true);
        o=c.transportGap(17,14,true);
        assert(!o.active && o.state==State::Lost && std::string(o.reason)=="mc_transport_timeout");
        // 实机/非仿真不允许凭冻结的ROS时间绕过单调时钟超时。
        Controller real;real.start(10,0,true);real.update(measurement());
        real.transportGap(10.6,10,false);
        assert(real.transportGap(13,10,false).state==State::Lost);
    } else if(test=="transport_recovery_bad_feedback") {
        Controller c;c.start(10,0,true);auto i=measurement();c.update(i);
        c.transportGap(10.6,10.6,false);
        i=measurement(10.8);i.feedback_valid=false;
        assert(!c.update(i).active);
        i=measurement(11);i.vision_valid=false;
        assert(!c.update(i).active);
        // 新MC仍给无效反馈，不可在WAIT_TRANSPORT里无限续期。
        i=measurement(12.7);i.feedback_valid=false;
        auto o=c.update(i);assert(o.state==State::Lost);
    } else if(test=="print-allocation") {
        auto a=allocate(std::atof(argv[2]),std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]));
        for(int n=0;n<4;++n) std::cout<<a.speed[n]<<" "<<a.tilt[n]<<" ";
        std::cout<<a.horizontal_scale<<" "<<a.vertical_scale<<"\n";
    } else return 2;
}
