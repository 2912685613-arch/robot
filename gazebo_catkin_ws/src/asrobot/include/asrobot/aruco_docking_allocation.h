#ifndef ASROBOT_ARUCO_DOCKING_ALLOCATION_H
#define ASROBOT_ARUCO_DOCKING_ALLOCATION_H
#include <eigen3/Eigen/Dense>
#include <array>
#include <cmath>
#include <algorithm>

namespace aruco_docking {
// new_gazebo_model + joint_command_calibration.yaml，H=(40,-40,40,-40) deg。
// 数据由实际URDF变换得出，test_docking_geometry.py 独立核对，模型修改后必须重算。
inline Eigen::Matrix<double,3,4> directions() {
    Eigen::Matrix<double,3,4> a;
    a << -.996194923017,.087152711770,.996194957118,-.087152711770,
         -.087153171799,-.996194963258,.087152782008,.996194963258,
         .043241718853,-.043225359824,.043195906957,-.043211496805;
    // 第三行是 r×direction 的 Z 力臂系数，单位 m，并非方向 Z。
    return a;
}
struct Allocation {
    bool valid = false;
    std::array<double,4> speed{{0,0,0,0}}, tilt{{0,0,0,0}};
    double horizontal_scale = 1, vertical_scale = 1;
    Eigen::Vector3d achieved = Eigen::Vector3d::Zero(); // Fx,Fy,Mz
};
inline Allocation allocate(double fx,double fy,double fz,double mz) {
    Allocation result;
    if(!std::isfinite(fx)||!std::isfinite(fy)||!std::isfinite(fz)||!std::isfinite(mz))return result;
    const auto a=directions();
    Eigen::Matrix4d m;
    m.topRows<3>()=a;
    m.row(3).setOnes();
    const Eigen::Vector4d neutral=m.fullPivLu().solve(Eigen::Vector4d(0,0,0,3.1));
    const Eigen::Vector4d delta=m.fullPivLu().solve(Eigen::Vector4d(fx,fy,mz,0));
    // 保持 Fx/Fy/Mz 的相对比例，饱和时统一缩放增量，避免逐个截断导致反向力矩。
    double scale=1;
    for(int n=0;n<4;++n) {
        if(delta[n]>0)scale=std::min(scale,(2.5-neutral[n])/delta[n]);
        if(delta[n]<0)scale=std::min(scale,(0.10-neutral[n])/delta[n]);
    }
    scale=std::max(0.,std::min(1.,scale));
    const Eigen::Vector4d h=neutral+scale*delta;
    double vertical_scale=1;
    for(int n=0;n<4;++n) {
        const double max_vertical=std::min(h[n]*std::tan(M_PI/6),std::sqrt(std::max(0.,3.1*3.1-h[n]*h[n])));
        if(std::fabs(fz/4)>max_vertical)vertical_scale=std::min(vertical_scale,max_vertical/std::fabs(fz/4));
    }
    for(int n=0;n<4;++n) {
        const double v=fz/4*vertical_scale;
        const double force=std::hypot(h[n],v);
        result.tilt[n]=std::atan2(v,h[n])*180/M_PI;
        // 实际UUV Basic模型 F=k*abs(omega)*omega，避免旧线性映射与真实推力不一致。
        result.speed[n]=std::min(100.,std::sqrt(force/.00031));
    }
    result.achieved=a*h;result.horizontal_scale=scale;result.vertical_scale=vertical_scale;
    result.valid=true;return result;
}
} // namespace aruco_docking
#endif
