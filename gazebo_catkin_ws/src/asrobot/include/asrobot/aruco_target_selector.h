#ifndef ASROBOT_ARUCO_TARGET_SELECTOR_H
#define ASROBOT_ARUCO_TARGET_SELECTOR_H
#include <cmath>
namespace aruco_docking {
// 只处理真实新图像；单帧看到很小的小码不立即切换。
struct TargetConfig {
    double to_inner = .30, to_outer = .70, min_inner_pixels = 24;
    int confirmation_frames = 3;
};
class TargetSelector {
public:
    explicit TargetSelector(TargetConfig c = TargetConfig()): cfg(c) {}
    bool selectInner(bool outer, double outer_z, bool inner, double inner_z, double min_side) {
        const bool stable_candidate = inner && std::isfinite(inner_z) && inner_z>0 &&
            min_side>=cfg.min_inner_pixels && inner_z<cfg.to_inner &&
            (!outer || (std::isfinite(outer_z) && outer_z>0 && outer_z<cfg.to_inner));
        if(!using_inner) {
            confirmed = stable_candidate ? confirmed+1 : 0;
            if(confirmed>=cfg.confirmation_frames) {using_inner=true;confirmed=0;}
        } else if(outer && (!inner || (std::isfinite(inner_z) && inner_z>cfg.to_outer))) {
            // 小码偶发丢失但大码仍完整可见：立即用大码的当前测量，不停发或等100帧。
            using_inner=false;confirmed=0;
        }
        return using_inner;
    }
private:
    TargetConfig cfg;
    bool using_inner=false;
    int confirmed=0;
};
} // namespace aruco_docking
#endif
