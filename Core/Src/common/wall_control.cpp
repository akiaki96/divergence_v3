#include "common/wall_control.hpp"
#include <algorithm>
#include "common/wall_sensor.hpp"
#include "config/mouse_config.hpp"

void WallControl::reset() {
    enabled_ = false;
    omega_ = 0.f;
    offset_deg_ = 0.f;
}

AxisReference WallControl::apply(const AxisReference& rot, float omega_target, float v_target) {
    omega_ = 0.f;
    if (enabled_ && omega_target == 0.f && v_target > config::wall::MIN_VELOCITY) {
        wall::Snapshot s = wall::read();
        // 左に寄る（左の値が大きい）ほど正。両側に壁があれば差，片側なら2倍して同じ重みにする
        float error_left = static_cast<float>(s.value[wall::left] - config::wall::REF_LEFT);
        float error_right = static_cast<float>(s.value[wall::right] - config::wall::REF_RIGHT);
        float error = 0.f;
        if (wall::hasLeft(s) && wall::hasRight(s)) {
            error = error_left - error_right;
        } else if (wall::hasLeft(s)) {
            error = 2.f * error_left;
        } else if (wall::hasRight(s)) {
            error = -2.f * error_right;
        }
        // 左に寄っていれば右へ（角速度は左旋回が正）
        omega_ = std::clamp(-config::wall::KP * error, -config::wall::MAX_OMEGA, config::wall::MAX_OMEGA);
        offset_deg_ += omega_ * config::control::DT_S;
    }
    return {rot.pos + offset_deg_, rot.vel + omega_, rot.acc};
}
