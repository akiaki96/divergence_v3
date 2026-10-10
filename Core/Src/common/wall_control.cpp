#include "common/wall_control.hpp"
#include <algorithm>
#include "common/wall_sensor.hpp"
#include "config/mouse_config.hpp"

void WallControl::reset() {
    enabled_ = false;
    omega_ = 0.f;
    offset_deg_ = 0.f;
    use_ranges_ = false;
    x0_ = nullptr;
    x1_ = nullptr;
    range_count_ = 0;
    range_index_ = 0;
}

void WallControl::setRanges(const float* x0, const float* x1, uint16_t n) {
    use_ranges_ = true;
    x0_ = x0;
    x1_ = x1;
    range_count_ = (x0 != nullptr && x1 != nullptr) ? n : 0;
    range_index_ = 0;
}

bool WallControl::inRange(float x) {
    if (!use_ranges_) return true;
    while (range_index_ < range_count_ && x >= x1_[range_index_]) ++range_index_;
    return range_index_ < range_count_ && x >= x0_[range_index_];
}

AxisReference WallControl::apply(const AxisReference& rot, float omega_target, float v_target, float x_target,
                                 bool stopping) {
    omega_ = 0.f;
    if (enabled_ && omega_target == 0.f && v_target > config::wall::MIN_VELOCITY && inRange(x_target)
        && (config::wall::CONTROL_WHILE_STOPPING || !stopping)) {
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
        // 左に寄っていれば右へ（角速度は左旋回が正）。ゲインと上限は速度に比例（距離での応答を速度によらずそろえる）
        float kp = config::wall::KP_PER_VELOCITY * v_target;
        float max_omega = config::wall::MAX_OMEGA_PER_VELOCITY * v_target;
        omega_ = std::clamp(-kp * error, -max_omega, max_omega);
        offset_deg_ += omega_ * config::control::DT_S;
    }
    return {rot.pos + offset_deg_, rot.vel + omega_, rot.acc};
}
