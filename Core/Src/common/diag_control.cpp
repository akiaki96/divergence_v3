#include "common/diag_control.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "config/diag_table.hpp"

namespace {
using namespace config::diag_control;
namespace table = config::diag_table;

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

const table::Entry* tableOf(DiagEdge::Side side) {
    return (side == DiagEdge::left) ? table::LEFT : table::RIGHT;
}
} // namespace

void DiagControl::reset() {
    enabled_ = false;
    correction_ = false;
    range_count_ = 0;
    range_index_ = 0;
    in_range_ = false;
    inject_deg_ = 0.f;
    bias_deg_ = 0.f;
    anchor_deg_ = 0.f;
    integral_deg_ = 0.f;
    lateral_ = 0.f;
    unmeasured_mm_ = 0.f;
    omega_ = 0.f;
    offset_deg_ = 0.f;
    measured_ = 0;
    active_ticks_ = 0;
    measured_ticks_ = 0;
}

uint8_t DiagControl::setRanges(const Range* ranges, uint8_t n) {
    uint8_t m = std::min(n, MAX_RANGES);
    for (uint8_t i = 0; i < m; ++i) ranges_[i] = ranges[i];
    range_count_ = m;
    range_index_ = 0;
    in_range_ = false;
    return m;
}

float DiagControl::sideOffset(DiagEdge::Side side, float since, int16_t value) {
    if (!(since == since)) return NaN;
    float f = (since - table::SINCE0_MM) / table::STEP_MM;
    if (f < 0.f) return NaN;
    auto i = static_cast<std::size_t>(f);
    if (i + 1 >= table::SIZE) return NaN;
    const table::Entry& a = tableOf(side)[i];
    const table::Entry& b = tableOf(side)[i + 1];
    if (a.sens <= 0.f || b.sens <= 0.f) return NaN;   // 「使わない」区間にかかる
    float t = f - static_cast<float>(i);
    float ref = a.ref + t * (b.ref - a.ref);
    float sens = a.sens + t * (b.sens - a.sens);
    return (static_cast<float>(value) - ref) / sens;
}

AxisReference DiagControl::apply(const AxisReference& rot, float omega_target, float v_target, float x,
                                 int16_t value_left, int16_t value_right, const DiagEdge& edge) {
    omega_ = 0.f;
    measured_ = 0;
    // 止めている間は何も足さない（ほかのモードが reset() を呼ばなくても，前の走行の補正が残らないように）
    if (!enabled_) return rot;
    if (range_count_ > 0) {
        // 過ぎた範囲を進める
        while (range_index_ < range_count_ && x >= ranges_[range_index_].x1) {
            ++range_index_;
            in_range_ = false;
            lateral_ = 0.f;   // 向きに足した分は offset_deg_ に残る
        }
        bool inside = range_index_ < range_count_ && x >= ranges_[range_index_].x0;
        if (inside && omega_target == 0.f && v_target > MIN_VELOCITY) {
            const Range& r = ranges_[range_index_];
            if (!in_range_) {   // 斜めの直線に入った：今の向きを基準に始め直す
                in_range_ = true;
                anchor_deg_ = offset_deg_;
                integral_deg_ = 0.f;
                lateral_ = 0.f;
                unmeasured_mm_ = 0.f;
                bias_deg_ += inject_deg_;
                inject_deg_ = 0.f;
            }
            ++active_ticks_;

            // 左へ寄れば左の値が増え，右へ寄れば右の値が増える。大きく離れて読めた側は壁の抜けとみなして使わない
            const int16_t values[DiagEdge::SIDE_COUNT] = {value_left, value_right};
            float y = 0.f;
            uint8_t n = 0;
            for (uint8_t i = 0; i < DiagEdge::SIDE_COUNT; ++i) {
                auto side = static_cast<DiagEdge::Side>(i);
                if (!(edge.lastEdge(side) >= r.x0 - EDGE_BEFORE_MM)) continue;   // この斜めの直線の切れ目がまだない
                float e = sideOffset(side, edge.since(side), values[i]);
                if (!(e >= -MAX_AWAY_MM)) continue;   // NaN（表の外）も
                measured_ |= static_cast<uint8_t>(1u << i);
                float toward = std::min(e, MAX_LATERAL_MM);
                y += (side == DiagEdge::left) ? toward : -toward;
                ++n;
            }
            if (n > 1) y /= static_cast<float>(n);

            float ds = v_target * config::control::DT_S;
            float k = std::min(1.f, ds / FILTER_MM);
            if (measured_ != 0) {
                ++measured_ticks_;
                unmeasured_mm_ = 0.f;
                lateral_ += k * (y - lateral_);
            } else {
                unmeasured_mm_ += ds;
                // 切れ目のまわりの短い区間は最後の値を保つ。長く読めなければ向きを入ったとき＋積分へ戻していく
                if (unmeasured_mm_ > HOLD_MM) lateral_ += k * (0.f - lateral_);
            }
            if (correction_) {
                if (measured_ != 0) {
                    integral_deg_ = std::clamp(integral_deg_ - KI_DEG_PER_MM2 * lateral_ * ds, -MAX_INTEGRAL_DEG,
                                               MAX_INTEGRAL_DEG);
                }
                // 左へ寄っていれば右へ向ける（角度は左旋回が正）。向きの目標へ，速度に比例した上限で近づける
                float target_deg = anchor_deg_ + integral_deg_ - KP_DEG_PER_MM * lateral_;
                float max_omega = MAX_OMEGA_PER_VELOCITY * v_target;
                omega_ = std::clamp((target_deg - offset_deg_) / config::control::DT_S, -max_omega, max_omega);
                offset_deg_ += omega_ * config::control::DT_S;
            }
        }
    }
    return {rot.pos + offset_deg_ + bias_deg_, rot.vel + omega_, rot.acc};
}
