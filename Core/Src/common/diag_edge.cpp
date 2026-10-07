#include "common/diag_edge.hpp"
#include <atomic>
#include <cmath>
#include "config/mouse_config.hpp"

namespace {
using namespace config::diag;

constexpr int16_t THRESH_ON[DiagEdge::SIDE_COUNT] = {THRESH_ON_LEFT, THRESH_ON_RIGHT};
constexpr int16_t THRESH_OFF[DiagEdge::SIDE_COUNT] = {THRESH_OFF_LEFT, THRESH_OFF_RIGHT};
static_assert(THRESH_OFF_LEFT < THRESH_ON_LEFT && THRESH_OFF_RIGHT < THRESH_ON_RIGHT,
              "config::diag: THRESH_OFF_* must be below THRESH_ON_* (hysteresis)");
} // namespace

void DiagEdge::reset() {
    active_ = false;
    x_ = 0.f;
    for (auto& s : side_) s = SideState{};
}

void DiagEdge::start() {
    for (auto& s : side_) {
        s.valid = false;   // 切れ目の記録は残し，ヒステリシスだけ最初の値から取り直す
        s.on = false;
    }
    std::atomic_signal_fence(std::memory_order_seq_cst);   // 上の書き込みを ISR が active_ より先に見るように
    active_ = true;
}

void DiagEdge::update(int16_t value_left, int16_t value_right, float x) {
    const int16_t values[SIDE_COUNT] = {value_left, value_right};
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        SideState& s = side_[i];
        int16_t v = values[i];
        if (!s.valid) {
            // 最初の値で状態を決める（壁の途中から始めても，その壁の切れ目を数えられるように）
            s.valid = true;
            s.on = v >= THRESH_ON[i];
            s.on_since = x;
        } else if (!s.on && v >= THRESH_ON[i]) {
            s.on = true;
            s.on_since = x;
        } else if (s.on && v < THRESH_OFF[i]) {
            s.on = false;
            // 前のtickと今のtickの間で OFF を横切った位置を補間する
            float t = (s.prev_value == v) ? 1.f
                                          : static_cast<float>(s.prev_value - THRESH_OFF[i]) / (s.prev_value - v);
            float x_edge = s.prev_x + t * (x - s.prev_x);
            if (x_edge - s.on_since >= MIN_WALL_MM) {
                s.edge_x = x_edge;
                ++s.count;
            }
        }
        s.prev_value = v;
        s.prev_x = x;
    }
    x_ = x;
}

float DiagEdge::since(Side side) const {
    return x_ - side_[side].edge_x;   // edge_x が NaN なら NaN
}
