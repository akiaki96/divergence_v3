#include "common/wall_edge.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include "config/mouse_config.hpp"

namespace {
using namespace config::wall_edge;

constexpr int16_t THRESH_ON[WallEdge::SIDE_COUNT] = {THRESH_ON_LEFT, THRESH_ON_RIGHT};
constexpr int16_t THRESH_OFF[WallEdge::SIDE_COUNT] = {THRESH_OFF_LEFT, THRESH_OFF_RIGHT};
constexpr float OFFSET[WallEdge::SIDE_COUNT] = {OFFSET_LEFT_MM, OFFSET_RIGHT_MM};
static_assert(THRESH_OFF_LEFT < THRESH_ON_LEFT && THRESH_OFF_RIGHT < THRESH_ON_RIGHT,
              "config::wall_edge: THRESH_OFF_* must be below THRESH_ON_* (hysteresis)");
} // namespace

void WallEdge::reset() {
    active_ = false;
    correct_ = false;
    window_ = WINDOW_MM;
    window_late_ = WINDOW_MM;
    queue_.clear();
    pending_count_ = 0;
    for (auto& s : side_) s = SideState{};
    event_count_ = 0;
    total_shift_ = 0.f;
}

void WallEdge::start(bool correct, float window_mm, float window_late_mm) {
    correct_ = correct;
    window_ = window_mm;
    window_late_ = window_late_mm;
    for (auto& s : side_) s = SideState{};
    std::atomic_signal_fence(std::memory_order_seq_cst);   // 上の書き込みを ISR が active_ より先に見るように
    active_ = true;
}

void WallEdge::start(bool correct, float window_mm) {
    start(correct, window_mm, window_mm);
}

void WallEdge::start(bool correct) {
    start(correct, WINDOW_MM, WINDOW_MM);
}

bool WallEdge::expect(float boundary_mm) {
    return queue_.push(boundary_mm);
}

float WallEdge::expectedX(Side side, float boundary, float v) {
    return boundary + OFFSET[side] + LAG_S * v;
}

void WallEdge::drainQueue() {
    float b;
    while (queue_.pop(b)) {
        if (pending_count_ == MAX_PENDING) {   // 一杯なら最も古い境界を捨てる
            std::copy(pending_ + 1, pending_ + MAX_PENDING, pending_);
            --pending_count_;
        }
        pending_[pending_count_++] = b;
    }
}

// 予想位置を窓の幅より過ぎた境界はもう来ない
void WallEdge::expire(float x, float v) {
    std::size_t keep = 0;
    for (std::size_t i = 0; i < pending_count_; ++i) {
        float latest = std::max(expectedX(left, pending_[i], v), expectedX(right, pending_[i], v));
        if (x <= latest + window_late_) pending_[keep++] = pending_[i];
    }
    pending_count_ = keep;
}

float WallEdge::onFallingEdge(Side side, float x_edge, float v) {
    // 予想位置が最も近い境界（窓の中。予想より後ろ（x_edge が先）は window_late_ まで，前は window_ まで）
    std::size_t best = pending_count_;
    float best_error = INFINITY;
    for (std::size_t i = 0; i < pending_count_; ++i) {
        float late = x_edge - expectedX(side, pending_[i], v);
        if (late > window_late_ || -late > window_) continue;
        float error = std::fabs(late);
        if (error <= best_error) {
            best = i;
            best_error = error;
        }
    }

    Event e{x_edge, NAN, 0.f, v, static_cast<uint8_t>(side)};
    if (best < pending_count_) {
        e.boundary = pending_[best];
        if (correct_) e.shift = expectedX(side, e.boundary, v) - x_edge;
        // 使った境界（とそれより手前の境界）は消す：1つの境界で2回補正しない（左右が同じ境界で切れても1回）
        std::copy(pending_ + best + 1, pending_ + pending_count_, pending_);
        pending_count_ -= best + 1;
    }
    record(e);
    return e.shift;
}

void WallEdge::record(const Event& e) {
    if (event_count_ < MAX_EVENTS) {
        events_[event_count_] = e;
        event_count_ = event_count_ + 1;
    }
}

float WallEdge::update(int16_t value_left, int16_t value_right, float x, float v_target, float omega_target) {
    if (!active_) return 0.f;
    drainQueue();

    // 直進中だけ見る。旋回・停止の間は状態を捨て，直進に戻ったら見え始めから数え直す
    if (omega_target != 0.f || v_target < MIN_VELOCITY) {
        for (auto& s : side_) s.valid = false;
        return 0.f;
    }
    expire(x, v_target);

    const int16_t values[SIDE_COUNT] = {value_left, value_right};
    float shift = 0.f;
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        SideState& s = side_[i];
        int16_t value = values[i];
        if (!s.valid) {
            s.valid = true;
            s.on = value > THRESH_ON[i];
            s.on_since = x;
        } else if (!s.on && value > THRESH_ON[i]) {
            s.on = true;
            s.on_since = x;
        } else if (s.on && value < THRESH_OFF[i]) {
            s.on = false;
            // OFF を下回った位置を前のtickとの間で補間する
            float drop = static_cast<float>(s.prev_value - value);
            float frac = (drop > 0.f) ? static_cast<float>(s.prev_value - THRESH_OFF[i]) / drop : 1.f;
            float x_edge = s.prev_x + std::clamp(frac, 0.f, 1.f) * (x - s.prev_x);
            if (x_edge - s.on_since >= MIN_WALL_MM && shift == 0.f) {
                shift = onFallingEdge(static_cast<Side>(i), x_edge, v_target);
            }
        }
        s.prev_value = value;
        s.prev_x = x;
    }
    if (shift != 0.f) {
        // 次のtickの補間も補正後の座標で行う
        for (auto& s : side_) {
            s.prev_x += shift;
            s.on_since += shift;
        }
        total_shift_ = total_shift_ + shift;
    }
    return shift;
}
