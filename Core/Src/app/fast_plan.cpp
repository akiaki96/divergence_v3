#include "app/fast_plan.hpp"

#include <algorithm>
#include <cmath>
#include "common/wall_edge.hpp"

namespace fast_plan {

namespace {

using config::maze::CELL_MM;
constexpr float HALF_MM = CELL_MM / 2.f;
constexpr float DIAG_STEP_MM = HALF_MM * 1.41421356f;   // 斜めの1歩（辺の中点から隣の中点まで）

// これより短い加速・減速の区間は作らない（PlanProfile は1tickに満たない区間を tooShort で弾く）
constexpr float MIN_SEGMENT_MM = 5.f;

// ACT_S* / ACT_V90_* → プリセットのターンと向き。ターンでなければ false
bool turnOf(uint8_t a, const RunPreset& p, const slalom::Param** turn, slalom::TurnDir* dir) {
    const DiagonalTurns* d = p.diagonal;
    using slalom::TurnDir;
    switch (a) {
    case ACT_TURN_LEFT_MOVE:  *turn = p.turns.s90; *dir = TurnDir::left;  return true;
    case ACT_TURN_RIGHT_MOVE: *turn = p.turns.s90; *dir = TurnDir::right; return true;
    case ACT_S90_LEFT:       *turn = p.turns.l90;  *dir = TurnDir::left;  return true;
    case ACT_S90_RIGHT:      *turn = p.turns.l90;  *dir = TurnDir::right; return true;
    case ACT_S180_LEFT:      *turn = p.turns.t180; *dir = TurnDir::left;  return true;
    case ACT_S180_RIGHT:     *turn = p.turns.t180; *dir = TurnDir::right; return true;
    case ACT_S45_in_LEFT:    *turn = d ? d->in45 : nullptr;   *dir = TurnDir::left;  return true;
    case ACT_S45_in_RIGHT:   *turn = d ? d->in45 : nullptr;   *dir = TurnDir::right; return true;
    case ACT_S45_out_LEFT:   *turn = d ? d->out45 : nullptr;  *dir = TurnDir::left;  return true;
    case ACT_S45_out_RIGHT:  *turn = d ? d->out45 : nullptr;  *dir = TurnDir::right; return true;
    case ACT_V90_LEFT:       *turn = d ? d->v90 : nullptr;    *dir = TurnDir::left;  return true;
    case ACT_V90_RIGHT:      *turn = d ? d->v90 : nullptr;    *dir = TurnDir::right; return true;
    case ACT_S135_in_LEFT:   *turn = d ? d->in135 : nullptr;  *dir = TurnDir::left;  return true;
    case ACT_S135_in_RIGHT:  *turn = d ? d->in135 : nullptr;  *dir = TurnDir::right; return true;
    case ACT_S135_out_LEFT:  *turn = d ? d->out135 : nullptr; *dir = TurnDir::left;  return true;
    case ACT_S135_out_RIGHT: *turn = d ? d->out135 : nullptr; *dir = TurnDir::right; return true;
    default:                 return false;
    }
}

// 直線を足す（直前も同じ種類の直線ならつなぐ）
Error addStraight(Steps* out, bool diagonal, float distance) {
    if (distance <= 0.f) return Error::none;
    if (!out->empty() && out->back().turn == nullptr && out->back().diagonal == diagonal) {
        out->back().distance += distance;
        return Error::none;
    }
    if (out->full()) return Error::tooManySteps;
    out->push_back(Step{nullptr, slalom::TurnDir::left, diagonal, distance});
    return Error::none;
}

// 手順 i の直線の始めと終わりの速度
float startSpeed(std::size_t i, const RunPreset& p) {
    return (i == 0) ? 0.f : p.turn_speed;
}
float endSpeed(const Steps& steps, std::size_t i, const RunPreset& p) {
    return (i + 1 == steps.size()) ? 0.f : p.turn_speed;
}

} // namespace

const char* errorName(Error e) {
    switch (e) {
    case Error::none:          return "none";
    case Error::noMotion:      return "no motion";
    case Error::unknownAction: return "unknown action";
    case Error::turnMissing:   return "turn not in preset";
    case Error::tooManySteps:  return "too many steps";
    }
    return "unknown";
}

Error build(const uint8_vector& actions, const RunPreset& p, float start_offset, Steps* out) {
    out->clear();
    for (uint8_t a : actions) {
        if (a == ACT_FINISH) break;
        Error e = Error::none;
        const slalom::Param* turn = nullptr;
        slalom::TurnDir dir = slalom::TurnDir::left;
        if (ACT_MOVE_0SEC <= a && a <= ACT_MOVE_32SEC) {
            e = addStraight(out, false, (a - ACT_MOVE_0SEC) * HALF_MM);
        } else if (ACT_MOVE_0SEC_DIA <= a && a <= ACT_MOVE_32SEC_DIA) {
            e = addStraight(out, true, (a - ACT_MOVE_0SEC_DIA) * DIAG_STEP_MM);
        } else if (turnOf(a, p, &turn, &dir)) {
            if (turn == nullptr) return Error::turnMissing;
            if (out->full()) return Error::tooManySteps;
            out->push_back(Step{turn, dir, false, 0.f});
        } else {
            return Error::unknownAction;
        }
        if (e != Error::none) return e;
    }
    if (out->empty()) return Error::noMotion;

    // 始め: 置いた位置から (0,0) の中央まで。最初が縦の直線ならそれに足し、ターンならその前に直線を入れる
    if (start_offset > 0.f) {
        if (out->front().turn == nullptr && !out->front().diagonal) {
            out->front().distance += start_offset;
        } else {
            if (out->full()) return Error::tooManySteps;
            out->insert(out->begin(), Step{nullptr, slalom::TurnDir::left, false, start_offset});
        }
    }

    // 終わり: 経路はゴール区画の中央で終わる。直線ならそこで止まり（台形の終速0）、
    // ターンならターンの速度のまま中央に着くので、止まれる距離だけ進んで止まる
    if (out->back().turn != nullptr) {
        float v = p.turn_speed;
        Error e = addStraight(out, false, v * v / (2.f * p.decel));
        if (e != Error::none) return e;
    }
    return Error::none;
}

uint8_t segments(const Steps& steps, std::size_t i, const RunPreset& p, Segment out[MAX_SEGMENTS_PER_STEP]) {
    const Step& s = steps[i];
    if (s.turn != nullptr) {
        out[0] = Segment{s.turn, s.dir, s.turn->speed, 0.f};
        return 1;
    }

    const float d = s.distance;
    const float v_in = startSpeed(i, p);
    const float v_out = endSpeed(steps, i, p);
    const float v_max = s.diagonal ? p.max_speed_dia : p.max_speed;
    const float a = p.accel;
    const float b = p.decel;

    // 加速度 a で上げて減速度 b で下げ、ちょうど d 進む頂点の速度（三角）
    float v_peak = std::sqrt((2.f * a * b * d + b * v_in * v_in + a * v_out * v_out) / (a + b));
    float cruise = 0.f;
    if (v_peak > v_max) {
        v_peak = v_max;
        cruise = d - (v_max * v_max - v_in * v_in) / (2.f * a) - (v_max * v_max - v_out * v_out) / (2.f * b);
    }
    // 始め・終わりの速度に届かない（d が短い）ときや、区間がごく短くなるときは、1区間で v_in → v_out
    float d_acc = (v_peak * v_peak - v_in * v_in) / (2.f * a);
    float d_dec = (v_peak * v_peak - v_out * v_out) / (2.f * b);
    bool tiny_acc = d_acc < MIN_SEGMENT_MM && v_peak - v_in > 1.f;
    bool tiny_dec = d_dec < MIN_SEGMENT_MM && v_peak - v_out > 1.f;
    if (v_peak < v_in || v_peak < v_out || tiny_acc || tiny_dec) {
        out[0] = Segment{nullptr, slalom::TurnDir::left, v_out, d};
        return 1;
    }

    uint8_t n = 0;
    if (d_acc >= MIN_SEGMENT_MM) out[n++] = Segment{nullptr, slalom::TurnDir::left, v_peak, d_acc};
    else cruise += d_acc;   // 速度がほとんど変わらない加速は等速に含める
    float d_dec_used = (d_dec >= MIN_SEGMENT_MM) ? d_dec : 0.f;
    if (d_dec_used == 0.f) cruise += d_dec;
    if (cruise >= MIN_SEGMENT_MM) {
        out[n++] = Segment{nullptr, slalom::TurnDir::left, v_peak, cruise};
    } else if (cruise > 0.f) {
        // ごく短い等速は減速(なければ加速)の区間に含める
        if (d_dec_used > 0.f) d_dec_used += cruise;
        else if (n > 0) out[n - 1].distance += cruise;
    }
    if (d_dec_used > 0.f) out[n++] = Segment{nullptr, slalom::TurnDir::left, v_out, d_dec_used};
    if (n == 0) out[n++] = Segment{nullptr, slalom::TurnDir::left, v_out, d};
    // 等速だけ（v_in = v_peak = v_out）のときなどで終速を合わせる
    out[n - 1].v_end = v_out;
    return n;
}

SegmentResult validate(const Steps& steps, const RunPreset& p, std::size_t* bad_step) {
    using namespace config::profile_limit;
    float v = 0.f;
    Segment seg[MAX_SEGMENTS_PER_STEP];
    for (std::size_t i = 0; i < steps.size(); i++) {
        uint8_t n = segments(steps, i, p, seg);
        for (uint8_t k = 0; k < n; k++) {
            SegmentResult r;
            if (seg[k].turn != nullptr) {
                // スラロームは並進の速度を保つので、入る速度がターンの速度でなければならない
                r = (std::fabs(v - seg[k].turn->speed) > 1e-3f) ? SegmentResult::directionMismatch
                                                                 : slalom::validate(*seg[k].turn, seg[k].dir);
            } else {
                r = validateSegment(v, seg[k].v_end, seg[k].distance, MAX_ACCEL_X, MAX_DECEL_X);
                v = seg[k].v_end;
            }
            if (r != SegmentResult::ok) {
                *bad_step = i;
                return r;
            }
        }
    }
    if (v != 0.f) {   // 最後は止まる
        *bad_step = steps.size();
        return SegmentResult::directionMismatch;
    }
    return SegmentResult::ok;
}

float estimatedTime(const Steps& steps, const RunPreset& p) {
    float t = 0.f;
    float v = 0.f;
    Segment seg[MAX_SEGMENTS_PER_STEP];
    for (std::size_t i = 0; i < steps.size(); i++) {
        uint8_t n = segments(steps, i, p, seg);
        for (uint8_t k = 0; k < n; k++) {
            if (seg[k].turn != nullptr) {
                t += slalom::totalDistance(*seg[k].turn, seg[k].dir) / seg[k].turn->speed;
            } else {
                float mean = 0.5f * (v + seg[k].v_end);
                if (mean > 0.f) t += seg[k].distance / mean;
                v = seg[k].v_end;
            }
        }
    }
    return t;
}

std::size_t edgeBoundaries(const Steps& steps, const RunPreset& p, int per_turn, float* out, std::size_t max) {
    using namespace config::wall_edge;
    std::size_t n = 0;
    float x = 0.f;   // 手順 i の始めの位置
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const Step& s = steps[i];
        if (s.turn == nullptr) {
            x += s.distance;
            continue;
        }
        // 入口が区画中央（大回り90°・180°・入45°・入135°）で，直前が縦横の直線のターンだけ。
        // 斜めから入るターン（出45°・出135°・V90）の前は斜めの直線で，横壁が区画境界で切れない。
        // 直線は [x − 長さ, x]，x がターンの入口（区画中央）
        const Step* prev = (i > 0) ? &steps[i - 1] : nullptr;
        bool from_center = (s.turn->entry == slalom::Anchor::center);
        if (from_center && prev != nullptr && prev->turn == nullptr && !prev->diagonal) {
            const float straight_start = x - prev->distance;
            // 入口に近い境界から per_turn 個まで数えてから，位置の小さい順に入れる
            int k = 0;
            while (k < per_turn) {
                float b = x - HALF_MM - static_cast<float>(k) * CELL_MM;
                float earliest = std::min(WallEdge::expectedX(WallEdge::left, b, p.turn_speed),
                                          WallEdge::expectedX(WallEdge::right, b, p.turn_speed)) - WINDOW_MM;
                // 直線に入ってから MIN_WALL_MM 以上壁を見ていないと壁切れにならない
                if (earliest - MIN_WALL_MM < straight_start) break;
                ++k;
            }
            for (int j = k - 1; j >= 0 && n < max; --j) {
                out[n++] = x - HALF_MM - static_cast<float>(j) * CELL_MM;
            }
        }
        x += slalom::totalDistance(*s.turn, s.dir);
    }
    return n;
}

} // namespace fast_plan
