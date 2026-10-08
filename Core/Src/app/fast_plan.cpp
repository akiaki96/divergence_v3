#include "app/fast_plan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "common/trapezoid.hpp"
#include "common/wall_edge.hpp"

namespace fast_plan {

namespace {

using config::maze::CELL_MM;
constexpr float HALF_MM = CELL_MM / 2.f;
constexpr float DIAG_STEP_MM = HALF_MM * 1.41421356f;   // 斜めの1歩（辺の中点から隣の中点まで）
static_assert(MAX_SEGMENTS_PER_STEP >= trapezoid::MAX_PARTS, "a straight step is split into trapezoid::MAX_PARTS");

// ACT_S* / ACT_V90_* / ACT_TURN_*_MOVE → ターンの種類と向き。ターンでなければ false
bool turnOf(uint8_t a, TurnKind* kind, slalom::TurnDir* dir) {
    using slalom::TurnDir;
    switch (a) {
    case ACT_TURN_LEFT_MOVE:  *kind = TURN_S90;    *dir = TurnDir::left;  return true;
    case ACT_TURN_RIGHT_MOVE: *kind = TURN_S90;    *dir = TurnDir::right; return true;
    case ACT_S90_LEFT:        *kind = TURN_L90;    *dir = TurnDir::left;  return true;
    case ACT_S90_RIGHT:       *kind = TURN_L90;    *dir = TurnDir::right; return true;
    case ACT_S180_LEFT:       *kind = TURN_180;    *dir = TurnDir::left;  return true;
    case ACT_S180_RIGHT:      *kind = TURN_180;    *dir = TurnDir::right; return true;
    case ACT_S45_in_LEFT:     *kind = TURN_IN45;   *dir = TurnDir::left;  return true;
    case ACT_S45_in_RIGHT:    *kind = TURN_IN45;   *dir = TurnDir::right; return true;
    case ACT_S45_out_LEFT:    *kind = TURN_OUT45;  *dir = TurnDir::left;  return true;
    case ACT_S45_out_RIGHT:   *kind = TURN_OUT45;  *dir = TurnDir::right; return true;
    case ACT_V90_LEFT:        *kind = TURN_V90;    *dir = TurnDir::left;  return true;
    case ACT_V90_RIGHT:       *kind = TURN_V90;    *dir = TurnDir::right; return true;
    case ACT_S135_in_LEFT:    *kind = TURN_IN135;  *dir = TurnDir::left;  return true;
    case ACT_S135_in_RIGHT:   *kind = TURN_IN135;  *dir = TurnDir::right; return true;
    case ACT_S135_out_LEFT:   *kind = TURN_OUT135; *dir = TurnDir::left;  return true;
    case ACT_S135_out_RIGHT:  *kind = TURN_OUT135; *dir = TurnDir::right; return true;
    default:                  return false;
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
    out->push_back(Step{nullptr, slalom::TurnDir::left, diagonal, TURN_L90, distance});
    return Error::none;
}

constexpr float SPEED_EPS = 1e-3f;   // [mm/s] 速度の比較の許容差（候補の速度は設計値そのもの）
constexpr std::size_t NO_TURN = SIZE_MAX;

// 候補のうち速度 v 以下でいちばん速いもの。無ければ nullptr
const slalom::Param* fastestAtMost(const TurnLadder& l, float v) {
    for (uint8_t i = 0; i < l.count; ++i) {
        if (l.list[i]->speed <= v + SPEED_EPS) return l.list[i];
    }
    return nullptr;
}

// 手順 i の直線の始めと終わりの速度：隣のターンの速度（ターンごとに違ってよい。最初と最後は0）。
// 直線どうしは build() でつなぐので，直線の隣はターンか端
float startSpeed(const Steps& steps, std::size_t i, const RunPreset& p) {
    if (i == 0) return 0.f;
    const slalom::Param* t = steps[i - 1].turn;
    return (t != nullptr) ? t->speed : p.turn_speed;
}
float endSpeed(const Steps& steps, std::size_t i, const RunPreset& p) {
    if (i + 1 == steps.size()) return 0.f;
    const slalom::Param* t = steps[i + 1].turn;
    return (t != nullptr) ? t->speed : p.turn_speed;
}

} // namespace

const char* errorName(Error e) {
    switch (e) {
    case Error::none:          return "none";
    case Error::noMotion:      return "no motion";
    case Error::unknownAction: return "unknown action";
    case Error::turnMissing:   return "turn not in preset";
    case Error::tooManySteps:  return "too many steps";
    case Error::speedUnfit:    return "turn speeds cannot be bridged";
    }
    return "unknown";
}

Error fitSpeeds(Steps* steps, const RunPreset& p) {
    // 短い直線は trapezoid::split が始めから終わりの速度へ1区間で変える（プリセットの accel・decel を超えてよい）ので，
    // 合わせる基準は validate() と同じ機体の上限
    using config::profile_limit::MAX_ACCEL_X;
    using config::profile_limit::MAX_DECEL_X;
    Steps& s = *steps;
    const std::size_t n = s.size();
    // 最後がターンなら，build() がその速度から止まれる直線を後ろに足すので，終わりの停止とは比べない
    const bool stop_after = (n > 0 && s[n - 1].turn == nullptr);
    for (bool changed = true; changed;) {
        changed = false;
        // 隣り合う2つ（prev：前のターン，NO_TURN なら始めの停止。i：次のターン，n なら終わりの停止）と，あいだの直線の長さ d
        std::size_t prev = NO_TURN;
        float d = 0.f;
        for (std::size_t i = 0; i <= n; ++i) {
            if (i < n && s[i].turn == nullptr) {
                d += s[i].distance;
                continue;
            }
            if (i == n && !stop_after) break;
            const float va = (prev == NO_TURN) ? 0.f : s[prev].turn->speed;
            const float vb = (i == n) ? 0.f : s[i].turn->speed;
            if (vb > va + SPEED_EPS) {
                // 加速しきれない：次のターンを落とす
                const float lim = std::sqrt(va * va + 2.f * MAX_ACCEL_X * d);
                if (vb > lim + SPEED_EPS) {
                    const slalom::Param* slow = fastestAtMost(p.turns[s[i].kind], lim);
                    if (slow == nullptr) return Error::speedUnfit;
                    s[i].turn = slow;
                    changed = true;
                }
            } else if (va > vb + SPEED_EPS) {
                // 減速しきれない：前のターンを落とす（始めの停止は 0 なのでここに来ない）
                const float lim = std::sqrt(vb * vb + 2.f * MAX_DECEL_X * d);
                if (va > lim + SPEED_EPS) {
                    const slalom::Param* slow = fastestAtMost(p.turns[s[prev].kind], lim);
                    if (slow == nullptr) return Error::speedUnfit;
                    s[prev].turn = slow;
                    changed = true;
                }
            }
            prev = i;
            d = 0.f;
        }
    }
    return Error::none;
}

Error build(const uint8_vector& actions, const RunPreset& p, float start_offset, Steps* out) {
    out->clear();
    for (uint8_t a : actions) {
        if (a == ACT_FINISH) break;
        Error e = Error::none;
        TurnKind kind = TURN_L90;
        slalom::TurnDir dir = slalom::TurnDir::left;
        if (ACT_MOVE_0SEC <= a && a <= ACT_MOVE_32SEC) {
            e = addStraight(out, false, (a - ACT_MOVE_0SEC) * HALF_MM);
        } else if (ACT_MOVE_0SEC_DIA <= a && a <= ACT_MOVE_32SEC_DIA) {
            e = addStraight(out, true, (a - ACT_MOVE_0SEC_DIA) * DIAG_STEP_MM);
        } else if (turnOf(a, &kind, &dir)) {
            const slalom::Param* turn = p.top(kind);
            if (turn == nullptr) return Error::turnMissing;
            if (out->full()) return Error::tooManySteps;
            out->push_back(Step{turn, dir, false, kind, 0.f});
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
            out->insert(out->begin(), Step{nullptr, slalom::TurnDir::left, false, TURN_L90, start_offset});
        }
    }

    Error fit = fitSpeeds(out, p);
    if (fit != Error::none) return fit;

    // 終わり: 経路はゴール区画の中央で終わる。直線ならそこで止まり（台形の終速0）、
    // ターンならターンの速度のまま中央に着くので、止まれる距離だけ進んで止まる
    if (out->back().turn != nullptr) {
        float v = out->back().turn->speed;
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
    const float v_in = startSpeed(steps, i, p);
    const float v_out = endSpeed(steps, i, p);
    const float v_max = s.diagonal ? p.max_speed_dia : p.max_speed;
    trapezoid::Part parts[trapezoid::MAX_PARTS];
    uint8_t n = trapezoid::split(d, v_in, v_out, v_max, p.accel, p.decel, parts);
    for (uint8_t k = 0; k < n; ++k) out[k] = Segment{nullptr, slalom::TurnDir::left, parts[k].v_end, parts[k].distance};
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

std::size_t edgeBoundaries(const Steps& steps, const RunPreset& /*p*/, int per_turn, float* out, std::size_t max) {
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
                float earliest = std::min(WallEdge::expectedX(WallEdge::left, b, s.turn->speed),
                                          WallEdge::expectedX(WallEdge::right, b, s.turn->speed)) - WINDOW_MM;
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

namespace {
// 斜め（diagonal = true）か縦横の直線の範囲。diagonalRanges・orthogonalRanges の共通部分
std::size_t straightRanges(const Steps& steps, bool diagonal, float* x0, float* x1, std::size_t max) {
    std::size_t n = 0;
    float x = 0.f;   // 手順 i の始めの位置
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const Step& s = steps[i];
        if (s.turn != nullptr) {
            x += slalom::totalDistance(*s.turn, s.dir);
            continue;
        }
        if (s.diagonal == diagonal && s.distance > 0.f && n < max) {
            x0[n] = x;
            x1[n] = x + s.distance;
            ++n;
        }
        x += s.distance;
    }
    return n;
}
} // namespace

std::size_t diagonalRanges(const Steps& steps, float* x0, float* x1, std::size_t max) {
    return straightRanges(steps, true, x0, x1, max);
}

std::size_t orthogonalRanges(const Steps& steps, float* x0, float* x1, std::size_t max) {
    return straightRanges(steps, false, x0, x1, max);
}

} // namespace fast_plan
