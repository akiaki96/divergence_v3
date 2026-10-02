#pragma once

#include <cstdint>
#include "device/planProfile.hpp"

// スラローム（並進速度を保ったまま旋回する）のパラメータと，PlanProfileへの積み方。
// パラメータは tools/slalom_params.json（設計値）と tools/slalom_tuning.json（実機での調整の差分）から
// tools/gen_slalom_params.py がビルド時に config/slalom_params.hpp として生成する。
//
// 1回のスラロームは
//   入口オフセット（直進）→ 角速度 0→omega_max（等角加速度）→ omega_maxで等角速度 → omega_max→0 → 出口オフセット（直進）
// で，並進速度はspeedのまま。角速度の加速・減速の角度は omega_max²/(2·alpha) で，シミュレータ
// （tools/slalom_profile_designer.py）の加速・等角速度・減速フェーズと同じ形になる
namespace slalom {

// 入口・出口の基準点（オフセットの起点・終点）が区画のどこにあるか（tools/slalom_presets.py の entry / exit）
enum class Anchor : uint8_t {
    edge,      // 区画境界（壁の中央）
    center,    // 区画中央
    diagonal,  // 斜め走行側
};

enum class TurnDir : uint8_t { left, right };

struct Param {
    const char* name;         // 例 "S90_500"（生成ヘッダの定数名と同じ）
    const char* speed_name;   // 例 "500" / "500 fan"（メニューで速度を選ぶときの表示）
    float angle;         // [deg] 旋回角の大きさ
    Anchor entry;
    Anchor exit;
    float speed;         // [mm/s] 並進速度（旋回中も一定）
    float omega_max;     // [dps] 最大角速度
    float alpha;         // [dps/s] 角加速度（加速・減速とも）
    float pre_offset;    // [mm] 入口の基準点から旋回を始めるまでの直進
    float post_offset;   // [mm] 旋回を終えてから出口の基準点までの直進
    bool fan;            // ファン（吸引）を回して走る条件で設計したか（config::fan::RUN_DUTYで回す）
};

// 生成ヘッダのALLの中で，同じ種類のターンのパラメータが並んでいる範囲（ALL[first]からcount個，速度の昇順）。
// メニューで種類→速度の順に選ぶために使う
struct TurnGroup {
    const char* name;   // 例 "S90"
    uint8_t first;
    uint8_t count;
};

// 角速度の台形の角度の内訳
struct Shape {
    float ramp_angle;     // [deg] 加速（＝減速）の角度
    float cruise_angle;   // [deg] 等角速度の角度（負なら台形にならない）
};

constexpr Shape shapeOf(const Param& p) {
    float ramp = p.omega_max * p.omega_max / (2.f * p.alpha);
    return {ramp, p.angle - 2.f * ramp};
}

// 旋回（入口・出口オフセットを除く）の間に進む距離 [mm]
constexpr float turnDistance(const Param& p) {
    Shape s = shapeOf(p);
    float ramp_time = p.omega_max / p.alpha;   // [s] 加速・減速それぞれ
    float cruise_time = s.cruise_angle / p.omega_max;
    return p.speed * (2.f * ramp_time + cruise_time);
}

// 入口オフセットから出口オフセットまでの距離 [mm]
constexpr float totalDistance(const Param& p) {
    return p.pre_offset + turnDistance(p) + p.post_offset;
}

// 積む前の検査：PlanProfileが積むときと同じvalidateSegment()で，オフセットの直進と角速度の各区間を調べる。
// オフセットは0なら積まない（負は弾く）。等角速度の角度が負（omega_maxまで加速しきれない）ならtooShort
constexpr SegmentResult validate(const Param& p) {
    using namespace config::profile_limit;
    if (p.pre_offset != 0.f) {
        SegmentResult r = validateSegment(p.speed, p.speed, p.pre_offset, MAX_ACCEL_X, MAX_DECEL_X);
        if (r != SegmentResult::ok) return r;
    }
    Shape s = shapeOf(p);
    if (s.cruise_angle < 0.f) return SegmentResult::tooShort;
    SegmentResult r = validateSegment(0.f, p.omega_max, s.ramp_angle, MAX_ALPHA, MAX_ALPHA_DECEL);
    if (r != SegmentResult::ok) return r;
    if (s.cruise_angle > 0.f) {
        r = validateSegment(p.omega_max, p.omega_max, s.cruise_angle, MAX_ALPHA, MAX_ALPHA_DECEL);
        if (r != SegmentResult::ok) return r;
    }
    r = validateSegment(p.omega_max, 0.f, s.ramp_angle, MAX_ALPHA, MAX_ALPHA_DECEL);
    if (r != SegmentResult::ok) return r;
    if (p.post_offset != 0.f) {
        r = validateSegment(p.speed, p.speed, p.post_offset, MAX_ACCEL_X, MAX_DECEL_X);
    }
    return r;
}

// スラローム1回分の区間を積む（待たずに戻る）。並進の目標速度がspeedになっている（直前の区間の終速がspeed）こと。
// 先にvalidate()で検査し，通らなければ何も積まずにその結果を返す
SegmentResult push(PlanProfile& plan, const Param& p, TurnDir dir);

const char* resultName(SegmentResult r);

} // namespace slalom
