#pragma once

#include <cstdint>
#include "common/types.hpp"

// 横壁による向きの補正。直進中（目標の角速度が0で並進が動いている間）だけ，左右のセンサーの値と
// 中心線上の値（config::wall::REF_*）の差から補正の角速度を作り，回転の目標に足す。
//
// 補正の角速度は積分して目標角度にも足し続ける（回転の外側は角度PIなので，角速度だけ足すと
// 角度の誤差として打ち消されてしまう）。足した角度は旋回の後も残る：旋回は相対角度で積むので，
// 「補正後の向き」を基準に曲がることになる
//
// 最短走行では縦横の直線の範囲（経路に沿った目標の並進位置 [x0, x1)）を教え，その中だけで補正する。
// 斜めの直線や，スラロームのオフセットの直進（入45°の出口側・出45°の入口側などは斜め）も目標の角速度は0なので，
// 角速度だけでは見分けられない。範囲を教えなければ（探索）直進中ならいつでも補正する
class WallControl {
public:
    // 走行の開始時（PlanProfile::reset() と同時）に呼ぶ。補正を0に戻して止め，範囲も外す
    void reset();

    void enable(bool on) {
        enabled_ = on;
    }

    // 補正する範囲 [x0[i], x1[i])（小さい順。PlanProfile の目標位置と同じ座標：前のターンの出口の基準点から
    // 次のターンの入口の基準点まで）。配列はコピーしないので，走り終わる（reset() する）まで残るものを渡す。
    // enable(true) の前に呼ぶ（ISRと同時に書き換えない）
    void setRanges(const float* x0, const float* x1, uint16_t n);

    // ISRから毎tick呼ぶ。rot は PlanProfile の回転の目標，omega_target / v_target / x_target は今の目標の
    // 角速度・並進速度・並進位置（直進中か，教えた範囲の中かの判定に使う）。補正を足した回転の目標を返す
    AxisReference apply(const AxisReference& rot, float omega_target, float v_target, float x_target);

    // ---- ログ用 ----
    float omega() const {
        return omega_;
    }

    float offset() const {
        return offset_deg_;
    }

private:
    // 範囲を教えていれば，x がどれかの範囲の中か（x は増えるだけなので，過ぎた範囲は進めて捨てる）
    bool inRange(float x);

    volatile bool enabled_ = false;
    bool use_ranges_ = false;    // false：範囲を教えていない（直進中ならいつでも補正する）
    const float* x0_ = nullptr;
    const float* x1_ = nullptr;
    uint16_t range_count_ = 0;   // 教えた範囲の数（0 ならどこでも補正しない）
    uint16_t range_index_ = 0;   // 今の（次の）範囲
    float omega_ = 0.f;        // [dps] 今のtickの補正
    float offset_deg_ = 0.f;   // [deg] 補正の積分
};
