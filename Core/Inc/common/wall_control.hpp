#pragma once

#include "common/types.hpp"

// 横壁による向きの補正。直進中（目標の角速度が0で並進が動いている間）だけ，左右のセンサーの値と
// 中心線上の値（config::wall::REF_*）の差から補正の角速度を作り，回転の目標に足す。
//
// 補正の角速度は積分して目標角度にも足し続ける（回転の外側は角度PIなので，角速度だけ足すと
// 角度の誤差として打ち消されてしまう）。足した角度は旋回の後も残る：旋回は相対角度で積むので，
// 「補正後の向き」を基準に曲がることになる
class WallControl {
public:
    // 走行の開始時（PlanProfile::reset() と同時）に呼ぶ。補正を0に戻して止める
    void reset();

    void enable(bool on) {
        enabled_ = on;
    }

    // ISRから毎tick呼ぶ。rot は PlanProfile の回転の目標，omega_target / v_target は今の目標の
    // 角速度・並進速度（直進中かどうかの判定に使う）。補正を足した回転の目標を返す
    AxisReference apply(const AxisReference& rot, float omega_target, float v_target);

    // ---- ログ用 ----
    float omega() const {
        return omega_;
    }

    float offset() const {
        return offset_deg_;
    }

private:
    volatile bool enabled_ = false;
    float omega_ = 0.f;        // [dps] 今のtickの補正
    float offset_deg_ = 0.f;   // [deg] 補正の積分
};
