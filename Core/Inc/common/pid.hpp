#pragma once

#include "common/etc.hpp"

// PI(D) + feedforward コントローラ。
// アンチワインドアップはback-calculation方式（translational_gain_tuning.md §3.2）:
//   I[k] = I[k-1] + (Ki*e[k] + (u_sat[k]-u_unsat[k])/Tt) * Ts
// kp/ki/kd/ff/back_calc_ttはsetGains()で後から書き換えられる前提（ゲインチューニング用）。
class PIDController {
public:

    void reset();
    float update(float target, float current);
    float update(float target, float current, float limit, bool& saturated);

    void setGains(float kp, float ki, float kd, float (*ff)(float), float back_calc_tt) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->ff = ff;
        this->back_calc_tt = back_calc_tt;
    }

    float kp;
    float ki;
    float kd;
    float back_calc_tt = 1.f;   // アンチワインドアップ back-calculation時定数 [s]

    // ff: 目標値 -> feedforward出力（u(t)と同じ単位で加算される）
    float (*ff)(float);


private:
    float integral_term_;   // 積分項の出力寄与（u(t)と同じ単位。ki既反映済み）
    float previous_error_;
};
