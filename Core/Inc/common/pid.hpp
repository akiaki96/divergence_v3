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

    // 目標値の関数ではない外部のfeedforward（例：指令の加速度FF）。次のupdate()の出力に加算され，
    // 飽和・back-calculationにも含まれる（FF分を含めてワインドアップを防ぐ）。reset()で0に戻る
    void setExternalFF(float value) {
        ext_ff_ = value;
    }

    // 診断用ゲッター（積分ワインドアップ・feedforward寄与をログで確認するため）
    float getIntegralTerm() const {
        return integral_term_;
    }
    float getLastFeedforward() const {
        return last_ff_;
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
    float last_ff_ = 0.f;    // 直近のupdate()で計算されたff(target)（ログ用）
    float ext_ff_ = 0.f;     // setExternalFF()で与えた外部FF
};
