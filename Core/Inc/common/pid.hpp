#pragma once

#include "common/etc.hpp"

// PI(D) + feedforward コントローラ。
// アンチワインドアップはback-calculation方式（translational_gain_tuning.md §3.2）:
//   I[k] = I[k-1] + (Ki*e[k] + (u_sat[k]-u_unsat[k])/Tt) * Ts
// feedforwardは呼び出し側が毎回計算してupdate()に渡す（目標値だけでなく目標加速度など
// 複数の量から作るため。出力 u = kp*e + I + kd*de/dt + feedforward）。
// kp/ki/kd/back_calc_ttはsetGains()で後から書き換えられる前提（ゲインチューニング用）。
class PIDController {
public:

    void reset();
    float update(float target, float current, float feedforward);
    // 出力を±limitで飽和させる版。飽和分はback-calculationで積分から引き戻す（feedforwardも飽和判定に含む）
    float update(float target, float current, float feedforward, float limit, bool& saturated);

    void setGains(float kp, float ki, float kd, float back_calc_tt) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->back_calc_tt = back_calc_tt;
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

private:
    float integral_term_ = 0.f;   // 積分項の出力寄与（u(t)と同じ単位。ki既反映済み）
    float previous_error_ = 0.f;
    float last_ff_ = 0.f;    // 直近のupdate()に渡されたfeedforward（ログ用）
};
