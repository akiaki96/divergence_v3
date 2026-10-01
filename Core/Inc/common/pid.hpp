#pragma once

#include "common/etc.hpp"

// PI(D) + feedforward コントローラ（出力は±limitで飽和）。
// アンチワインドアップはback-calculation方式（translational_gain_tuning.md §3.2）:
//   I[k] = I[k-1] + (Ki*e[k] + (u_sat[k]-u_unsat[k])/Tt) * Ts
// feedforwardは呼び出し側が毎回計算してupdate()に渡す（目標値だけでなく目標加速度など
// 複数の量から作るため。出力 u = kp*e + I + kd*de/dt + feedforward）。
// 外側のPループ（位置・角度）は1行で書けるのでこのクラスを使わない（MotorDriver::update()）。
// kp/ki/kd/back_calc_ttはsetGains()で後から書き換えられる前提（ゲインチューニング用）。
class PIDController {
public:

    void reset();
    // 飽和分はback-calculationで積分から引き戻す（feedforwardも飽和判定に含む）
    float update(float target, float current, float feedforward, float limit, bool& saturated);

    void setGains(float kp, float ki, float kd, float back_calc_tt) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->back_calc_tt = back_calc_tt;
    }

    // 診断用ゲッター（積分ワインドアップをログで確認するため）
    float getIntegralTerm() const {
        return integral_term_;
    }

    float kp = 0.f;
    float ki = 0.f;
    float kd = 0.f;
    float back_calc_tt = 1.f;   // アンチワインドアップ back-calculation時定数 [s]

private:
    float integral_term_ = 0.f;   // 積分項の出力寄与（u(t)と同じ単位。ki既反映済み）
    float previous_error_ = 0.f;
};
