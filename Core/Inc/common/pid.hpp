#pragma once

// PI + feedforward コントローラ（出力は±limitで飽和）。ゲインは構築時に定数で与える。
// アンチワインドアップはback-calculation方式（translational_gain_tuning.md §3.2）:
//   I[k] = I[k-1] + (Ki*e[k] + (u_sat[k]-u_unsat[k])/Tt) * Ts
// feedforwardは呼び出し側が毎回計算してupdate()に渡す（目標値だけでなく目標加速度など
// 複数の量から作るため。出力 u = kp*e + I + feedforward）。
// カスケードの外側・内側ともにこのクラスを使う（AxisController）。外側の位置P制御はki=0で表す。
class PIController {
public:
    // back_calc_tt: アンチワインドアップ back-calculation時定数 [s]
    constexpr PIController(float kp, float ki, float back_calc_tt)
        : kp_(kp), ki_(ki), back_calc_tt_(back_calc_tt) {}

    void reset();

    // 飽和分はback-calculationで積分から引き戻す（feedforwardも飽和判定に含む）。
    // hold_integral=trueのtickは積分しない（カスケードの下流が飽和しているときの条件付き積分に使う）
    float update(float target, float current, float feedforward, float limit, bool hold_integral = false);

    // 直近のupdate()の飽和の向き（+1:上限, −1:下限, 0:飽和なし）。次のupdate()まで保持する
    float saturation() const {
        return saturation_;
    }

    // 診断用ゲッター（積分ワインドアップをログで確認するため）
    float getIntegralTerm() const {
        return integral_term_;
    }

private:
    const float kp_;
    const float ki_;
    const float back_calc_tt_;

    float integral_term_ = 0.f;   // 積分項の出力寄与（u(t)と同じ単位。ki既反映済み）
    float saturation_ = 0.f;
};
