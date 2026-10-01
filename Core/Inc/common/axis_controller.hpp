#pragma once

#include "common/pid.hpp"
#include "common/types.hpp"

// 1軸（並進 or 回転）のカスケード制御：
//   外側：速度指令 v_cmd = v_ref（FF）+ PI(pos_ref − pos)        … 並進は位置P（ki=0），回転は角度PI
//   内側：電圧 u = inner_ff + PI(v_cmd − vel)                   … 並進は速度PI，回転は角速度PI
// 内側の電圧FFは呼び出し側が目標軌道だけから計算して渡す（実測値を通さない＝2自由度制御のFF）。
// 内側が飽和している向きへさらに外側の誤差が押す間は外側の積分を止める（条件付き積分）
class AxisController {
public:
    constexpr AxisController(PIController outer, float command_limit, PIController inner)
        : outer_(outer), inner_(inner), command_limit_(command_limit) {}

    void reset();

    // 出力は内側の電圧 [V]（±inner_limitで飽和）
    float update(const AxisReference& ref, const AxisMeasurement& meas, float inner_ff, float inner_limit);

    // 外側の出力（速度指令 [mm/s] / 角速度指令 [dps]）
    float command() const {
        return command_;
    }

    // 内側の直近の飽和の向き（+1:上限, −1:下限, 0:飽和なし）
    float saturation() const {
        return inner_.saturation();
    }

    // ---- ログ用 ----
    float outerIntegralTerm() const {
        return outer_.getIntegralTerm();
    }

    float innerIntegralTerm() const {
        return inner_.getIntegralTerm();
    }

private:
    PIController outer_;
    PIController inner_;
    const float command_limit_;
    float command_ = 0.f;
};
