#include "common/axis_controller.hpp"

void AxisController::reset() {
    outer_.reset();
    inner_.reset();
    command_ = 0.f;
}

float AxisController::update(const AxisReference& ref, const AxisMeasurement& meas, float inner_ff, float inner_limit,
                             float outer_ki_scale) {
    // inner_.saturation()はこのtickのupdate()前なので前tickの飽和。
    // 位置の誤差が正 → 速度指令が増 → 電圧が増 なので，飽和と同じ向きに誤差が押す間は外側の積分を止める
    bool hold_outer_integral = inner_.saturation() * (ref.pos - meas.pos) > 0.f;
    command_ = outer_.update(ref.pos, meas.pos, ref.vel, command_limit_, hold_outer_integral, outer_ki_scale);
    return inner_.update(command_, meas.vel, inner_ff, inner_limit);
}
