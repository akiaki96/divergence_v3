#include "common/pid.hpp"
#include "config/mouse_config.hpp"

void PIController::reset() {
    integral_term_ = 0.f;
    saturation_ = 0.f;
}

float PIController::update(float target, float current, float feedforward, float limit, bool hold_integral) {
    float error = target - current;
    float u_unsat = (kp_ * error) + integral_term_ + feedforward;

    float u_sat = u_unsat;
    saturation_ = 0.f;
    if (u_unsat > limit) {
        u_sat = limit;
        saturation_ = 1.f;
    } else if (u_unsat < -limit) {
        u_sat = -limit;
        saturation_ = -1.f;
    }

    // back-calculation: 飽和分(u_sat - u_unsat)だけ積分項を引き戻し，ワインドアップを防ぐ
    if (!hold_integral) {
        integral_term_ += (ki_ * error + (u_sat - u_unsat) / back_calc_tt_) * config::control::DT_S;
    }

    return u_sat;
}
