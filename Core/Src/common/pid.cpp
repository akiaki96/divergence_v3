#include "common/pid.hpp"
#include "config/mouse_config.hpp"

void PIDController::reset() {
    integral_term_ = 0.f;
    previous_error_ = 0.f;
}

float PIDController::update(float target, float current, float feedforward, float limit, bool& saturated, bool hold_integral) {
    float error = target - current;
    float derivative = (error - previous_error_) / config::control::DT_S;
    previous_error_ = error;

    float u_unsat = (kp * error) + integral_term_ + (kd * derivative) + feedforward;

    float u_sat = u_unsat;
    saturated = false;
    if (u_unsat > limit) {
        u_sat = limit;
        saturated = true;
    } else if (u_unsat < -limit) {
        u_sat = -limit;
        saturated = true;
    }

    // back-calculation: 飽和分(u_sat - u_unsat)だけ積分項を引き戻し，ワインドアップを防ぐ
    if (!hold_integral) {
        integral_term_ += (ki * error + (u_sat - u_unsat) / back_calc_tt) * config::control::DT_S;
    }

    return u_sat;
}