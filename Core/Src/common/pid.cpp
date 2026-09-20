#include "common/pid.hpp"
#include "config/mouse_config.hpp"

void PIDController::reset() {
    integral_term_ = 0.f;
    previous_error_ = 0.f;
    ext_ff_ = 0.f;
}

float PIDController::update(float target, float current) {
    float error = target - current;
    integral_term_ += ki * error * config::control::DT_S;
    float derivative = (error - previous_error_) / config::control::DT_S;
    previous_error_ = error;
    last_ff_ = ff(target);

    return (kp * error) + integral_term_ + (kd * derivative) + last_ff_ + ext_ff_;
}

float PIDController::update(float target, float current, float limit, bool& saturated) {
    float error = target - current;
    float derivative = (error - previous_error_) / config::control::DT_S;
    previous_error_ = error;
    last_ff_ = ff(target);

    float u_unsat = (kp * error) + integral_term_ + (kd * derivative) + last_ff_ + ext_ff_;

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
    integral_term_ += (ki * error + (u_sat - u_unsat) / back_calc_tt) * config::control::DT_S;

    return u_sat;
}