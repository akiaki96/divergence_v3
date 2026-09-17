#include "common/pid.hpp"
#include "config/mouse_config.hpp"

void PIDController::reset() {
    integral_ = 0.f;
    previous_error_ = 0.f;
}

float PIDController::update(float target, float current) {
    float error = target - current;
    integral_ += error * config::control::DT_S;
    float derivative = (error - previous_error_) / config::control::DT_S;
    previous_error_ = error;

    return (kp * error) + (ki * integral_) + (kd * derivative) + ff(target);
}

float PIDController::update(float target, float current, float limit, bool& saturated) {
    float error = target - current;
    float temp_integral_ = integral_ + error * config::control::DT_S;
    float derivative = (error - previous_error_) / config::control::DT_S;
    previous_error_ = error;

    float temp_output = (kp * error) + (ki * temp_integral_) + (kd * derivative) + ff(target);

    float output = temp_output;
    if (temp_output > limit) {
        output = limit;
        saturated = true;
    } else if (temp_output < -limit) {
        output = -limit;
        saturated = true;
    } else {
        integral_ = temp_integral_;
        saturated = false;
    }

    return output;
}