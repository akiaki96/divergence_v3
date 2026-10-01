#include "device/motorDriver.hpp"
#include "device/device_instance.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "config/mouse_config.hpp"

MotorDriver::MotorDriver(Motor& left, Motor& right)
    :motorLeft_(left),
    motorRight_(right)
{}

// 並進速度feedforward [V]（translational_gain_tuning.md §2.2 + 1次遅れの逆モデル）:
//   u_ff(v, a) = v/A_GAIN + U0_DEADZONE*sign(v) + ACCEL_FF_GAIN*a
// 静的成分は速度指令v（位置Pの補正を含む）から，加速度成分は軌道の目標加速度aから作る
static float velocity_x_ff(float velocity_x, float accel_x) {
    float accel_ff = config::pid_velocity_x::ACCEL_FF_GAIN * accel_x;
    float abs_v = (velocity_x < 0.f) ? -velocity_x : velocity_x;
    if (abs_v < config::pid_velocity_x::ZERO_VELOCITY_EPS) return accel_ff;

    float sign = (velocity_x > 0.f) ? 1.f : -1.f;
    return (velocity_x / config::pid_velocity_x::A_GAIN) + sign * config::pid_velocity_x::U0_DEADZONE + accel_ff;
}

void MotorDriver::init() {
    pid_velocity_x_.setGains(
        config::pid_velocity_x::kp,
        config::pid_velocity_x::ki,
        config::pid_velocity_x::kd,
        config::pid_velocity_x::BACK_CALC_TT
    );

    pid_angle_.setGains(
        config::pid_rotation::ANGLE_KP,
        config::pid_rotation::ANGLE_KI,
        0.f,
        config::pid_rotation::ANGLE_BACK_CALC_TT
    );

    pid_omega_.setGains(
        config::pid_rotation::OMEGA_KP,
        config::pid_rotation::OMEGA_KI,
        0.f,
        config::pid_rotation::OMEGA_TI
    );
}

void MotorDriver::enable() {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void MotorDriver::disable() {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}

float MotorDriver::dutyFromVoltage(float voltage) const {
    float vbatt = battery.voltage();
    // 安全下限：Vbatt異常低下（センサ異常・切断等）時のゼロ割り/暴走防止
    if (vbatt < config::motor::kVbattMinSafe) vbatt = config::motor::kVbattMinSafe;

    float duty = voltage / vbatt;
    // duty飽和処理
    if (duty >  config::motor::MAX_DUTY) duty =  config::motor::MAX_DUTY;
    if (duty < -config::motor::MAX_DUTY) duty = -config::motor::MAX_DUTY;
    return duty;
}

void MotorDriver::switchToVelocityX() {
    pid_velocity_x_.reset();
    pid_angle_.reset();
    pid_omega_.reset();
    rotation_saturation_ = 0.f;
    state = MotorDriverState::setVelocity;
}

void MotorDriver::update(float current_velocity_x, float current_position_x, float current_omega, float current_angle) {
    switch (state) {
        case MotorDriverState::setDuty:
        break;

        case MotorDriverState::modeSelecting:
        break;

        case MotorDriverState::setVelocity: {
            // 目標値（軌道）はPlanProfile::update()が生成して渡す。ここでは追従制御だけを行う。
            // 各ループのfeedforwardは目標値から計算してupdate()に渡す
            // 並進：位置P（FF=目標速度）→ 速度PI（FF=静的成分＋加速度成分）
            float local_target_velocity_x = target_velocity_x_ + config::pid_position_x::kp * (target_position_x_ - current_position_x);

            bool saturated = false;
            float limit = config::pid_velocity_x::voltage_limit_ratio * battery.voltage();
            float velocity_ff = velocity_x_ff(local_target_velocity_x, target_accel_x_);
            float base_batt = pid_velocity_x_.update(local_target_velocity_x, current_velocity_x, velocity_ff, limit, saturated);
            velocity_pid_saturated_ = saturated;

            // ---- 回転：角度PI（FF=目標角速度）→ 角速度の指令 → 角速度PI（ジャイロ）----
            // 電圧差が飽和している向きへさらに誤差が押す間は角度PIの積分を止める（角度誤差が正→ω_cmd増→電圧差増）
            float angle_error = target_angle_ - current_angle;
            bool hold_angle_integral = rotation_saturation_ * angle_error > 0.f;
            bool angle_saturated = false;
            omega_cmd_ = pid_angle_.update(target_angle_, current_angle, target_omega_, config::pid_rotation::OMEGA_CMD_LIMIT, angle_saturated, hold_angle_integral);

            float diff_limit = config::pid_rotation::VOLTAGE_LIMIT_RATIO * battery.voltage();
            bool omega_saturated = false;
            float diff_batt = pid_omega_.update(omega_cmd_, current_omega, 0.f, diff_limit, omega_saturated);
            rotation_saturation_ = omega_saturated ? ((diff_batt > 0.f) ? 1.f : -1.f) : 0.f;

            // diff = R − L（正でω正）。v_L = v − diff/2, v_R = v + diff/2 のkinematic配分
            float half_diff = diff_batt / 2.f;
            setDuty(dutyFromVoltage(base_batt - half_diff), dutyFromVoltage(base_batt + half_diff));
            break;
        }
    }
}