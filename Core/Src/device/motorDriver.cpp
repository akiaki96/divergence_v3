#include "device/motorDriver.hpp"
#include "device/device_instance.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "config/mouse_config.hpp"

MotorDriver::MotorDriver(Motor& left, Motor& right)
    :motorLeft_(left),
    motorRight_(right)
{}

// 並進速度feedforward（translational_gain_tuning.md §2.2の定常成分のみの簡易版）:
//   u_ff(v) = v/A_GAIN + U0_DEADZONE*sign(v)
// 目標軌道がstep（滑らかな加減速プロファイルでない）前提のため微分項(T/K * v_dot)は省略し，
// 過渡応答はPIフィードバックに委ねる。台形加減速プロファイルを導入する場合はここを拡張する。
float velocity_x_ff(float velocity_x) {
    float abs_v = (velocity_x < 0.f) ? -velocity_x : velocity_x;
    if (abs_v < config::pid_velocity_x::ZERO_VELOCITY_EPS) return 0.f;

    float sign = (velocity_x > 0.f) ? 1.f : -1.f;
    return (velocity_x / config::pid_velocity_x::A_GAIN) + sign * config::pid_velocity_x::U0_DEADZONE;
}

void MotorDriver::init() {
    pid_velocity_x_.setGains(
        config::pid_velocity_x::kp,
        config::pid_velocity_x::ki,
        config::pid_velocity_x::kd,
        velocity_x_ff,
        config::pid_velocity_x::BACK_CALC_TT
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

float MotorDriver::getLeftDuty(void) const {
    return motorLeft_.getDuty();
}
float MotorDriver::getRightDuty(void) const {
    return motorRight_.getDuty();
}

void MotorDriver::setLampGrad(float lamp) {
    lamp_grad_ = lamp;
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
    duty_diff_ = 0.f;
    applied_duty_diff_ = 0.f;
    prbs_rot_diff_ = nullptr;
    state = MotorDriverState::setVelocity;
}


void MotorDriver::update() {
    switch (state) {
        case MotorDriverState::off:
            setDuty(0.f, 0.f);
        break;

        case MotorDriverState::setDuty:
        break;

        case MotorDriverState::lampDuty:
            setDuty(
                getLeftDuty() + lamp_grad_*config::control::DT_S,
                getRightDuty() + lamp_grad_*config::control::DT_S
            );
        break;

        case MotorDriverState::setVoltage:
            setDuty(
                dutyFromVoltage(target_voltage_L_),
                dutyFromVoltage(target_voltage_R_)
            );
        break;

        case MotorDriverState::prbsDuty: {
            if (prbs_ == nullptr || prbs_->isFinished()) {
                setBreak();
                state = MotorDriverState::off;
                break;
            }
            float duty = prbs_->update();
            setDuty(duty, duty);   // 並進方向：左右同相
            break;
        }

        case MotorDriverState::modeSelecting:
        break;

        case MotorDriverState::setVelocity: {
            bool saturated = false;
            float limit = config::pid_velocity_x::voltage_limit_ratio * battery.voltage();
            float base_batt = pid_velocity_x_.update(velocity_x_, (encoderLeft.velocity() + encoderRight.velocity()) / 2.f, limit, saturated);
            velocity_pid_saturated_ = saturated;

            // 注意：setVoltage()はstateをMotorDriverState::setVoltageへ書き換えてしまうため，
            // ここで呼ぶとPIDが次tickから二度と回らなくなる（固定電圧のstep入力に化ける）。
            // stateをsetVelocityに保ったまま，直接duty変換のみ行う。
            // duty_diff_（回転方向のstep/PRBS同定用）はduty空間で左右に重畳する：
            // v_L = v* - diff/2, v_R = v* + diff/2 のkinematic配分に対応。
            // prbs_rot_diff_が設定されていればPRBS出力を優先する（回転PRBS同定用）。
            float diff = duty_diff_;
            if (prbs_rot_diff_ != nullptr && !prbs_rot_diff_->isFinished()) {
                diff = prbs_rot_diff_->update();
            }
            applied_duty_diff_ = diff;   // ログ用：PRBS駆動時もgetDutyDiff()で実値を参照できるようにする
            float base_duty = dutyFromVoltage(base_batt);
            float half_diff = diff / 2.f;
            setDuty(base_duty - half_diff, base_duty + half_diff);
            break;
        }
    }
}