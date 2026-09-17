#include "device/motorDriver.hpp"
#include "device/device_instance.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "config/mouse_config.hpp"

MotorDriver::MotorDriver(Motor& left, Motor& right)
    :motorLeft_(left),
    motorRight_(right)
{}

float velocity_x_ff(float velocity_x) {
    if (velocity_x == 0.f) return 0.f;
    float sign = (velocity_x > 0.f) ? 1.f : -1.f;
    return (velocity_x / config::pid_velocity_x::a_gain) + sign * config::pid_velocity_x::u0_deadzone;
}

void MotorDriver::init() {
    pid_velocity_x_.setGains(
        config::pid_velocity_x::kp,
        config::pid_velocity_x::ki,
        config::pid_velocity_x::kd,
        velocity_x_ff
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

        case MotorDriverState::setVelocity:
            bool saturated = false;
            float limit = config::pid_velocity_x::voltage_limit_ratio * battery.voltage();
            float base_batt = pid_velocity_x_.update(velocity_x_, (encoderLeft.velocity() + encoderRight.velocity()) / 2.f, limit, saturated);

            setVoltage(base_batt, base_batt);
        break;
    }
}