#include "device/motorDriver.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "config/mouse_config.hpp"

MotorDriver::MotorDriver(Motor& left, Motor& right)
    :motorLeft_(left),
    motorRight_(right)
{}

void MotorDriver::init() {

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

void MotorDriver::update() {
    switch (state) {
        case MotorDriverState::setDuty:
        break;

        case MotorDriverState::lampDuty:
            setDuty(
                getLeftDuty() + lamp_grad_*config::control::DT_S,
                getRightDuty() + lamp_grad_*config::control::DT_S
            );
        break;
    }
}