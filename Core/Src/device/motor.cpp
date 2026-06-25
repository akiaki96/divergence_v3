#include "device/motor.hpp"
#include "config/mouse_config.hpp"
#include <cstdlib>

/* |duty| < 1.0f */
void Motor::setDuty(float duty)
{
    /* |pwm| < config::motor::MAX_PWM */
    uint16_t pwm = static_cast<uint16_t>(std::abs(duty) * config::motor::MAX_PWM);

    if ((duty >= 0.f) ^ (direction_ == Direction::Normal)) {
        HAL_GPIO_WritePin(cwPort_, cwPin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ccwPort_, ccwPin_, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(cwPort_, cwPin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ccwPort_, ccwPin_, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(htim_, channel_, pwm);
}

void Motor::setBreak() {
    HAL_GPIO_WritePin(cwPort_, cwPin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ccwPort_, ccwPin_, GPIO_PIN_SET);
}