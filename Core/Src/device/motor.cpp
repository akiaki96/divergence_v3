#include "device/motor.hpp"
#include "config/mouse_config.hpp"
#include <cmath>
#include <cstdlib>

Motor::Motor(
    TIM_HandleTypeDef* htim,
    uint32_t channel,
    GPIO_TypeDef* cwPort,
    uint16_t cwPin,
    GPIO_TypeDef* ccwPort,
    uint16_t ccwPin,
    Direction direction
)
    : htim_(htim),
      channel_(channel),
      cwPort_(cwPort),
      cwPin_(cwPin),
      ccwPort_(ccwPort),
      ccwPin_(ccwPin),
      direction_(direction)
{
}

/* |duty| < 1.0f */
void Motor::setDuty(float duty)
{
    duty_ = fmaxf(fminf(duty, config::motor::MAX_DUTY), -config::motor::MAX_DUTY);
    
    /* |pwm| < config::motor::MAX_PWM */
    uint16_t pwm = static_cast<uint16_t>(std::abs(duty) * config::motor::MAX_PWM);

    if ((duty >= 0.f) ^ (direction_ == Direction::Normal)) {
        HAL_GPIO_WritePin(cwPort_, cwPin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ccwPort_, ccwPin_, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(cwPort_, cwPin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ccwPort_, ccwPin_, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(htim_, channel_, pwm);
}

void Motor::setBreak() {
    HAL_GPIO_WritePin(cwPort_, cwPin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ccwPort_, ccwPin_, GPIO_PIN_SET);
}