#include "device/fan.hpp"
#include <cmath>

Fan::Fan(TIM_HandleTypeDef* htim, uint32_t channel)
    : htim_(htim),
      channel_(channel)
{
}

void Fan::init() {
    setDuty(0.f);
    HAL_TIM_PWM_Start(htim_, channel_);
}

void Fan::setDuty(float duty) {
    duty_ = fmaxf(fminf(duty, 1.f), 0.f);
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim_) + 1;
    __HAL_TIM_SET_COMPARE(htim_, channel_, static_cast<uint32_t>(duty_ * period));
}

void Fan::setPwmPeriod(uint32_t period_ticks) {
    __HAL_TIM_SET_AUTORELOAD(htim_, period_ticks - 1);
    setDuty(duty_);
}
