#pragma once

#include "stm32f4xx_hal.h"
#include "common/types.hpp"

class Motor {
public:
    Motor() = delete;
    Motor(
        TIM_HandleTypeDef* htim,
        uint32_t channel,
        GPIO_TypeDef* cwPort,
        uint16_t cwPin,
        GPIO_TypeDef* ccwPort,
        uint16_t ccwPin,
        Direction direction
    );

    void setDuty(float duty);
    void setBreak();

    float getDuty() const { return duty_; }

private:
    TIM_HandleTypeDef* const htim_;
    uint32_t const channel_;

    GPIO_TypeDef* const cwPort_;
    uint16_t const cwPin_;

    GPIO_TypeDef* const ccwPort_;
    uint16_t const ccwPin_;

    Direction const direction_;

    float duty_ = 0.f;
};