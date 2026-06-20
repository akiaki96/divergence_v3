#pragma once

#include "stm32f4xx_hal.h"

class Led {
public:
    Led(GPIO_TypeDef* port, uint16_t pin);

    void on();
    void off();
    void toggle();

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};