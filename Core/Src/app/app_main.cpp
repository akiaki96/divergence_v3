#include "app/app_main.hpp"
#include "common/debug.hpp"
#include "device/led_manager.hpp"
#include "device/device_instance.hpp"
#include "tim.h"
#include <cstdio>

extern "C" {
#include "stm32f4xx_hal.h"
#include "gpio.h"
}

void app_main() {
    LOG("Hello divergence_v3!!\r\n");
    HAL_TIM_Base_Start_IT(&htim6);

    encoderLeft.init();
    encoderRight.init();

    uint16_t i = 0;
    while (true) {
        HAL_Delay(100);
        LOG("encoderLeft %6.2f, encoderRight %6.2f\r\n", encoderLeft.velocity(), encoderRight.velocity());
        ledBar16.set(1<<(i&0xf));
        i++;
    }
}