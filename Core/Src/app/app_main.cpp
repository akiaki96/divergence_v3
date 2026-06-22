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

void app_init() {
    LOG("Hello divergence_v3!!\r\n");

    HAL_TIM_Base_Start_IT(&htim6);
    encoderLeft.init();
    encoderRight.init();
}


void app_main() {
    app_init();

    uint16_t i = 0;
    while (true) {
        HAL_Delay(100);
        LOG("encoderLeft %6.2fmm/s, encoderRight %6.2fmm/s\r\n", encoderLeft.velocity(), encoderRight.velocity());
        LOG("encoderLeft %6.2fmm, encoderRight %6.2fmm\r\n", encoderLeft.distance(), encoderRight.distance());
        ledBar16.set(1<<(i&0xf));
        i++;
    }
}