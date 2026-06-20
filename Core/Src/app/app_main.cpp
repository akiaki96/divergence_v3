#include "app/app_main.hpp"
#include "common/debug.hpp"
#include <cstdio>

extern "C" {
#include "stm32f4xx_hal.h"
#include "gpio.h"
}

void app_main() {
    LOG("Hello divergence_v3!!\r\n");
    LOG("UART layer ready\r\n");

    uint16_t i = 0;
    while (true) {
        HAL_Delay(100);
        LOG("tick %d\r\n", i++);
    }
}