#include "app/app_main.hpp"
#include "common/debug.hpp"
#include "device/led_manager.hpp"
#include <cstdio>

extern "C" {
#include "stm32f4xx_hal.h"
#include "gpio.h"
}

void app_main() {
    LOG("Hello divergence_v3!!\r\n");

    LedManager ledManager;
    ledManager.bar.clear();

    uint16_t i = 0;
    while (true) {
        HAL_Delay(100);
        LOG("tick %d\r\n", i++);
        ledManager.bar.set(1<<(i&0xf));
        ledManager.rFront.toggle();
    }
}