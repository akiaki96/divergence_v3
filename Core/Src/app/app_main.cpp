#include "app/app_main.hpp"
#include "common/debug.hpp"
#include "device/led.hpp"
#include <cstdio>

extern "C" {
#include "stm32f4xx_hal.h"
#include "gpio.h"
}

void app_main() {
    LOG("Hello divergence_v3!!\r\n");

    Led led(LED_1_GPIO_Port, LED_1_Pin);

    uint16_t i = 0;
    while (true) {
        HAL_Delay(100);
        LOG("tick %d\r\n", i++);
        led.toggle();
    }
}