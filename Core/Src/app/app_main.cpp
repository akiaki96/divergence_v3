#include "app/app_main.hpp"
#include "common/debug.hpp"
#include "device/led_manager.hpp"
#include "device/device_instance.hpp"
#include "menu/menuInstance.hpp"
#include "tim.h"
#include <cstdio>

extern "C" {
#include "stm32f4xx_hal.h"
#include "gpio.h"
}

void app_init() {

    HAL_TIM_Base_Start_IT(&htim6);
    encoderLeft.init();
    encoderRight.init();

    motorDriver.enable();

    imu.init();
    motorDriver.init();

}

// #include <etl/vector.h>
// void foo() {
//     etl::vector<uint8_t, 10> v;
// }

void app_main() {
    app_init();

    LOG("Hello divergence_v3!!\r\n");
    menuController.currentInfo();
    menuController.tree();

    menuInputController.asyncUpdate(); // LOOP!
}