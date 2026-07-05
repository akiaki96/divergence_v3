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

    motor.enable();

    imu.init();
    motor.init();

}


void app_main() {
    app_init();

    // ledBar16.set(0xff00);
    // HAL_Delay(1000);
    // ledBar16.set(0xffff);
    // imu.calibrate();
    // HAL_Delay(1200);
    // ledBar16.set(0x0000);

    LOG("Hello divergence_v3!!\r\n");
    menuController.currentInfo();
    menuController.tree();

    menuInputController.asyncUpdate(); // LOOP!
}