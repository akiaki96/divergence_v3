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

    logger.start();

    HAL_Delay(100);

    motor.enable();
    motor.setDuty(0.1f, -0.1f);

    HAL_Delay(1000);

    motor.setDuty(0.f, 0.f);

    logger.stop();

    logger.dump();

    uint16_t i = 0;
    while (true) {

    

        // LOG("%04d, %04d, %04d, %04d\r\n", adcValue.irL.filtered_,adcValue.irFL.filtered_, adcValue.irFR.filtered_, adcValue.irR.filtered_);

        // LOG("encoderLeft %6.2fmm/s, encoderRight %6.2fmm/s, imu %6.2fmm/s\r\n", encoderLeft.velocity(), encoderRight.velocity(), imu.velX());
        // LOG("encoderLeft %6.2fmm, encoderRight %6.2fmm, imu %6.2fmm\r\n", encoderLeft.distance(), encoderRight.distance(), imu.posX());
        
        // LOG("%6.2f, %6.2f, %6.2f\r\n", imu.accelX(), imu.accelY(), imu.accelZ());
        // LOG("%6.2f, %6.2f, %6.2f\r\n", imu.velX(), imu.velY(), imu.velZ());
        
        // LOG("%6.2f, %6.2f, %6.2f\r\n", imu.gyroAngleX(), imu.gyroAngleY(), imu.gyroAngleZ());

        // LOG("\n");
        ledBar16.set(1<<(i&0xf));
        i++;
    }
}