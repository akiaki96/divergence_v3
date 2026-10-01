#include "device/device_instance.hpp"
#include "menu/menuInstance.hpp"
#include "tim.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        globalTime += config::control::DT_S;
        
        ledBar16.update();
        adcValue.update();

        encoderLeft.update();
        encoderRight.update();
        imu.update();

        planProfile.update();   // センサ更新後に実測の並進速度・位置と角速度・角度を計算（同じtickの値で制御する）
        motorDriver.update(
            planProfile.getCurrentVelocityX(), planProfile.getCurrentPositionX(),
            planProfile.getCurrentOmega(), planProfile.getCurrentAngle()
        );

        logger.sample();

        menuInputController.syncUpdate();
    }
}