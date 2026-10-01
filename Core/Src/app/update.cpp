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

        planProfile.update();   // センサ更新後に実測値を計算し，目標軌道を1tick進める（同じtickの値で制御する）
        motorDriver.update(
            planProfile.transReference(), planProfile.transMeasurement(),
            planProfile.rotReference(), planProfile.rotMeasurement()
        );

        logger.sample();

        menuInputController.syncUpdate();
    }
}