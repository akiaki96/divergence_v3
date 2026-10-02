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

        odometry.update();   // センサ更新後に実測値を計算する（同じtickの値で制御する）
        if (motorDriver.state == MotorDriverState::setVelocity) {
            planProfile.update();   // 閉ループ中だけ目標軌道を1tick進める
        }
        // 横壁の補正を回転の目標に足す（直進中で有効なときだけ。それ以外はそのまま）
        AxisReference rot_ref = wallControl.apply(
            planProfile.rotReference(), planProfile.getTargetOmega(), planProfile.getTargetVelocityX()
        );
        motorDriver.update(
            planProfile.transReference(), odometry.translation(),
            rot_ref, odometry.rotation()
        );

        logger.sample();

        menuInputController.syncUpdate();
    }
}