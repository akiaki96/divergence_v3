#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "menu/menuInstance.hpp"
#include "app/update.hpp"
#include "tim.h"

namespace {
volatile uint32_t g_last_cycles = 0;   // 0：まだ測っていない
volatile uint32_t g_max_gap_cycles = 0;
} // namespace

namespace control_timing {
void reset() {
    g_last_cycles = 0;
    g_max_gap_cycles = 0;
}

uint32_t maxGapUs() {
    return g_max_gap_cycles / (SystemCoreClock / 1000000u);
}
} // namespace control_timing

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        uint32_t now = DWT->CYCCNT;
        if (g_last_cycles != 0 && now - g_last_cycles > g_max_gap_cycles) g_max_gap_cycles = now - g_last_cycles;
        g_last_cycles = now;
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
        // 壁切れで並進の実測位置を補正する（有効なときだけ。同じtickの制御から補正後の位置を使う）
        if (wallEdge.active()) {
            wall::Snapshot ws = wall::read();
            float shift = wallEdge.update(ws.value[wall::left], ws.value[wall::right], odometry.positionX(),
                                          planProfile.getTargetVelocityX(), planProfile.getTargetOmega());
            if (shift != 0.f) odometry.shiftPositionX(shift);
        }
        // 斜めの直線の切れ目からの距離（有効なときだけ）
        wall::Snapshot side = wall::read();
        if (diagEdge.active()) {
            diagEdge.update(side.value[wall::left], side.value[wall::right], odometry.positionX());
        }
        // 横壁の補正を回転の目標に足す（直進中で有効なときだけ。それ以外はそのまま）
        AxisReference rot_ref = wallControl.apply(
            planProfile.rotReference(), planProfile.getTargetOmega(), planProfile.getTargetVelocityX()
        );
        // 斜めの直線では切れ目からの距離の表で向きを補正する（教えた斜めの直線の中で有効なときだけ）
        rot_ref = diagControl.apply(rot_ref, planProfile.getTargetOmega(), planProfile.getTargetVelocityX(),
                                    odometry.positionX(), side.value[wall::left], side.value[wall::right], diagEdge);
        motorDriver.update(
            planProfile.transReference(), odometry.translation(),
            rot_ref, odometry.rotation()
        );

        logger.sample();

        menuInputController.syncUpdate();
    }
}