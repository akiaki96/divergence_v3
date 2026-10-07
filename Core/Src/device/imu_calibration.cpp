#include "device/imu_calibration.hpp"

#include "device/device_instance.hpp"
#include "common/debug.hpp"

void calibrateImuForRun(float fan_duty) {
    if (fan_duty > 0.f) {
        fan.setDuty(fan_duty);
        HAL_Delay(config::fan::STEADY_MS);
    }
    imu.calibrate();
    // 校正は制御周期の割り込み（Imu::update）が REFFERENCE_NUM 回の平均を取って終わる。割り込みが止まっていても
    // 抜けられるよう，2倍の時間で打ち切る
    const uint32_t start = HAL_GetTick();
    while (imu.calibrating()) {
        if (HAL_GetTick() - start > 2u * config::imu::REFFERENCE_NUM) {
            LOG("imu: calibration did not finish in %u ms\r\n", static_cast<unsigned>(2u * config::imu::REFFERENCE_NUM));
            break;
        }
        HAL_Delay(1);
    }
}
