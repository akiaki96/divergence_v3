#include "device/device_test.hpp"
#include "common/debug.hpp"
#include "common/wall_sensor.hpp"

onenter(imu_acc, 
    ledBar16.set(0);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0xFFFF);
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.accelX(), imu.accelY(), imu.accelZ());
        ledBar16.set(imu.accelZ(), LedBarValMode::pmbit8, 10.f * 1000.f);
    }
)
onenter(imu_gyro, 
    ledBar16.set(0);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0xFFFF);
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.gyroAngleX(), imu.gyroAngleY(), imu.gyroAngleZ());
        ledBar16.set(imu.gyroAngleZ(), LedBarValMode::pmlinear8, 360.f);
    }
)

onenter(imu_acc_fan, 
    ledBar16.set(0);
    fan.setDuty(0.2);
    HAL_Delay(2000);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0xFFFF);
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.accelX(), imu.accelY(), imu.accelZ());
        ledBar16.set(imu.accelZ(), LedBarValMode::pmbit8, 10.f * 1000.f);
    }
)

onenter(imu_gyro_fan, 
    ledBar16.set(0);
    fan.setDuty(0.2);
    HAL_Delay(2000);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0xFFFF);
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.gyroAngleX(), imu.gyroAngleY(), imu.gyroAngleZ());
        ledBar16.set(imu.gyroAngleZ(), LedBarValMode::pmlinear8, 360.f);
    }
)

onenter(encoder_right, 
    encoderRight.reset();
    while (true) {
        LOG("dist: %+.2f, vel: %+.2f\r\n", encoderRight.distance(), encoderRight.velocity());
        ledBar16.set(encoderRight.distance(), LedBarValMode::pmbit8, 180.f*4);
    }
)
onenter(encoder_left, 
    encoderLeft.reset();
    while (true) {
        LOG("dist: %+.2f, vel: %+.2f\r\n", encoderLeft.distance(), encoderLeft.velocity());
        ledBar16.set(encoderLeft.distance(), LedBarValMode::pmbit8, 180.f*4);
    }
)

// 壁センサーの確認：機体の位置ごとの値と壁の判定を出す。各センサーを手でふさいで位置の対応を確かめ，
// 探索で壁を読む位置（区画境界の config::search::READ_LEAD_MM 手前）での値から config::wall の閾値を決める。
// 前は前左・前右それぞれの判定（FL / FR）と，どちらかが超えたときの前壁の判定（F）を出す。
// 前の閾値をきちんと決めるときは Device → IR → Front check を使う。
// LEDバーは左から 左・前左・前右・右 の壁の判定（壁ありで点灯）
void wall_check_onenter() {
    while (true) {
        wall::Snapshot s = wall::read();
        bool l = wall::hasLeft(s);
        bool fl = wall::hasFrontLeft(s);
        bool fr = wall::hasFrontRight(s);
        bool f = wall::hasFront(s);
        bool r = wall::hasRight(s);
        LOG("L %4d  FL %4d (>%d:%d)  FR %4d (>%d:%d)  R %4d | wall L%d F%d R%d | raw irL %4d irFL %4d irFR %4d irR %4d\r\n",
            s.value[wall::left], s.value[wall::front_left], config::wall::THRESH_FRONT_LEFT, fl,
            s.value[wall::front_right], config::wall::THRESH_FRONT_RIGHT, fr, s.value[wall::right],
            l, f, r, irL.filtered_, irFL.filtered_, irFR.filtered_, irR.filtered_);
        ledBar16.set(static_cast<uint16_t>((l ? 0xF000 : 0) | (fl ? 0x0300 : 0) | (fr ? 0x00C0 : 0) | (r ? 0x000F : 0)));
        HAL_Delay(100);
    }
}

onenter(battery, 
    while (true) {
        LOG("Battery raw: %04d, (V): %f\r\n", battery.raw_, battery.voltage());
        ledBar16.set(battery.voltage(), LedBarValMode::pmbit8, 12.f);
    }
)
