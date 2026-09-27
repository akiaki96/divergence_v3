#include "device/device_test.hpp"
#include "common/debug.hpp"

onenter(imu_acc, 
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.accelX(), imu.accelY(), imu.accelZ());
        ledBar16.set(imu.accelZ(), LedBarValMode::pmbit8, 10.f * 1000.f);
    }
)
onenter(imu_gyro, 
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

onenter(battery, 
    while (true) {
        LOG("Battery raw: %04d, (V): %f\r\n", battery.raw_, battery.voltage());
        ledBar16.set(battery.voltage(), LedBarValMode::pmbit8, 12.f);
    }
)
