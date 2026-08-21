#include "device/device_test.hpp"
#include "common/debug.hpp"

onenter(imu_acc, 
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.accelX(), imu.accelY(), imu.accelZ());
        ledBar16.set(imu.accelZ(), pmbit8, 10.f);
    }
)
onenter(imu_gyro, 
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.gyroAngleX(), imu.gyroAngleY(), imu.gyroAngleZ());
        ledBar16.set(imu.gyroAngleZ(), pmlinear8, 360.f);
    }
)

onenter(encoder_right, 
    encoderRight.reset();
    while (true) {
        LOG("dist: %+.2f, vel: %+.2f\r\n", encoderRight.distance(), encoderRight.velocity());
        ledBar16.set(encoderRight.distance(), pmbit8, 180.f*4);
    }
)
onenter(encoder_left, 
    encoderLeft.reset();
    while (true) {
        LOG("dist: %+.2f, vel: %+.2f\r\n", encoderLeft.distance(), encoderLeft.velocity());
        ledBar16.set(encoderLeft.distance(), pmbit8, 180.f*4);
    }
)


onenter(log_wait, 
    logger.initLoggedVal();
    // logger.add("right_encoder_velocity", encoderRight.velocity());
    // logger.add("left_encoder_velocity", encoderLeft.velocity());
    logger.add(
        "left_encoder_velocity",
        etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderLeft)
    );
    logger.add(
        "right_encoder_velocity",
        etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderRight)
    );

    ledBar16.set(0xFFFF);
    HAL_Delay(1000);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(2000);
    
)