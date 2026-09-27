#include "test/motor_id.hpp"
#include "common/etc.hpp"
#include "config/mouse_config.hpp"

void id_init_log(void) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);

    logger.initLoggedVal();
    // logger.add(
    //     "left_encoder_velocity",
    //     etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderLeft)
    // );
    // logger.add(
    //     "right_encoder_velocity",
    //     etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderRight)
    // );
    logger.add(
        "encoder_velocity_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getCurrentVelocityX>(motorDriver)
    );
    logger.add(
        "battery",
        etl::delegate<float()>::create<Battery, &Battery::voltage>(battery)
    );
    logger.add(
        "Left Duty",
        etl::delegate<float()>::create<Motor, &Motor::getDuty>(motorLeft)
    );
    logger.add(
        "Right Duty",
        etl::delegate<float()>::create<Motor, &Motor::getDuty>(motorRight)
    );
    logger.add(
        "gyro_z",
        etl::delegate<float()>::create<Imu, &Imu::gyroZ>(imu)
    );
    logger.add(
        "accel_x",
        etl::delegate<float()>::create<Imu, &Imu::accelX>(imu)
    );

    ledBar16.set(0xFFFF);
}

// 並進速度PI+FF制御の追従性検証用：id_init_log()の共通フィールドに加え，
// 目標速度target_velocity_xも記録する（実速度との比較でステップ応答・追従誤差を評価するため）
void id_init_log_velocity(void) {
    id_init_log();
    logger.add(
        "target_velocity_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetVelocityX>(motorDriver)
    );
    logger.add(
        "current_distance_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getCurrentPositionX>(motorDriver)
    );
    logger.add(
        "target_distance_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetPositionX>(motorDriver)
    );
    // // 積分ワインドアップとfeedforward寄与を確認するための診断フィールド
    // logger.add(
    //     "pid_integral_term",
    //     etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXIntegralTerm>(motorDriver)
    // );
    // logger.add(
    //     "pid_feedforward",
    //     etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXFeedforward>(motorDriver)
    // );
    // logger.add(
    //     "pid_saturated",
    //     etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXSaturated>(motorDriver)
    // );
}

onenter(right_set050, 
    id_init_log();
    logger.dirName = "right_set0_50";

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setDuty(0.f, 0.5f);
    HAL_Delay(1000);
    motorDriver.setBreak();
    HAL_Delay(500);
    logger.stop();
    ledBar16.set(0xFFFF);
)

// 並進速度PI+FF制御の追従性検証（velocity_x_ff, config::pid_velocity_x）。
// target_velocity_xへステップ指令し，実速度(left/right_encoder_velocity平均)の追従を
// ログから確認する。duration_msは閉ループ時定数λ=0.1s基準で整定後も十分保持できる長さとする。
void velocity_step_tester(float target_velocity_x, uint32_t duration_ms) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    motorDriver.setTargetAccelX(0.f);
    motorDriver.setTargetVelocityX(0.f);
    motorDriver.resetTargetPositionX();

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(target_velocity_x);
    HAL_Delay(duration_ms/2);
    motorDriver.setTargetVelocityX(0.f);
    HAL_Delay(duration_ms/2);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(velocity_step_300,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_300");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(300.f, 2000);
)

onenter(velocity_step_600,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_600");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(600.f, 2000);
)

onenter(velocity_step_900,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_900");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(900.f, 2000);
)

onenter(velocity_step_000,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_000");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(0.f, 2000);
)