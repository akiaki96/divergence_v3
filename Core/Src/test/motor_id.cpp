#include "test/motor_id.hpp"
#include "test/closed_loop_test.hpp"
#include "common/etc.hpp"
#include "config/mouse_config.hpp"

void id_init_log(void) {
    logger.initLoggedVal();
    // logger.add<&Encoder::velocity>("left_encoder_velocity", encoderLeft);
    // logger.add<&Encoder::velocity>("right_encoder_velocity", encoderRight);
    logger.add<&Odometry::velocityX>("encoder_velocity_x", odometry);
    logger.add<&Battery::voltage>("battery", battery);
    logger.add<&Motor::getDuty>("Left Duty", motorLeft);
    logger.add<&Motor::getDuty>("Right Duty", motorRight);
    logger.add<&Imu::gyroZ>("gyro_z", imu);
    logger.add<&Imu::accelX>("accel_x", imu);
    // この記録を使う試験（duty・速度ステップ・plan_profileの並進／回転／高速）はどれも静止100msを含めて2.2s以内
    logger.setDuration(2500);

    ledBar16.set(0xFFFF);
}

// 並進速度PI+FF制御の追従性検証用：id_init_log()の共通フィールドに加え，
// 目標速度target_velocity_xも記録する（実速度との比較でステップ応答・追従誤差を評価するため）
void id_init_log_velocity(void) {
    id_init_log();
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
}

// 回転（角度PI → 角速度PI）の追従性検証用：並進のフィールドに加え，回転の目標値と実測・角度PIの積分項を記録する
void id_init_log_omega(void) {
    id_init_log();
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&PlanProfile::getTargetOmega>("target_omega", planProfile);
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&MotorDriver::getAngleIntegralTerm>("angle_integral_term", motorDriver);
}

onenter(right_set050, 
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
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
static void velocity_step(float target_velocity_x, uint32_t duration_ms) {
    planProfile.setVelocityX(target_velocity_x);
    HAL_Delay(duration_ms/2);
    planProfile.setVelocityX(0.f);
    HAL_Delay(duration_ms/2);
}

onenter(velocity_step_300,
    runClosedLoopTest({"velocity_step_x", "velocity_step_300", id_init_log_velocity,
                       [] { velocity_step(300.f, 2000); }, 0.f, 0.f, 0});
)

onenter(velocity_step_600,
    runClosedLoopTest({"velocity_step_x", "velocity_step_600", id_init_log_velocity,
                       [] { velocity_step(600.f, 2000); }, 0.f, 0.f, 0});
)

onenter(velocity_step_900,
    runClosedLoopTest({"velocity_step_x", "velocity_step_900", id_init_log_velocity,
                       [] { velocity_step(900.f, 2000); }, 0.f, 0.f, 0});
)

onenter(velocity_step_000,
    runClosedLoopTest({"velocity_step_x", "velocity_step_000", id_init_log_velocity,
                       [] { velocity_step(0.f, 2000); }, 0.f, 0.f, 0});
)
