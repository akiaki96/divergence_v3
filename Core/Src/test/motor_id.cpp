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

// 回転角速度PI+FF・角度P制御の追従性検証用：並進のフィールドに加え，回転の目標値と実測を記録する
void id_init_log_omega(void) {
    id_init_log();
    logger.add(
        "target_velocity_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetVelocityX>(motorDriver)
    );
    logger.add(
        "target_omega",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetOmega>(motorDriver)
    );
    logger.add(
        "target_angle",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetAngle>(motorDriver)
    );
    logger.add(
        "current_angle",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getCurrentAngle>(motorDriver)
    );
    logger.add(
        "omega_integral_term",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getOmegaIntegralTerm>(motorDriver)
    );
    // omega_feedforward（getOmegaFeedforward）はtarget_omegaとランプの角加速度から後計算できるので記録しない
    // （Global_time込み12列＝2000サンプル＝2.0sに試験全体を収めるため。logger::MAX_BUFFER_SIZE参照）
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
    motorDriver.setTargetAlpha(0.f);   // 回転は目標角度0を保持（直進）
    motorDriver.setTargetOmega(0.f);
    motorDriver.resetTargetAngle();

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
    motorDriver.setBreak();
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

// 目標角速度をalpha[dps/s]のランプでomega_targetへ動かす（ランプ終了後は値を丸めて角加速度0）
static void omega_ramp_to(float omega_target, float alpha) {
    float d = omega_target - motorDriver.getTargetOmega();
    float a = (d >= 0.f) ? alpha : -alpha;
    uint32_t ramp_ms = (uint32_t)(1000.f * d / a);
    motorDriver.setTargetAlpha(a);
    HAL_Delay(ramp_ms);
    motorDriver.setTargetAlpha(0.f);
    motorDriver.setTargetOmega(omega_target);
}

// 回転角速度PI+FF（omega_ff, config::pid_omega）と角度P（config::pid_angle）の追従性検証。
// FF・Tiは並進700mm/sで同定した値なので，700mm/sで直進してから角速度をランプ指令（運用仕様2500dps/s）で与え，
// hold_ms保持した後にランプで0へ戻す。目標角度は目標角速度の積分（ランプ分を含む）。
// 走行距離は約1m（直進0.5s＋旋回＋停止）なので，旋回で膨らむ分も含め十分な余白を確保すること。
// 記録は全体で約1.84s（±430。ログ上限2.0s以内）。
void omega_ramp_tester(float omega_target, uint32_t hold_ms) {
    constexpr float V_X = 700.f;          // [mm/s]
    constexpr float ALPHA = 2500.f;       // [dps/s]
    constexpr uint32_t STRAIGHT_MS = 500; // 並進速度が整定するまで

    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    motorDriver.setTargetAccelX(0.f);
    motorDriver.setTargetVelocityX(0.f);
    motorDriver.resetTargetPositionX();
    motorDriver.setTargetAlpha(0.f);
    motorDriver.setTargetOmega(0.f);
    motorDriver.resetTargetAngle();

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(V_X);
    HAL_Delay(STRAIGHT_MS);
    omega_ramp_to(omega_target, ALPHA);
    HAL_Delay(hold_ms);
    omega_ramp_to(0.f, ALPHA);
    HAL_Delay(200);
    motorDriver.setTargetVelocityX(0.f);
    HAL_Delay(300);
    logger.stop();
    motorDriver.setBreak();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(omega_ramp_pos430,
    id_init_log_omega();
    logger.setDirName("omega_2dof_v700_x");
    logger.setFileName("omega_ramp_pos430");
    logger.setIncludeTimestamp(false);
    omega_ramp_tester(430.f, 400);
)

onenter(omega_ramp_neg430,
    id_init_log_omega();
    logger.setDirName("omega_2dof_v700_x");
    logger.setFileName("omega_ramp_neg430");
    logger.setIncludeTimestamp(false);
    omega_ramp_tester(-430.f, 400);
)

onenter(omega_ramp_pos250,
    id_init_log_omega();
    logger.setDirName("omega_2dof_v700_x");
    logger.setFileName("omega_ramp_pos250");
    logger.setIncludeTimestamp(false);
    omega_ramp_tester(250.f, 400);
)

onenter(omega_ramp_neg250,
    id_init_log_omega();
    logger.setDirName("omega_2dof_v700_x");
    logger.setFileName("omega_ramp_neg250");
    logger.setIncludeTimestamp(false);
    omega_ramp_tester(-250.f, 400);
)

onenter(omega_ramp_pos100,
    id_init_log_omega();
    logger.setDirName("omega_2dof_v700_x");
    logger.setFileName("omega_ramp_pos100");
    logger.setIncludeTimestamp(false);
    omega_ramp_tester(100.f, 400);
)

onenter(omega_ramp_neg100,
    id_init_log_omega();
    logger.setDirName("omega_2dof_v700_x");
    logger.setFileName("omega_ramp_neg100");
    logger.setIncludeTimestamp(false);
    omega_ramp_tester(-100.f, 400);
)
