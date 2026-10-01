#include "test/motor_id.hpp"
#include "test/closed_loop_test.hpp"
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
        etl::delegate<float()>::create<PlanProfile, &PlanProfile::getCurrentVelocityX>(planProfile)
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
        etl::delegate<float()>::create<PlanProfile, &PlanProfile::getCurrentPositionX>(planProfile)
    );
    logger.add(
        "target_distance_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetPositionX>(motorDriver)
    );
}

// 回転（角度PI → 角速度PI）の追従性検証用：並進のフィールドに加え，回転の目標値と実測・角度PIの積分項を記録する
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
        etl::delegate<float()>::create<PlanProfile, &PlanProfile::getCurrentAngle>(planProfile)
    );
    logger.add(
        "angle_integral_term",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getAngleIntegralTerm>(motorDriver)
    );
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
static void velocity_step(float target_velocity_x, uint32_t duration_ms) {
    planProfile.setTargetVelocityX(target_velocity_x);
    HAL_Delay(duration_ms/2);
    planProfile.setTargetVelocityX(0.f);
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

// 目標角速度をalpha[dps/s]のランプでomega_targetへ動かす（omega2omegaで，角加速度がalphaになる角度を与える：
// θ = (ω2^2 − ω1^2) / (2α)，αはω1→ω2の向き）。終端角速度で固定されるまで戻らない
static void omega_ramp_to(float omega_target, float alpha) {
    float omega1 = motorDriver.getTargetOmega();
    float a = (omega_target >= omega1) ? alpha : -alpha;
    float angle = (omega_target * omega_target - omega1 * omega1) / (2.f * a);
    planProfile.omega2omega(omega_target, angle);
}

// 回転（config::pid_rotation：角度PI → 角速度PI）の追従性検証。
// 700mm/sで直進してから角速度をランプ指令（運用仕様2500dps/s）で与え，
// hold_ms保持した後にランプで0へ戻す。目標角度は目標角速度の積分（ランプ分を含む）。
// 走行距離は約1m（直進0.5s＋旋回＋停止）なので，旋回で膨らむ分も含め十分な余白を確保すること。
// 記録は全体で約1.84s（±430。ログ上限2.0s以内）。
static void omega_ramp(float omega_target, uint32_t hold_ms) {
    constexpr float V_X = 700.f;          // [mm/s]
    constexpr float ALPHA = 2500.f;       // [dps/s]
    constexpr uint32_t STRAIGHT_MS = 500; // 並進速度が整定するまで

    planProfile.setTargetVelocityX(V_X);
    HAL_Delay(STRAIGHT_MS);
    omega_ramp_to(omega_target, ALPHA);
    HAL_Delay(hold_ms);
    omega_ramp_to(0.f, ALPHA);
    HAL_Delay(200);
    planProfile.setTargetVelocityX(0.f);
    HAL_Delay(300);
}

onenter(omega_ramp_pos430,
    runClosedLoopTest({"rot_angle_pi_v700_x", "omega_ramp_pos430", id_init_log_omega,
                       [] { omega_ramp(430.f, 400); }, 0.f, 0.f, 0});
)

onenter(omega_ramp_neg430,
    runClosedLoopTest({"rot_angle_pi_v700_x", "omega_ramp_neg430", id_init_log_omega,
                       [] { omega_ramp(-430.f, 400); }, 0.f, 0.f, 0});
)

onenter(omega_ramp_pos250,
    runClosedLoopTest({"rot_angle_pi_v700_x", "omega_ramp_pos250", id_init_log_omega,
                       [] { omega_ramp(250.f, 400); }, 0.f, 0.f, 0});
)

onenter(omega_ramp_neg250,
    runClosedLoopTest({"rot_angle_pi_v700_x", "omega_ramp_neg250", id_init_log_omega,
                       [] { omega_ramp(-250.f, 400); }, 0.f, 0.f, 0});
)

onenter(omega_ramp_pos100,
    runClosedLoopTest({"rot_angle_pi_v700_x", "omega_ramp_pos100", id_init_log_omega,
                       [] { omega_ramp(100.f, 400); }, 0.f, 0.f, 0});
)

onenter(omega_ramp_neg100,
    runClosedLoopTest({"rot_angle_pi_v700_x", "omega_ramp_neg100", id_init_log_omega,
                       [] { omega_ramp(-100.f, 400); }, 0.f, 0.f, 0});
)
