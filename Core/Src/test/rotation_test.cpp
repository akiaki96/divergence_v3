#include "test/rotation_test.hpp"
#include "test/closed_loop_test.hpp"

// 回転の制御（config::pid_rotation：角度PI → 角速度PI，どちらもジャイロ）の実機試験。手順はrunClosedLoopTest()を参照。
// 並進の目標速度は0のまま（その場）で回転だけを動かし，角度の追従と角速度の追従を記録する。

// 回転の試験のログ（Global_time込み13列＝1846サンプル）。外側（角度）と内側（角速度）を両方記録し，
// エンコーダ由来の角速度もジャイロと比べるために記録する
static void rotation_init_log(void) {
    logger.initLoggedVal();
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&PlanProfile::getTargetOmega>("target_omega", planProfile);
    logger.add<&Imu::gyroZ>("gyro_z", imu);
    logger.add<&MotorDriver::getOmegaCommand>("omega_cmd", motorDriver);
    logger.add<&Odometry::encoderOmega>("encoder_omega", odometry);
    logger.add<&MotorDriver::getAngleIntegralTerm>("angle_integral_term", motorDriver);
    logger.add<&MotorDriver::getOmegaIntegralTerm>("omega_integral_term", motorDriver);
    logger.add<&Motor::getDuty>("Left Duty", motorLeft);
    logger.add<&Motor::getDuty>("Right Duty", motorRight);
    logger.add<&Battery::voltage>("battery", battery);
    logger.add<&Odometry::velocityX>("encoder_velocity_x", odometry);
    logger.setDuration(1500);   // その場旋回（180°でも静止100ms＋0.64s＋整定0.5s）
}

// ---- 角度保持 ----
// 静止したまま目標角度0を3.5s保持する。走行中に手で車体を回し，元の向きへ戻るか（符号・PIの効き）を見る。
// 新しい制御を最初に動かすときの確認用（車体が勝手に回り続けるなら符号が逆）。
// ログは静止100msを含めて3.8s記録する（間引きは列数から決まる）
namespace {
constexpr uint32_t ANGLE_HOLD_MS = 3500;
constexpr uint32_t ANGLE_HOLD_LOG_MS = 3800;
}

static void angle_hold_init_log(void) {
    rotation_init_log();
    logger.setDuration(ANGLE_HOLD_LOG_MS);
}

onenter(rot_angle_hold,
    runClosedLoopTest({"rot_omega_pi_gyro", "rot_angle_hold", angle_hold_init_log,
                       [] { HAL_Delay(ANGLE_HOLD_MS); }, 0.f, 0.f, 0});
)

// ---- その場旋回 ----
// 目標角速度を台形にする：0 →（PIVOT_ALPHAで加速）→ PIVOT_OMEGA → 等角速度 →（減速）→ 0。
// 360dpsで車輪は約190mm/s。180°で所要約0.64s，整定0.5sを含めて記録は約1.3s（rotation_init_log() で1.5s）
namespace {
constexpr float PIVOT_OMEGA = 360.f;    // [dps]
constexpr float PIVOT_ALPHA = 2500.f;   // [dps/s]
constexpr float PIVOT_RAMP_ANGLE = PIVOT_OMEGA * PIVOT_OMEGA / (2.f * PIVOT_ALPHA);   // [deg] 加速・減速の角度（約26°）
static_assert(2.f * PIVOT_RAMP_ANGLE < 90.f, "pivot ramps must fit in a 90 deg turn");
}

// angle[deg]は符号つき（正で左旋回）
static void pivot(float angle) {
    float dir = (angle > 0.f) ? 1.f : -1.f;
    planProfile.turn(dir * PIVOT_OMEGA, dir * PIVOT_RAMP_ANGLE);
    planProfile.turn(dir * PIVOT_OMEGA, angle - dir * 2.f * PIVOT_RAMP_ANGLE);
    planProfile.turn(0.f, dir * PIVOT_RAMP_ANGLE);
}

onenter(rot_pivot_pos90,
    runClosedLoopTest({"rot_omega_pi_gyro", "rot_pivot_pos90", rotation_init_log, [] { pivot(90.f); }});
)

onenter(rot_pivot_neg90,
    runClosedLoopTest({"rot_omega_pi_gyro", "rot_pivot_neg90", rotation_init_log, [] { pivot(-90.f); }});
)

onenter(rot_pivot_pos180,
    runClosedLoopTest({"rot_omega_pi_gyro", "rot_pivot_pos180", rotation_init_log, [] { pivot(180.f); }});
)

onenter(rot_pivot_neg180,
    runClosedLoopTest({"rot_omega_pi_gyro", "rot_pivot_neg180", rotation_init_log, [] { pivot(-180.f); }});
)
