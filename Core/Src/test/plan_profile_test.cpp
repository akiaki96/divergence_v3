#include "test/plan_profile_test.hpp"
#include "test/motor_id.hpp"
#include "common/etc.hpp"

// PlanProfile（stepVelocity / stepAccel / vel2vel）の実機試験の共通環境。
// IMU校正 → planProfile.init()（目標値0・原点取り直し）→ ログ開始 → profile() → planProfile.stop() で停止
// → 停止待ち → ブレーキ → ログダンプ。ログは速度試験と同じ項目（目標/実測の速度・位置）。
// 各プロファイル関数は距離到達まで戻らないので，profile()内では必ず到達できる呼び方だけを使う
// （目標速度0のstepVelocityや，v2=0への減速はループが終わらないおそれがある）。停止は最後のstop()で行う。
static void plan_profile_tester_head(void) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    planProfile.init();

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用
}

static void plan_profile_tester_tail(void) {
    HAL_Delay(500);   // 停止の整定
    planProfile.stop();
    logger.stop();
    motorDriver.setBreak();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

// 500mm/sへステップして270mm走行
static void profile_step_velocity(void) {
    planProfile.stepVelocity(500.f, 270.f);
}

// 2000mm/s^2で90mm加速（約600mm/s）→ 600mm/sで90mm等速
static void profile_step_accel(void) {
    planProfile.stepAccel(2000.f, 90.f);
    planProfile.stepVelocity(600.f, 90.f);
}

// 0→600mm/sを90mmで加速 → 600→300mm/sを90mmで減速 → 300mm/sで90mm等速
static void profile_vel2vel(void) {
    planProfile.vel2vel(0.f, 600.f, 90.f);
    planProfile.vel2vel(600.f, 600.f, 90.f);
    planProfile.stepVelocity(600.f, 90.f);
    planProfile.vel2vel(600.f, 300.f, 90.f);
    planProfile.vel2vel(300.f, 0.f, 90.f);
}

onenter(plan_step_velocity,
    id_init_log_velocity();
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_step_velocity");
    logger.setIncludeTimestamp(false);

    plan_profile_tester_head();
    profile_step_velocity();
    plan_profile_tester_tail();
)

onenter(plan_step_accel,
    id_init_log_velocity();
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_step_accel");
    logger.setIncludeTimestamp(false);
    plan_profile_tester_head();
    profile_step_accel();
    plan_profile_tester_tail();
)

onenter(plan_vel2vel,
    id_init_log_velocity();
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_vel2vel");
    logger.setIncludeTimestamp(false);
    plan_profile_tester_head();
    profile_vel2vel();
    plan_profile_tester_tail();
)
