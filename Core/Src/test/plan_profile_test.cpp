#include "test/plan_profile_test.hpp"
#include "test/motor_id.hpp"
#include "common/etc.hpp"
#include "common/debug.hpp"

// PlanProfile（stepVelocity / stepAccel / vel2vel）の実機試験の共通環境。
// IMU校正 → planProfile.init()（目標値0・原点取り直し）→ ログ開始 → profile() → planProfile.stop() で停止
// → 停止待ち → ブレーキ → ログダンプ。ログは速度試験と同じ項目（目標/実測の速度・位置）。
// 閉ループへの切替（planProfile.start()）は走行開始時の1回だけで，区間の間ではPIも原点もリセットしない。
// 各区間は目標値で終わりを判定するので，vel2vel(0, d)で減速して止めることもできる。
static void plan_profile_tester_head(void) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    planProfile.init();

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用
    planProfile.start();
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

// 0→600mm/sを90mmで加速 → 600mm/sで180mm等速 → 600→300mm/sを90mmで減速 → 300→0mm/sを90mmで減速して停止
static void profile_vel2vel(void) {
    planProfile.vel2vel(600.f, 90.f);
    planProfile.vel2vel(600.f, 90.f);
    planProfile.stepVelocity(600.f, 90.f);
    planProfile.vel2vel(300.f, 90.f);
    planProfile.vel2vel(0.f, 90.f);
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

// ---- 回転（stepOmega / stepAlpha / omega2omega）----
// FF・Tiは並進700mm/sで同定した値なので，700mm/sまで加速してから旋回し，旋回後に減速して止める。
// 旋回は ω: 0→±430dps（2500dps/s, 37°）→ ±430dpsで90° → 0（37°）の計約164°。記録は約1.7s（ログ上限2.0s）

// omega2omegaで角速度を台形に変化させる。dir=+1で左旋回，-1で右旋回
static void profile_turn_omega2omega(float dir) {
    planProfile.vel2vel(700.f, 100.f);
    planProfile.omega2omega(dir * 430.f, dir * 37.f);
    planProfile.stepOmega(dir * 430.f, dir * 90.f);
    planProfile.omega2omega(0.f, dir * 37.f);
    planProfile.vel2vel(0.f, 100.f);
}

// 同じ旋回をstepAlpha（加速・減速）とstepOmega（等角速度）で行う
static void profile_turn_step_alpha(float dir) {
    planProfile.vel2vel(700.f, 100.f);
    planProfile.stepAlpha(dir * 2500.f, dir * 37.f);
    planProfile.stepOmega(dir * 430.f, dir * 90.f);
    planProfile.stepAlpha(-dir * 2500.f, dir * 37.f);   // 減速は目標角速度0で止まる
    planProfile.vel2vel(0.f, 100.f);
}

onenter(plan_turn_pos430,
    id_init_log_omega();
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_turn_omega2omega_pos430");
    logger.setIncludeTimestamp(false);
    plan_profile_tester_head();
    profile_turn_omega2omega(1.f);
    plan_profile_tester_tail();
)

onenter(plan_turn_neg430,
    id_init_log_omega();
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_turn_omega2omega_neg430");
    logger.setIncludeTimestamp(false);
    plan_profile_tester_head();
    profile_turn_omega2omega(-1.f);
    plan_profile_tester_tail();
)

onenter(plan_turn_step_alpha_pos430,
    id_init_log_omega();
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_turn_step_alpha_pos430");
    logger.setIncludeTimestamp(false);
    plan_profile_tester_head();
    profile_turn_step_alpha(1.f);
    plan_profile_tester_tail();
)

// ---- 並進の高速試験（吸引ファンON）----
// ファン20%で吸着させ，0→2000mm/s（180mm）→ 2000mm/sで360mm → 2000→0mm/s（180mm）の計720mm。
// 加速度は 2000^2/(2*180) ≈ 11,100mm/s^2（約1.13G）。所要約0.54s。停止の遅れも含め1m以上の直線を確保すること。
// 並進の同定（K_p, T_p1, FF）は約900mm/sまでの実測なので，2000mm/sでの追従は外挿（ログで確認する）。
namespace {
constexpr float FAST_FAN_DUTY = 0.20f;
constexpr uint32_t FAST_FAN_SPINUP_MS = 1000;   // ファンのスピンアップ待ち（吸着力が立ち上がるまで）
constexpr float FAST_MIN_BATTERY_V = 7.4f;      // [V] これ未満なら走らない（2000mm/sに必要な電圧余裕の確保）
constexpr float FAST_V_MAX = 2000.f;            // [mm/s]
constexpr float FAST_ACCEL_DIST = 180.f;        // [mm]
constexpr float FAST_CRUISE_DIST = 360.f;       // [mm]
constexpr float FAST_DECEL_DIST = 180.f;        // [mm]
}

// IMU校正はファンを回す前に行う（ファンの振動がジャイロのオフセット推定に乗らないように）。
// ファンをスピンアップさせてから原点を取り，ログを始めて走る。停止後にファンを止める
static void plan_fast_tester(void) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    fan.stop();

    float v0 = battery.voltage();
    if (v0 < FAST_MIN_BATTERY_V) {
        LOG("fast test not started: battery %.2f V < %.2f V\r\n", v0, FAST_MIN_BATTERY_V);
        for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s
            ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
            HAL_Delay(500);
        }
        ledBar16.set(0x0000);
        return;
    }

    imu.calibrate();
    HAL_Delay(1100);

    fan.setDuty(FAST_FAN_DUTY);
    HAL_Delay(FAST_FAN_SPINUP_MS);

    planProfile.init();

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間（ファンON）：オフセット推定用
    planProfile.start();

    planProfile.vel2vel(FAST_V_MAX, FAST_ACCEL_DIST);
    planProfile.stepVelocity(FAST_V_MAX, FAST_CRUISE_DIST);
    planProfile.vel2vel(0.f, FAST_DECEL_DIST);

    HAL_Delay(500);   // 停止の整定
    planProfile.stop();
    logger.stop();
    motorDriver.setBreak();
    fan.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(plan_fast_2000,
    id_init_log_velocity();
    logger.add(
        "fan_duty",
        etl::delegate<float()>::create<Fan, &Fan::getDuty>(fan)
    );
    logger.add(
        "target_accel_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetAccelX>(motorDriver)
    );
    logger.setDirName("plan_profile_x");
    logger.setFileName("plan_fast_2000_fan020");
    logger.setIncludeTimestamp(false);
    plan_fast_tester();
)
