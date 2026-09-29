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
// ファンで吸着させて高速・高加速度で走る。並進の同定（K_p, T_p1, FF）は約900mm/s・ファンなしの実測なので，
// 高速域の追従は外挿（ログで確認する）。停止の遅れも含め，経路長＋0.3m以上の直線を確保すること。
namespace {
constexpr float FAST_FAN_DUTY = 0.20f;
constexpr uint32_t FAST_FAN_SPINUP_MS = 1000;   // ファンのスピンアップ待ち（吸着力が立ち上がるまで）

// 台形（加速→定速→減速）の高速試験
struct FastProfile {
    float v_max;        // [mm/s]
    float accel_dist;   // [mm] 0→v_max
    float cruise_dist;  // [mm] v_maxで定速
    float decel_dist;   // [mm] v_max→0
};

// 台形の加速・減速がプロファイルの加速度の上限（config::profile_limit）内か。超えていればコンパイルエラーにする
constexpr bool withinAccelLimit(const FastProfile& p) {
    return config::profile_limit::withinAccelLimit(0.f, p.v_max, p.accel_dist) &&
           config::profile_limit::withinAccelLimit(p.v_max, 0.f, p.decel_dist);
}

// 0→2000mm/s（180mm, 約1.13G）→ 360mm → 0（180mm）。経路720mm，所要約0.54s
constexpr FastProfile FAST_2000 = {2000.f, 180.f, 360.f, 180.f};
static_assert(withinAccelLimit(FAST_2000), "FAST_2000 exceeds config::profile_limit");

constexpr float FAST_MIN_BATTERY_V = 7.4f;   // [V] これ未満なら走らない（高速に必要な電圧余裕の確保）
}

// 高速試験のログ：速度試験の項目に，ファンduty・目標加速度・速度PIの飽和フラグを加える（13列＝約1.8s）
static void plan_fast_init_log(const char* file_name) {
    id_init_log_velocity();
    logger.add(
        "fan_duty",
        etl::delegate<float()>::create<Fan, &Fan::getDuty>(fan)
    );
    logger.add(
        "target_accel_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetAccelX>(motorDriver)
    );
    logger.add(
        "velocity_saturated",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXSaturated>(motorDriver)
    );
    logger.setDirName("plan_profile_x");
    logger.setFileName(file_name);
    logger.setIncludeTimestamp(false);
}

// ファンONの高速試験の共通環境：電池電圧の確認 → IMU校正（ファンの振動がジャイロのオフセット推定に
// 乗らないようにファンを回す前に行う）→ ファンのスピンアップ → 原点取り → ログ開始 → profile() → 停止 → ファン停止
static void plan_fan_tester(void (*profile)(void), float min_battery_v) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    fan.stop();

    float v0 = battery.voltage();
    if (v0 < min_battery_v) {
        LOG("fast test not started: battery %.2f V < %.2f V\r\n", v0, min_battery_v);
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

    profile();

    HAL_Delay(500);   // 停止の整定（目標から遅れた分を位置Pで追いつく時間を含む）
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

static void run_fast_profile(const FastProfile& profile) {
    planProfile.vel2vel(profile.v_max, profile.accel_dist);
    planProfile.stepVelocity(profile.v_max, profile.cruise_dist);
    planProfile.vel2vel(0.f, profile.decel_dist);
}

onenter(plan_fast_2000,
    plan_fast_init_log("plan_fast_2000_fan020");
    plan_fan_tester([] { run_fast_profile(FAST_2000); }, FAST_MIN_BATTERY_V);
)
