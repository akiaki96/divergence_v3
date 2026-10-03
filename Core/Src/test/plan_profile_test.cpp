#include "test/plan_profile_test.hpp"
#include "test/closed_loop_test.hpp"
#include "test/motor_id.hpp"
#include "common/debug.hpp"

// PlanProfile（straight / turn / setVelocityX）の実機試験。手順はrunClosedLoopTest()を参照。
// 閉ループへの切替（motorDriver.switchToVelocityX()）は走行開始時の1回だけで，区間の間ではPIも原点もリセットしない。
// 各区間は目標値で終わりを判定するので，straight(0, d)で減速して止めることもできる。

// 500mm/sへステップして270mm走行
static void profile_step_velocity(void) {
    planProfile.setVelocityX(500.f);
    planProfile.straight(500.f, 270.f);
}

// 0→600mm/sを90mmで加速 → 600mm/sで180mm等速 → 600→300mm/sを90mmで減速 → 300→0mm/sを90mmで減速して停止
static void profile_straight(void) {
    planProfile.straight(600.f, 90.f);
    planProfile.straight(600.f, 90.f);
    planProfile.straight(600.f, 90.f);
    planProfile.straight(300.f, 90.f);
    planProfile.straight(0.f, 90.f);
}

onenter(plan_step_velocity,
    runClosedLoopTest({"plan_profile_x", "plan_step_velocity", id_init_log_velocity, profile_step_velocity});
)

onenter(plan_vel2vel,
    runClosedLoopTest({"plan_profile_x", "plan_vel2vel", id_init_log_velocity, profile_straight});
)

// ---- 回転（turn）----
// 700mm/sまで加速してから旋回し，旋回後に減速して止める（旧方式のω FF・Tiの同定条件に合わせたまま）。
// 旋回は ω: 0→±430dps（2500dps/s, 37°）→ ±430dpsで90° → 0（37°）の計約164°。記録は約1.7s（ログ上限2.0s）

// turnで角速度を台形に変化させる。dir=+1で左旋回，-1で右旋回
static void profile_turn(float dir) {
    planProfile.straight(700.f, 100.f);
    planProfile.turn(dir * 430.f, dir * 37.f);
    planProfile.turn(dir * 430.f, dir * 90.f);
    planProfile.turn(0.f, dir * 37.f);
    planProfile.straight(0.f, 100.f);
}

onenter(plan_turn_pos430,
    runClosedLoopTest({"plan_profile_x", "plan_turn_omega2omega_pos430_angle_pi", id_init_log_omega,
                       [] { profile_turn(1.f); }});
)

onenter(plan_turn_neg430,
    runClosedLoopTest({"plan_profile_x", "plan_turn_omega2omega_neg430_angle_pi", id_init_log_omega,
                       [] { profile_turn(-1.f); }});
)

// ---- 並進の高速試験（吸引ファンON）----
// ファンで吸着させて高速・高加速度で走る。並進の同定（K_p, T_p1, FF）は約900mm/s・ファンなしの実測なので，
// 高速域の追従は外挿（ログで確認する）。停止の遅れも含め，経路長＋0.3m以上の直線を確保すること。
namespace {
constexpr float FAST_FAN_DUTY = 0.20f;

// 台形（加速→定速→減速）の高速試験
struct FastProfile {
    float v_max;        // [mm/s]
    float accel_dist;   // [mm] 0→v_max
    float cruise_dist;  // [mm] v_maxで定速
    float decel_dist;   // [mm] v_max→0
};

// 台形の加速・減速がプロファイルの加速度の上限（config::profile_limit）内か。超えていればコンパイルエラーにする
// （積むときにも同じvalidateSegment()で検査される）
constexpr bool withinAccelLimit(const FastProfile& p) {
    using namespace config::profile_limit;
    return validateSegment(0.f, p.v_max, p.accel_dist, MAX_ACCEL_X, MAX_DECEL_X) == SegmentResult::ok &&
           validateSegment(p.v_max, 0.f, p.decel_dist, MAX_ACCEL_X, MAX_DECEL_X) == SegmentResult::ok;
}

// 0→2000mm/s（180mm, 約1.13G）→ 360mm → 0（180mm）。経路720mm，所要約0.54s
constexpr FastProfile FAST_2000 = {2000.f, 180.f, 360.f, 180.f};
static_assert(withinAccelLimit(FAST_2000), "FAST_2000 exceeds config::profile_limit");

constexpr float FAST_MIN_BATTERY_V = 7.4f;   // [V] これ未満なら走らない（高速に必要な電圧余裕の確保）
}

// 高速試験のログ：速度試験の項目に，ファンduty・目標加速度・速度PIの飽和フラグを加える（記録時間は id_init_log() の2.5s）
static void plan_fast_init_log(void) {
    id_init_log_velocity();
    logger.add<&Fan::getDuty>("fan_duty", fan);
    logger.add<&PlanProfile::getTargetAccelX>("target_accel_x", planProfile);
    logger.add<&MotorDriver::getVelocityXSaturated>("velocity_saturated", motorDriver);
}

static void run_fast_profile(const FastProfile& profile) {
    planProfile.straight(profile.v_max, profile.accel_dist);
    planProfile.straight(profile.v_max, profile.cruise_dist);
    planProfile.straight(0.f, profile.decel_dist);
}

onenter(plan_fast_2000,
    runClosedLoopTest({"plan_profile_x", "plan_fast_2000_fan020", plan_fast_init_log,
                       [] { run_fast_profile(FAST_2000); }, FAST_FAN_DUTY, FAST_MIN_BATTERY_V});
)

// ---- 低速のエンコーダ検証 ----
// 50mm/sで990mm（90mm×11）まっすぐ走り，エンコーダの距離と実際に進んだ距離（定規・迷路の区画で測る）を比べる。
// 低速なのでスリップはほぼ起きず，差はエンコーダの換算（車輪径・ギヤ比・分解能）の誤差になる。
//   実測距離 / 990mm = 真の車輪径 / WHEEL_RADIUS_MM×2 の比。左右の距離の差は左右の車輪径の差（向きはジャイロで保持）。
// ファンありでも同じ試験をする：吸着でタイヤが押しつぶされると実効半径が小さくなり，同じ回転で進む距離が変わる。
// 所要約21s。ログは20msごと（50Hz, 1mm/サンプル）に間引いて記録する（12列×約1050サンプル）
namespace {
constexpr float ENC_CHECK_VELOCITY = 50.f;        // [mm/s]
constexpr float ENC_CHECK_DISTANCE = 90.f * 11;   // [mm]
constexpr uint32_t ENC_CHECK_LOG_MS = 24000;      // [ms] 静止100ms＋990mm/50mm/s（19.8s）＋整定1.5s
}

static void encoder_check_init_log(void) {
    logger.initLoggedVal();
    logger.add<&Encoder::distance>("left_distance", encoderLeft);
    logger.add<&Encoder::distance>("right_distance", encoderRight);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::velocityX>("encoder_velocity_x", odometry);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&Imu::gyroZ>("gyro_z", imu);
    logger.add<&Battery::voltage>("battery", battery);
    logger.add<&Motor::getDuty>("Left Duty", motorLeft);
    logger.add<&Motor::getDuty>("Right Duty", motorRight);
    logger.add<&Fan::getDuty>("fan_duty", fan);
    logger.setDuration(ENC_CHECK_LOG_MS);
}

// 走り終えたら停止指令を出して1000ms整定させ，ログを止める前にエンコーダの距離を表示する
static void encoder_check_profile(void) {
    planProfile.setVelocityX(ENC_CHECK_VELOCITY);
    planProfile.straight(ENC_CHECK_VELOCITY, ENC_CHECK_DISTANCE);
    planProfile.waitUntilIdle();
    planProfile.stop();
    HAL_Delay(1000);   // 停止の整定

    float left = encoderLeft.distance();
    float right = encoderRight.distance();
    LOG("encoder check (fan %.2f): target %.1f mm, left %.1f mm, right %.1f mm, average %.1f mm, left-right %.1f mm\r\n",
        fan.getDuty(), ENC_CHECK_DISTANCE, left, right, (left + right) / 2.f, left - right);
}

onenter(plan_encoder_check,
    runClosedLoopTest({"plan_profile_x", "encoder_check_50", encoder_check_init_log,
                       encoder_check_profile, 0.f, 0.f, 0});
)

onenter(plan_encoder_check_fan,
    runClosedLoopTest({"plan_profile_x", "encoder_check_50_fan020", encoder_check_init_log,
                       encoder_check_profile, FAST_FAN_DUTY, 0.f, 0});
)
