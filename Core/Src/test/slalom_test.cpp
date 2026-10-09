#include "test/slalom_test.hpp"
#include "test/closed_loop_test.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"
#include <cstdio>

// スラロームの実機試験。並進の目標速度をspeedまで上げてからslalom::push()で旋回し，次の区画中央で止める。
//
// 置き方（クラシック迷路，1区画180mm）：機体の後端を区画の後壁に当て，区画の中心線に沿って前へ向ける。
//   入口がedge（小回り90°）  … その区画の前の境界が入口の基準点
//   入口がcenter（大回り・180°）… 1つ先の区画の中央が入口の基準点
// 出口の基準点からは，edgeなら半区画，centerなら1区画進んで（次の区画中央で）止まる。
// 斜め（diagonal）の入口・出口は未対応。
//
// 結果はログ（slalom/<name>_<left|right>）と，走行後に表示する最終角度・走行距離の目標との差で見る
namespace {
// 区画の大きさと置いたときの車軸の位置は探索と共有する（config::maze）。config::mouse::BACK_TO_AXLE_MMが
// ずれていると入口の位置がずれ，その分をpre_offsetの調整で吸収してしまう（確かめ方は test/axle_check_test.hpp）
using config::maze::CELL_MM;
using config::maze::START_MM;

constexpr float RUNUP_ACCEL = 0.5f * config::profile_limit::G;   // [mm/s^2] 入口までの加速（足りれば）
constexpr float RUNUP_CRUISE_FRAC = 0.3f;   // 助走のうち，入口の前に等速で走る割合（加速を上げるときに残す分）
constexpr float STOP_DECEL_LIMIT = config::profile_limit::MAX_DECEL_X;
constexpr uint32_t SETTLE_MS = 500;            // 止まってから最終位置を読むまで

// 置いた位置から入口の基準点まで [mm]
constexpr float runupDistance(slalom::Anchor entry) {
    return (entry == slalom::Anchor::edge) ? (CELL_MM - START_MM) : (CELL_MM + CELL_MM / 2.f - START_MM);
}

// 出口の基準点から止まる位置（次の区画中央）まで [mm]
constexpr float stopDistance(slalom::Anchor exit) {
    return (exit == slalom::Anchor::edge) ? (CELL_MM / 2.f) : CELL_MM;
}

// 入口までの加速度 [mm/s^2]：RUNUP_ACCEL で助走に収まればそれ，収まらない（高速）なら助走の RUNUP_CRUISE_FRAC を
// 等速に残せる最小の加速度。config::profile_limit::MAX_ACCEL_X を超えるなら走らない（checkRunnable）
constexpr float runupAccel(const slalom::Param& p) {
    float need = p.speed * p.speed / (2.f * (1.f - RUNUP_CRUISE_FRAC) * runupDistance(p.entry));
    return (need > RUNUP_ACCEL) ? need : RUNUP_ACCEL;
}

constexpr float accelDistance(const slalom::Param& p) {
    return p.speed * p.speed / (2.f * runupAccel(p));
}

// 走行時間 [ms]：静止100ms・助走（加速＋等速）・入口〜出口・停止・整定。低速ほど長い（200mm/s の L90 で約5s）
constexpr float runMs(const slalom::Param& p, slalom::TurnDir dir) {
    float accel = accelDistance(p);
    float s = (2.f * accel + (runupDistance(p.entry) - accel) + slalom::totalDistance(p, dir) + 2.f * stopDistance(p.exit)) / p.speed;
    return 100.f + s * 1000.f + SETTLE_MS;
}

// 走行がまるごとログに収まる長さ [ms]（2割の余裕）。ログが一杯になると記録が止まり，止まった位置が残らない
uint32_t logMs(const slalom::Param& p, slalom::TurnDir dir) {
    return static_cast<uint32_t>(runMs(p, dir) * 1.2f);
}

// 試験中のパラメータ（runClosedLoopTest()の関数ポインタは引数を持てないので，ここで受け渡す）
const slalom::Param* g_param = nullptr;
slalom::TurnDir g_dir = slalom::TurnDir::left;
uint32_t g_log_ms = 0;
char g_file_name[32];

struct Result {
    float angle;      // [deg] 最終角度（odometry）
    float distance;   // [mm] 並進の走行距離（エンコーダ平均）
};
Result g_result;

void blinkRefused() {
    for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s（runClosedLoopTestと同じ）
        ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
        HAL_Delay(500);
    }
    ledBar16.set(0x0000);
}

void slalom_init_log() {
    logger.initLoggedVal();
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&PlanProfile::getTargetOmega>("target_omega", planProfile);
    logger.add<&Imu::gyroZ>("gyro_z", imu);
    logger.add<&Odometry::encoderOmega>("encoder_omega", odometry);
    logger.add<&MotorDriver::getOmegaCommand>("omega_cmd", motorDriver);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&Odometry::velocityX>("encoder_velocity_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&Motor::getDuty>("Left Duty", motorLeft);
    logger.add<&Motor::getDuty>("Right Duty", motorRight);
    logger.add<&Battery::voltage>("battery", battery);
    logger.add<&Imu::accelX>("accel_x", imu);
    logger.add<&Fan::getDuty>("fan_duty", fan);
    logger.setDuration(g_log_ms);
}

void slalom_profile() {
    const slalom::Param& p = *g_param;
    float accel = accelDistance(p);
    planProfile.straight(p.speed, accel);
    planProfile.straight(p.speed, runupDistance(p.entry) - accel);
    slalom::push(planProfile, p, g_dir);
    planProfile.straight(0.f, stopDistance(p.exit));

    // 止まって整定してから最終位置を読む（runClosedLoopTestのsettleは0にしてある）
    planProfile.waitUntilIdle();
    HAL_Delay(SETTLE_MS);
    g_result = {odometry.angle(), odometry.positionX()};
}

// 走らせる前の検査。だめなら理由を表示してfalse
bool checkRunnable(const slalom::Param& p, slalom::TurnDir dir) {
    using namespace config::profile_limit;
    if (p.entry == slalom::Anchor::diagonal || p.exit == slalom::Anchor::diagonal) {
        LOG("slalom test: %s has a diagonal entry/exit (not supported)\r\n", p.name);
        return false;
    }
    float accel = accelDistance(p);
    float runup = runupDistance(p.entry);
    if (runup - accel <= 0.f) {
        LOG("slalom test: run-up %.1f mm is shorter than the acceleration %.1f mm\r\n", runup, accel);
        return false;
    }
    if (runupAccel(p) > MAX_ACCEL_X) {
        LOG("slalom test: run-up accel %.0f mm/s^2 for %.0f mm/s exceeds MAX_ACCEL_X %.0f\r\n",
            runupAccel(p), p.speed, MAX_ACCEL_X);
        return false;
    }
    SegmentResult r = slalom::validate(p, dir);
    if (r != SegmentResult::ok) {
        slalom::Shape s = slalom::shapeOf(p, dir);
        LOG("slalom test: %s rejected (%s): ramp %.1f deg, cruise %.1f deg, alpha %.0f dps/s (limit %.0f)\r\n",
            p.name, slalom::resultName(r), s.ramp_angle, s.cruise_angle, p.motion(dir).alpha, MAX_ALPHA);
        return false;
    }
    if (validateSegment(p.speed, 0.f, stopDistance(p.exit), MAX_ACCEL_X, STOP_DECEL_LIMIT) != SegmentResult::ok) {
        LOG("slalom test: cannot stop from %.0f mm/s in %.1f mm\r\n", p.speed, stopDistance(p.exit));
        return false;
    }
    return true;
}
} // namespace

void runSlalomTest(const slalom::Param& p, slalom::TurnDir dir) {
    const char* dir_name = (dir == slalom::TurnDir::left) ? "left" : "right";
    const slalom::Motion& m = p.motion(dir);
    slalom::Shape s = slalom::shapeOf(p, dir);
    LOG("slalom test %s %s: speed %.0f mm/s, omega %.0f dps, alpha %.0f dps/s, pre %.1f mm, post %.1f mm, fan %s\r\n",
        p.name, dir_name, p.speed, m.omega_max, m.alpha, m.pre_offset, m.post_offset, p.fan ? "on" : "off");
    LOG("  ramp %.2f deg x2, cruise %.2f deg, turn %.1f mm, pre..post %.1f mm\r\n",
        s.ramp_angle, s.cruise_angle, slalom::turnDistance(p, dir), slalom::totalDistance(p, dir));

    if (!checkRunnable(p, dir)) {
        blinkRefused();
        return;
    }

    g_param = &p;
    g_dir = dir;
    g_log_ms = logMs(p, dir);
    LOG("  run about %.1f s, log %.1f s, run-up accel %.2f G\r\n", runMs(p, dir) / 1000.f, g_log_ms / 1000.f,
        runupAccel(p) / config::profile_limit::G);
    g_result = {0.f, 0.f};
    std::snprintf(g_file_name, sizeof(g_file_name), "%s_%s", p.name, dir_name);

    // ファンONで設計したパラメータはファンを回して走る（滑りがファンの有無で変わるため，設計と同じ条件にする）
    float fan_duty = p.fan ? config::fan::RUN_DUTY : 0.f;
    runClosedLoopTest({"slalom", g_file_name, slalom_init_log, slalom_profile, fan_duty, 0.f, 0});

    // 目標：最終角度は±angle，走行距離は 入口まで＋スラローム＋止まるまで
    float target_angle = (dir == slalom::TurnDir::left) ? p.angle : -p.angle;
    float target_distance = runupDistance(p.entry) + slalom::totalDistance(p, dir) + stopDistance(p.exit);
    LOG("slalom result %s %s: angle %.2f deg (target %.1f, error %+.2f), distance %.1f mm (target %.1f, error %+.1f)\r\n",
        p.name, dir_name, g_result.angle, target_angle, g_result.angle - target_angle,
        g_result.distance, target_distance, g_result.distance - target_distance);
}
