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
// 区画の大きさと置いたときの車軸の位置（実測の BACK_TO_AXLE_MM を含む）は探索と共有する
using config::maze::CELL_MM;
using config::maze::START_MM;

constexpr float RUNUP_ACCEL = 0.5f * config::profile_limit::G;   // [mm/s^2] 入口までの加速
constexpr float STOP_DECEL_LIMIT = config::profile_limit::MAX_DECEL_X;
constexpr uint32_t SETTLE_MS = 500;            // 止まってから最終位置を読むまで
constexpr uint32_t LOG_DECIMATION = 2;         // [tick] 15列×1600サンプル＝3.2s

// 置いた位置から入口の基準点まで [mm]
constexpr float runupDistance(slalom::Anchor entry) {
    return (entry == slalom::Anchor::edge) ? (CELL_MM - START_MM) : (CELL_MM + CELL_MM / 2.f - START_MM);
}

// 出口の基準点から止まる位置（次の区画中央）まで [mm]
constexpr float stopDistance(slalom::Anchor exit) {
    return (exit == slalom::Anchor::edge) ? (CELL_MM / 2.f) : CELL_MM;
}

constexpr float accelDistance(float speed) {
    return speed * speed / (2.f * RUNUP_ACCEL);
}

// 試験中のパラメータ（runClosedLoopTest()の関数ポインタは引数を持てないので，ここで受け渡す）
const slalom::Param* g_param = nullptr;
slalom::TurnDir g_dir = slalom::TurnDir::left;
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
    logger.setDecimation(LOG_DECIMATION);
}

void slalom_profile() {
    const slalom::Param& p = *g_param;
    float accel = accelDistance(p.speed);
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
bool checkRunnable(const slalom::Param& p) {
    using namespace config::profile_limit;
    if (p.entry == slalom::Anchor::diagonal || p.exit == slalom::Anchor::diagonal) {
        LOG("slalom test: %s has a diagonal entry/exit (not supported)\r\n", p.name);
        return false;
    }
    float accel = accelDistance(p.speed);
    float runup = runupDistance(p.entry);
    if (runup - accel <= 0.f) {
        LOG("slalom test: run-up %.1f mm is shorter than the acceleration %.1f mm\r\n", runup, accel);
        return false;
    }
    SegmentResult r = slalom::validate(p);
    if (r != SegmentResult::ok) {
        slalom::Shape s = slalom::shapeOf(p);
        LOG("slalom test: %s rejected (%s): ramp %.1f deg, cruise %.1f deg, alpha %.0f dps/s (limit %.0f)\r\n",
            p.name, slalom::resultName(r), s.ramp_angle, s.cruise_angle, p.alpha, MAX_ALPHA);
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
    slalom::Shape s = slalom::shapeOf(p);
    LOG("slalom test %s %s: speed %.0f mm/s, omega %.0f dps, alpha %.0f dps/s, pre %.1f mm, post %.1f mm\r\n",
        p.name, dir_name, p.speed, p.omega_max, p.alpha, p.pre_offset, p.post_offset);
    LOG("  ramp %.2f deg x2, cruise %.2f deg, turn %.1f mm, pre..post %.1f mm\r\n",
        s.ramp_angle, s.cruise_angle, slalom::turnDistance(p), slalom::totalDistance(p));

    if (!checkRunnable(p)) {
        blinkRefused();
        return;
    }

    g_param = &p;
    g_dir = dir;
    g_result = {0.f, 0.f};
    std::snprintf(g_file_name, sizeof(g_file_name), "%s_%s", p.name, dir_name);

    runClosedLoopTest({"slalom", g_file_name, slalom_init_log, slalom_profile, 0.f, 0.f, 0});

    // 目標：最終角度は±angle，走行距離は 入口まで＋スラローム＋止まるまで
    float target_angle = (dir == slalom::TurnDir::left) ? p.angle : -p.angle;
    float target_distance = runupDistance(p.entry) + slalom::totalDistance(p) + stopDistance(p.exit);
    LOG("slalom result %s %s: angle %.2f deg (target %.1f, error %+.2f), distance %.1f mm (target %.1f, error %+.1f)\r\n",
        p.name, dir_name, g_result.angle, target_angle, g_result.angle - target_angle,
        g_result.distance, target_distance, g_result.distance - target_distance);
}
