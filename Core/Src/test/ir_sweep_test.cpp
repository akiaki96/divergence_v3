#include "test/ir_sweep_test.hpp"
#include "test/closed_loop_test.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"
#include <cstdio>

// 前の壁への往復（考え方はir_sweep_test.hpp）
namespace {
constexpr float CELL_MM = 180.f;
constexpr float WALL_HALF_MM = 6.f;
constexpr float MIN_GAP_MM = 5.f;                                   // [mm] 前端と壁の最小のすき間
constexpr float VELOCITY = 40.f;                                    // [mm/s] 低速（1サンプル間の移動が小さい）
constexpr float ACCEL = 0.1f * config::profile_limit::G;            // [mm/s^2]
constexpr float RAMP_MM = VELOCITY * VELOCITY / (2.f * ACCEL);      // [mm] 加速・減速それぞれ（約0.8mm）
constexpr uint32_t HOLD_MS = 1000;      // 一番近い所で止まる時間（静止時のノイズを見る）
constexpr uint32_t SETTLE_MS = 500;     // 戻った後
constexpr uint32_t LOG_COLUMNS = 10;    // Global_time込み（ir_sweep_init_log()）
// メニューで決定してから走り出すまでの待ち。前の壁が近いと前のセンサーの値が大きく，前のセンサーを手でふさぐ
// メニューの決定ができないので，壁から離して決定し，この間に置く（LEDバーが端から消えていく）
constexpr uint32_t START_COUNTDOWN_MS = 3000;

// 置いたときの車軸から前の壁の面まで [mm]
constexpr float startAxleToWall(uint32_t cells) {
    return CELL_MM * cells - WALL_HALF_MM - (WALL_HALF_MM + config::mouse::BACK_TO_AXLE_MM);
}

constexpr float closestAxleToWall() {
    return config::mouse::FRONT_TO_AXLE_MM + MIN_GAP_MM;
}

constexpr float travel(uint32_t cells) {
    return startAxleToWall(cells) - closestAxleToWall();
}
static_assert(travel(1) > 2.f * RAMP_MM, "the front wall one cell ahead is too close for the sweep");

// 走行時間（静止100ms・往復・止まる時間・整定）から，ログに収まる間引きを決める（2割の余裕）
uint32_t logDecimation(uint32_t cells) {
    float ms = 100.f + 2.f * (travel(cells) / VELOCITY * 1000.f + 2.f * VELOCITY / ACCEL * 1000.f) + HOLD_MS + SETTLE_MS;
    uint32_t samples = Logger::MAX_BUFFER_SIZE / LOG_COLUMNS;
    return static_cast<uint32_t>(ms * 1.2f / samples) + 1;
}

float g_travel = 0.f;
uint32_t g_decimation = 1;
char g_file_name[16];

void ir_sweep_init_log() {
    logger.initLoggedVal();
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&Battery::voltage>("battery", battery);
    // 変数名は読み込んでいるピンと食い違っている可能性がある（feature/search の wall_sensor.hpp 参照）ので，
    // 変数名のまま記録し，どれが前のセンサーかは tools/fit_ir.py がデータから判断する
    logger.add<&IrSensor::value>("ir_var_L", irL);
    logger.add<&IrSensor::value>("ir_var_FL", irFL);
    logger.add<&IrSensor::value>("ir_var_FR", irFR);
    logger.add<&IrSensor::value>("ir_var_R", irR);
    logger.setDecimation(g_decimation);
}

void countdown() {
    constexpr int STEPS = 8;
    for (int i = STEPS; i > 0; --i) {
        ledBar16.set(static_cast<uint16_t>((1u << (2 * i)) - 1));   // 点いているLEDが減っていく
        HAL_Delay(START_COUNTDOWN_MS / STEPS);
    }
    ledBar16.set(0x0000);
}

void ir_sweep_profile() {
    float d = g_travel;
    planProfile.straight(VELOCITY, RAMP_MM);
    planProfile.straight(VELOCITY, d - 2.f * RAMP_MM);
    planProfile.straight(0.f, RAMP_MM);
    planProfile.waitUntilIdle();
    HAL_Delay(HOLD_MS);
    planProfile.straight(-VELOCITY, -RAMP_MM);
    planProfile.straight(-VELOCITY, -(d - 2.f * RAMP_MM));
    planProfile.straight(0.f, -RAMP_MM);
}
} // namespace

void runIrFrontSweep(uint32_t cells) {
    g_travel = travel(cells);
    g_decimation = logDecimation(cells);
    std::snprintf(g_file_name, sizeof(g_file_name), "front_%lucell", static_cast<unsigned long>(cells));
    LOG("IR front sweep: front wall %lu cell(s) ahead, axle to wall %.1f -> %.1f mm and back at %.0f mm/s, "
        "log every %lu ms\r\n",
        static_cast<unsigned long>(cells), startAxleToWall(cells), closestAxleToWall(), VELOCITY,
        static_cast<unsigned long>(g_decimation));
    LOG("  place the back end against the back wall (BACK_TO_AXLE_MM %.1f, FRONT_TO_AXLE_MM %.1f)\r\n",
        config::mouse::BACK_TO_AXLE_MM, config::mouse::FRONT_TO_AXLE_MM);

    LOG("  starting in %.0f s: place the robot now\r\n", START_COUNTDOWN_MS / 1000.f);
    countdown();
    runClosedLoopTest({"ir_sweep", g_file_name, ir_sweep_init_log, ir_sweep_profile, 0.f, 0.f, SETTLE_MS});

    LOG("IR front sweep done: encoder %.1f mm at the end (should be about 0), angle %+.2f deg\r\n",
        odometry.positionX(), odometry.angle());
}
