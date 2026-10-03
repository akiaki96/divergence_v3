#include "test/wall_edge_test.hpp"
#include <cmath>
#include <cstdio>
#include "app/wall_edge_log.hpp"
#include "common/debug.hpp"
#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "test/closed_loop_test.hpp"

// 壁切れの試験（考え方は wall_edge_test.hpp）
namespace {
using config::maze::CELL_MM;

constexpr uint32_t N_CELLS = 4;
constexpr float INJECT_MM = 10.f;                              // [mm] inject で教える境界のずれ
constexpr float ACCEL = 1.0f * config::profile_limit::G;       // [mm/s^2] 加速・減速（700mm/sでも最後の境界の後で減速する）
constexpr float MAX_VELOCITY = 700.f;
constexpr uint32_t SETTLE_MS = 500;

// 区画境界 k（k = 0 … N_CELLS−1）の経路に沿った距離
constexpr float boundaryX(uint32_t k) {
    return (k + 1) * CELL_MM - config::maze::START_MM;
}
// 後端が N_CELLS 区画先の境界の線に来る車軸の位置
constexpr float STOP_X = N_CELLS * CELL_MM - config::maze::START_MM + config::mouse::BACK_TO_AXLE_MM;

constexpr float rampDistance(float v) {
    return v * v / (2.f * ACCEL);
}
// 最後の境界の壁切れ（境界の手前）を過ぎてから減速する
static_assert(STOP_X - rampDistance(MAX_VELOCITY) > boundaryX(N_CELLS - 1), "wall edge test decelerates too early");
static_assert(ACCEL <= config::profile_limit::MAX_ACCEL_X, "wall edge test accel exceeds the profile limit");

WallEdgeMode g_mode = WallEdgeMode::calib;
float g_velocity = 0.f;
char g_file_name[24];
char g_edge_name[32];

struct Result {
    float x;       // [mm] 止まったときの実測（補正込み）
    float shift;   // [mm] 補正の合計
};
Result g_result;

const char* modeName(WallEdgeMode m) {
    switch (m) {
    case WallEdgeMode::calib:  return "calib";
    case WallEdgeMode::verify: return "verify";
    default:                   return "inject";
    }
}

float irLeft() {
    return wall::read().value[wall::left];
}

float irRight() {
    return wall::read().value[wall::right];
}

void wall_edge_init_log() {
    logger.initLoggedVal();
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add("ir_l", Logger::Getter::create<&irLeft>());
    logger.add("ir_r", Logger::Getter::create<&irRight>());
    logger.add<&WallEdge::totalShift>("edge_shift", wallEdge);
    // 静止100ms ＋ 走行（加速・減速の分を足す）＋ 整定，2割の余裕
    float run_ms = (STOP_X + rampDistance(g_velocity)) / g_velocity * 1000.f;
    logger.setDuration(static_cast<uint32_t>((100.f + run_ms + SETTLE_MS) * 1.2f));
}

void wall_edge_profile() {
    // runClosedLoopTest が odometry / planProfile を0にそろえた後なので，境界の座標もここから
    bool correct = (g_mode != WallEdgeMode::calib);
    float inject = (g_mode == WallEdgeMode::inject) ? INJECT_MM : 0.f;
    wallEdge.reset();
    for (uint32_t k = 0; k < N_CELLS; ++k) wallEdge.expect(boundaryX(k) + inject);
    // calib は OFFSET_* がまだ合っていないので，最も近い境界に対応づける（窓を半区画にする）
    wallEdge.start(correct, correct ? config::wall_edge::WINDOW_MM : CELL_MM / 2.f);

    float v = g_velocity;
    float ramp = rampDistance(v);
    planProfile.straight(v, ramp);
    planProfile.straight(v, STOP_X - 2.f * ramp);
    planProfile.straight(0.f, ramp);

    planProfile.waitUntilIdle();
    HAL_Delay(SETTLE_MS);
    wallEdge.stop();
    g_result = {odometry.positionX(), wallEdge.totalShift()};
}

void printEvents() {
    float sum[WallEdge::SIDE_COUNT] = {};
    uint32_t count[WallEdge::SIDE_COUNT] = {};
    for (uint32_t i = 0; i < wallEdge.eventCount(); ++i) {
        const WallEdge::Event& e = wallEdge.event(i);
        const char* side = (e.side == WallEdge::left) ? "L" : "R";
        if (std::isnan(e.boundary)) {
            LOG("  edge %2lu %s x %6.1f  (no boundary)\r\n", static_cast<unsigned long>(i), side, e.x);
            continue;
        }
        float offset = e.x - e.boundary;
        LOG("  edge %2lu %s x %6.1f  boundary %6.1f  offset %+6.1f  shift %+5.1f  v %4.0f\r\n",
            static_cast<unsigned long>(i), side, e.x, e.boundary, offset, e.shift, e.velocity);
        sum[e.side] += offset;
        ++count[e.side];
    }
    for (uint8_t s = 0; s < WallEdge::SIDE_COUNT; ++s) {
        if (count[s] > 0) {
            LOG("  %s: %lu edges, mean offset %+.1f mm\r\n", (s == WallEdge::left) ? "left" : "right",
                static_cast<unsigned long>(count[s]), sum[s] / count[s]);
        }
    }
}
} // namespace

void runWallEdgeTest(WallEdgeMode mode, float velocity) {
    g_mode = mode;
    g_velocity = velocity;
    g_result = {0.f, 0.f};
    std::snprintf(g_file_name, sizeof(g_file_name), "%s_%.0f", modeName(mode), velocity);
    std::snprintf(g_edge_name, sizeof(g_edge_name), "%s_edges", g_file_name);
    LOG("wall edge %s: %lu cells at %.0f mm/s, stop at %.1f mm (back end on the boundary line)%s\r\n",
        modeName(mode), static_cast<unsigned long>(N_CELLS), velocity, STOP_X,
        (mode == WallEdgeMode::inject) ? ", boundaries shifted by INJECT_MM" : "");

    runClosedLoopTest({"wall_edge", g_file_name, wall_edge_init_log, wall_edge_profile, 0.f,
                       config::search::MIN_BATTERY_V, 0});

    LOG("wall edge %s result: %lu edges, total shift %+.1f mm, stopped at %.1f mm (corrected odometry)\r\n",
        modeName(mode), static_cast<unsigned long>(wallEdge.eventCount()), g_result.shift, g_result.x);
    printEvents();
    LOG("  measure delta = how far the back end stopped past the boundary line (+ forward)\r\n");
    wall_edge_log::dump("wall_edge", g_edge_name);
}
