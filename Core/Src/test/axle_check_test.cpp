#include "test/axle_check_test.hpp"
#include "test/closed_loop_test.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"
#include <cstdio>

// BACK_TO_AXLE_MMの確認（考え方はaxle_check_test.hpp）。台形で BACK_TO_AXLE_MM + 90·n 走って止まる
namespace {
constexpr float HALF_CELL_MM = 90.f;
constexpr float VELOCITY = 200.f;                                  // [mm/s] 滑らない低速
constexpr float ACCEL = 0.25f * config::profile_limit::G;          // [mm/s^2] 加速・減速
constexpr float RAMP_MM = VELOCITY * VELOCITY / (2.f * ACCEL);     // [mm] 加速・減速それぞれ（約8mm）
constexpr uint32_t SETTLE_MS = 500;
constexpr uint32_t LOG_DECIMATION = 4;   // [tick] 10列×2400サンプル＝9.6s（n=8でも収まる）

constexpr float checkDistance(uint32_t n) {
    return config::mouse::BACK_TO_AXLE_MM + HALF_CELL_MM * n;
}
static_assert(checkDistance(1) > 2.f * RAMP_MM, "axle check is shorter than its ramps");

uint32_t g_n = 1;
char g_file_name[16];

struct Result {
    float left, right, angle;   // [mm] [mm] [deg]
};
Result g_result;

void axle_check_init_log() {
    logger.initLoggedVal();
    logger.add<&Encoder::distance>("left_distance", encoderLeft);
    logger.add<&Encoder::distance>("right_distance", encoderRight);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::velocityX>("encoder_velocity_x", odometry);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&Battery::voltage>("battery", battery);
    logger.add<&Motor::getDuty>("Left Duty", motorLeft);
    logger.setDecimation(LOG_DECIMATION);
}

void axle_check_profile() {
    float d = checkDistance(g_n);
    planProfile.straight(VELOCITY, RAMP_MM);
    planProfile.straight(VELOCITY, d - 2.f * RAMP_MM);
    planProfile.straight(0.f, RAMP_MM);

    // 止まって整定してから読む（runClosedLoopTestのsettleは0にしてある）
    planProfile.waitUntilIdle();
    HAL_Delay(SETTLE_MS);
    g_result = {encoderLeft.distance(), encoderRight.distance(), odometry.angle()};
}
} // namespace

void runAxleCheck(uint32_t n) {
    float d = checkDistance(n);
    LOG("axle check n=%lu: drive %.1f mm (BACK_TO_AXLE_MM %.1f + 90 x %lu) at %.0f mm/s\r\n",
        static_cast<unsigned long>(n), d, config::mouse::BACK_TO_AXLE_MM, static_cast<unsigned long>(n), VELOCITY);
    LOG("  place the axle (wheel centers) on a floor line; the back end should stop on the line %lu mm ahead\r\n",
        static_cast<unsigned long>(HALF_CELL_MM * n));

    g_n = n;
    g_result = {0.f, 0.f, 0.f};
    std::snprintf(g_file_name, sizeof(g_file_name), "n%lu", static_cast<unsigned long>(n));

    runClosedLoopTest({"axle_check", g_file_name, axle_check_init_log, axle_check_profile, 0.f, 0.f, 0});

    float average = (g_result.left + g_result.right) / 2.f;
    LOG("axle check n=%lu result: encoder average %.1f mm (target %.1f, error %+.1f), left %.1f, right %.1f, angle %+.2f deg\r\n",
        static_cast<unsigned long>(n), average, d, average - d, g_result.left, g_result.right, g_result.angle);
    LOG("  measure delta = how far the back end stopped past the line (+ forward): true BACK_TO_AXLE_MM = %.1f - delta\r\n",
        config::mouse::BACK_TO_AXLE_MM);
}
