#include "test/front_check_test.hpp"
#include <algorithm>
#include "common/debug.hpp"
#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "test/closed_loop_test.hpp"

// 前壁の閾値の確認（考え方は front_check_test.hpp）
namespace {
constexpr float READ_X = config::maze::CELL_MM - config::maze::START_MM - config::search::READ_LEAD_MM;   // [mm]
constexpr float VELOCITY = 200.f;                                  // [mm/s]
constexpr float ACCEL = 0.25f * config::profile_limit::G;          // [mm/s^2]
constexpr float RAMP_MM = VELOCITY * VELOCITY / (2.f * ACCEL);
constexpr uint32_t SETTLE_MS = 300;
constexpr uint32_t SAMPLE_COUNT = 500;                             // 1ms ごと
static_assert(READ_X > 2.f * RAMP_MM, "front check is shorter than its ramps");

enum Sensor : uint8_t { FL, FR, SENSOR_COUNT };
constexpr const char* SENSOR_NAME[SENSOR_COUNT] = {"FL", "FR"};
constexpr int16_t THRESH[SENSOR_COUNT] = {config::wall::THRESH_FRONT_LEFT, config::wall::THRESH_FRONT_RIGHT};

struct Stats {
    uint32_t count = 0;
    int32_t min = INT16_MAX, max = INT16_MIN;
    float sum = 0.f;
    void add(int16_t v) {
        ++count;
        min = std::min<int32_t>(min, v);
        max = std::max<int32_t>(max, v);
        sum += v;
    }
    float mean() const {
        return count > 0 ? sum / count : 0.f;
    }
};

// 測った値の貯め（場面 × センサー）と，今の閾値での判定の誤り
struct Totals {
    Stats s[2][SENSOR_COUNT];
    uint32_t runs[2] = {};
    uint32_t wrong[2] = {};   // wall: 前壁なしと読んだ回数（読み落とし），no_wall: 前壁ありと読んだ回数
    uint32_t samples[2] = {};
};
Totals g_totals;

FrontCase g_case = FrontCase::wall;
Stats g_run[SENSOR_COUNT];
uint32_t g_run_wrong = 0;

float irFrontLeft() {
    return wall::read().value[wall::front_left];
}

float irFrontRight() {
    return wall::read().value[wall::front_right];
}

void front_check_init_log() {
    logger.initLoggedVal();
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add("ir_fl", Logger::Getter::create<&irFrontLeft>());
    logger.add("ir_fr", Logger::Getter::create<&irFrontRight>());
    float run_ms = (READ_X + RAMP_MM) / VELOCITY * 1000.f;
    logger.setDuration(static_cast<uint32_t>((100.f + run_ms + SETTLE_MS + SAMPLE_COUNT) * 1.2f));
}

void front_check_profile() {
    planProfile.straight(VELOCITY, RAMP_MM);
    planProfile.straight(VELOCITY, READ_X - 2.f * RAMP_MM);
    planProfile.straight(0.f, RAMP_MM);
    planProfile.waitUntilIdle();
    HAL_Delay(SETTLE_MS);

    // 止まって位置を保っている間に読む（閉ループのまま）
    uint8_t c = static_cast<uint8_t>(g_case);
    bool expect_wall = (g_case == FrontCase::wall);
    for (uint32_t i = 0; i < SAMPLE_COUNT; ++i) {
        wall::Snapshot s = wall::read();
        int16_t v[SENSOR_COUNT] = {s.value[wall::front_left], s.value[wall::front_right]};
        for (uint8_t k = 0; k < SENSOR_COUNT; ++k) {
            g_run[k].add(v[k]);
            g_totals.s[c][k].add(v[k]);
        }
        if (wall::hasFront(s) != expect_wall) {
            ++g_run_wrong;
            ++g_totals.wrong[c];
        }
        ++g_totals.samples[c];
        HAL_Delay(1);
    }
    ++g_totals.runs[c];
}

const char* caseName(FrontCase c) {
    return (c == FrontCase::wall) ? "wall" : "no wall";
}
} // namespace

void showFrontCheck() {
    const auto& w = g_totals.s[static_cast<uint8_t>(FrontCase::wall)];
    const auto& n = g_totals.s[static_cast<uint8_t>(FrontCase::no_wall)];
    LOG("front check: %lu wall runs, %lu no-wall runs (read %.0f mm from the start, %.0f mm before the boundary)\r\n",
        static_cast<unsigned long>(g_totals.runs[0]), static_cast<unsigned long>(g_totals.runs[1]), READ_X,
        config::search::READ_LEAD_MM);
    for (uint8_t k = 0; k < SENSOR_COUNT; ++k) {
        LOG("  %s: wall min %4ld mean %6.1f | no wall max %4ld mean %6.1f | now THRESH %d\r\n", SENSOR_NAME[k],
            static_cast<long>(w[k].count ? w[k].min : 0), w[k].mean(),
            static_cast<long>(n[k].count ? n[k].max : 0), n[k].mean(), THRESH[k]);
        if (w[k].count > 0 && n[k].count > 0) {
            long margin = w[k].min - n[k].max;
            if (margin > 0) {
                LOG("      -> suggested THRESH_FRONT_%s = %ld (margin %ld each side)\r\n",
                    (k == FL) ? "LEFT" : "RIGHT", static_cast<long>((w[k].min + n[k].max) / 2), margin / 2);
            } else {
                LOG("      -> %s alone cannot separate (wall min <= no-wall max by %ld)\r\n", SENSOR_NAME[k], -margin);
            }
        }
    }
    LOG("  with the current thresholds: missed %lu / %lu wall samples, false %lu / %lu no-wall samples\r\n",
        static_cast<unsigned long>(g_totals.wrong[0]), static_cast<unsigned long>(g_totals.samples[0]),
        static_cast<unsigned long>(g_totals.wrong[1]), static_cast<unsigned long>(g_totals.samples[1]));
}

void resetFrontCheck() {
    g_totals = Totals{};
    LOG("front check: cleared\r\n");
}

void runFrontCheck(FrontCase c) {
    g_case = c;
    for (auto& s : g_run) s = Stats{};
    g_run_wrong = 0;
    LOG("front check (%s): place the back end on the back wall; %s; drive %.0f mm and read %lu samples\r\n",
        caseName(c), (c == FrontCase::wall) ? "front wall at the far side of the next cell" : "no front wall there",
        READ_X, static_cast<unsigned long>(SAMPLE_COUNT));

    runClosedLoopTest({"front_check", (c == FrontCase::wall) ? "wall" : "no_wall", front_check_init_log,
                       front_check_profile, 0.f, 0.f, 0});

    LOG("front check (%s) this run: FL %ld..%ld mean %.1f, FR %ld..%ld mean %.1f, judged wrong %lu / %lu\r\n",
        caseName(c), static_cast<long>(g_run[FL].min), static_cast<long>(g_run[FL].max), g_run[FL].mean(),
        static_cast<long>(g_run[FR].min), static_cast<long>(g_run[FR].max), g_run[FR].mean(),
        static_cast<unsigned long>(g_run_wrong), static_cast<unsigned long>(SAMPLE_COUNT));
    showFrontCheck();
}
