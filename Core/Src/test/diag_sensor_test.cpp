#include "test/diag_sensor_test.hpp"
#include <cmath>
#include <cstdio>
#include "common/debug.hpp"
#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "test/closed_loop_test.hpp"

// 斜め走行のセンサーのデータ収集（考え方は diag_sensor_test.hpp，手順は tools/DIAGONAL.md）
namespace {
using config::maze::CELL_MM;
using config::maze::START_MM;
using config::diag::PITCH_MM;
using config::diag::PILLAR_LATERAL_MM;

constexpr float RUNUP_ACCEL = 0.5f * config::profile_limit::G;   // [mm/s^2] 入口までの加速（スラロームの試験と同じ）
constexpr float STOP_MM = PITCH_MM / 2.f;   // [mm] 最後の区間の後半で止まる（柱の横を過ぎてから減速する）
// [mm] 出口の基準点のこれだけ手前からログを取り直す。最初の内側の柱（−63.6 mm）とその前の壁を入れる
constexpr float LOG_BEFORE_MM = 100.f;
constexpr uint32_t SETTLE_MS = 300;
constexpr uint32_t MAX_HALF_STEPS = 8;      // 迷路の 5×5 区画に収まる長さ（tools/DIAGONAL.md）

// 置いた位置から入45°の入口（1区画先の区画中央）まで
constexpr float RUNUP_MM = CELL_MM + CELL_MM / 2.f - START_MM;

constexpr float accelDistance(float v) {
    return v * v / (2.f * RUNUP_ACCEL);
}

// 試験中の値（runClosedLoopTest() の関数ポインタは引数を持てないので，ここで受け渡す）
const slalom::Param* g_param = nullptr;
slalom::TurnDir g_dir = slalom::TurnDir::left;
uint32_t g_half_steps = 0;
DiagTestMode g_mode = DiagTestMode::log;
float g_anchor = 0.f;        // [mm] 出口の基準点の経路に沿った距離（目標の座標）
uint32_t g_log_ms = 0;
bool g_log_restarted = false;
char g_file_name[40];

struct Result {
    float diag_x;        // [mm] 止まった位置（出口の基準点から）
    float angle_error;   // [deg] 止まったときの実測 − 目標
    uint32_t edges[DiagEdge::SIDE_COUNT];
    float control_offset;   // [deg] DiagControl が向きに足した補正
    uint32_t measured_ticks;
    uint32_t active_ticks;
};
Result g_result;

float diagX() {
    return odometry.positionX() - g_anchor;
}
float targetDiagX() {
    return planProfile.getTargetPositionX() - g_anchor;
}
float angleError() {
    return odometry.angle() - planProfile.getTargetAngle();
}
float irL() {
    return wall::read().value[wall::left];
}
float irFL() {
    return wall::read().value[wall::front_left];
}
float irFR() {
    return wall::read().value[wall::front_right];
}
float irR() {
    return wall::read().value[wall::right];
}

// ログを取り直してから止まるまで [ms]：等速の区間・減速（平均 v/2）・整定，1割の余裕
uint32_t logMs(float v, uint32_t half_steps) {
    float cruise = LOG_BEFORE_MM + half_steps * PITCH_MM - STOP_MM;
    float s = cruise / v + 2.f * STOP_MM / v;
    return static_cast<uint32_t>((s * 1000.f + SETTLE_MS) * 1.1f);
}

void diag_init_log() {
    logger.initLoggedVal();
    logger.add("diag_x", Logger::Getter::create<&diagX>());
    logger.add("target_diag_x", Logger::Getter::create<&targetDiagX>());
    logger.add("angle_error", Logger::Getter::create<&angleError>());
    logger.add("ir_l", Logger::Getter::create<&irL>());
    logger.add("ir_fl", Logger::Getter::create<&irFL>());
    logger.add("ir_fr", Logger::Getter::create<&irFR>());
    logger.add("ir_r", Logger::Getter::create<&irR>());
    logger.add<&DiagEdge::sinceLeft>("since_l", diagEdge);
    logger.add<&DiagEdge::sinceRight>("since_r", diagEdge);
    logger.add<&DiagControl::lateral>("diag_lat", diagControl);
    logger.add<&DiagControl::offset>("diag_offset", diagControl);
    logger.setDuration(g_log_ms);
}

void diag_profile() {
    const slalom::Param& p = *g_param;
    float v = p.speed;
    float accel = accelDistance(v);
    // 斜めの直線（出口の基準点から止まるまで）だけ DiagControl に教える。log でも横のずれは記録する
    diagControl.reset();
    DiagControl::Range range{g_anchor, g_anchor + g_half_steps * PITCH_MM};
    diagControl.setRanges(&range, 1);
    if (g_mode == DiagTestMode::inject) diagControl.injectAngle(DIAG_TEST_INJECT_DEG);
    diagControl.start(g_mode != DiagTestMode::log);
    planProfile.straight(v, accel);
    planProfile.straight(v, RUNUP_MM - accel);
    slalom::push(planProfile, p, g_dir);
    planProfile.straight(v, g_half_steps * PITCH_MM - STOP_MM);
    planProfile.straight(0.f, STOP_MM);

    // 斜めの直線の手前でログと切れ目の検出を始め直す（助走と旋回の分でバッファを使わないように）
    diagEdge.reset();
    while (planProfile.getTargetPositionX() < g_anchor - LOG_BEFORE_MM && !planProfile.isIdle()) {
    }
    if (!planProfile.isIdle()) {
        logger.stop();
        logger.start();
        diagEdge.start();
        g_log_restarted = true;
    }

    planProfile.waitUntilIdle();
    HAL_Delay(SETTLE_MS);
    g_result = {diagX(), angleError(), {diagEdge.edgeCount(DiagEdge::left), diagEdge.edgeCount(DiagEdge::right)},
                diagControl.offset(), diagControl.measuredTicks(), diagControl.activeTicks()};
    diagEdge.stop();
    diagControl.stop();
}

void blinkRefused() {
    for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s（runClosedLoopTestと同じ）
        ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
        HAL_Delay(500);
    }
    ledBar16.set(0x0000);
}

// 走らせる前の検査。だめなら理由を表示してfalse
bool checkRunnable(const slalom::Param& p, slalom::TurnDir dir, uint32_t half_steps) {
    using namespace config::profile_limit;
    if (p.entry != slalom::Anchor::center || p.exit != slalom::Anchor::diagonal || p.angle != 45.f) {
        LOG("diag test: %s is not an in45 (center -> diagonal, 45 deg)\r\n", p.name);
        return false;
    }
    if (half_steps < 1 || half_steps > MAX_HALF_STEPS) {
        LOG("diag test: half steps %lu out of 1..%lu\r\n", static_cast<unsigned long>(half_steps),
            static_cast<unsigned long>(MAX_HALF_STEPS));
        return false;
    }
    if (RUNUP_MM - accelDistance(p.speed) <= 0.f) {
        LOG("diag test: run-up %.1f mm is shorter than the acceleration %.1f mm\r\n", RUNUP_MM, accelDistance(p.speed));
        return false;
    }
    SegmentResult r = slalom::validate(p, dir);
    if (r != SegmentResult::ok) {
        LOG("diag test: %s rejected (%s)\r\n", p.name, slalom::resultName(r));
        return false;
    }
    if (validateSegment(p.speed, 0.f, STOP_MM, MAX_ACCEL_X, MAX_DECEL_X) != SegmentResult::ok) {
        LOG("diag test: cannot stop from %.0f mm/s in %.1f mm\r\n", p.speed, STOP_MM);
        return false;
    }
    return true;
}

// 柱の位置（出口の基準点からの経路に沿った距離）。内側（曲がった向き）は −63.6 + 254.6k，外側は +63.6 + 254.6k
void printPillars(slalom::TurnDir dir, uint32_t half_steps) {
    const char* inner = (dir == slalom::TurnDir::left) ? "L" : "R";
    const char* outer = (dir == slalom::TurnDir::left) ? "R" : "L";
    LOG("  pillars (diag_x, lateral %.1f mm):", PILLAR_LATERAL_MM);
    for (uint32_t k = 0; k <= half_steps; ++k) {
        float x = -PITCH_MM / 2.f + k * PITCH_MM;
        LOG(" %s%.1f", (k % 2 == 0) ? inner : outer, x);
    }
    LOG("\r\n");
}
} // namespace

void runDiagSensorTest(const slalom::Param& p, slalom::TurnDir dir, uint32_t half_steps, DiagTestMode mode) {
    const char* dir_name = (dir == slalom::TurnDir::left) ? "left" : "right";
    static const char* const MODE_SUFFIX[] = {"", "_ctrl", "_inj"};
    static const char* const MODE_NAME[] = {"log only", "control", "control + inject"};
    if (!checkRunnable(p, dir, half_steps)) {
        blinkRefused();
        return;
    }

    g_param = &p;
    g_dir = dir;
    g_half_steps = half_steps;
    g_mode = mode;
    g_anchor = RUNUP_MM + slalom::totalDistance(p, dir);
    g_log_ms = logMs(p.speed, half_steps);
    g_log_restarted = false;
    g_result = {};
    std::snprintf(g_file_name, sizeof(g_file_name), "%s_%s_n%lu%s", p.name, dir_name,
                  static_cast<unsigned long>(half_steps), MODE_SUFFIX[static_cast<uint8_t>(mode)]);

    float diag_mm = half_steps * PITCH_MM;
    LOG("diag test %s %s: %.0f mm/s, run-up %.1f mm, diagonal from %.1f mm, %lu x %.1f = %.1f mm (stop in the last %.1f mm)\r\n",
        p.name, dir_name, p.speed, RUNUP_MM, g_anchor, static_cast<unsigned long>(half_steps), PITCH_MM, diag_mm, STOP_MM);
    LOG("  diagonal control: %s", MODE_NAME[static_cast<uint8_t>(mode)]);
    if (mode == DiagTestMode::inject) LOG(" (heading %+.1f deg at the diagonal start)", DIAG_TEST_INJECT_DEG);
    LOG("\r\n");
    printPillars(dir, half_steps);

    float fan_duty = p.fan ? config::fan::RUN_DUTY : 0.f;
    runClosedLoopTest({"diag", g_file_name, diag_init_log, diag_profile, fan_duty, config::search::MIN_BATTERY_V, 0});

    if (!g_log_restarted) {
        LOG("diag test: the log was not restarted before the diagonal (profile ended early)\r\n");
    }
    LOG("diag result %s %s: stopped at diag_x %.1f mm (target %.1f, error %+.1f), angle error %+.2f deg, "
        "edges L %lu / R %lu\r\n",
        p.name, dir_name, g_result.diag_x, diag_mm, g_result.diag_x - diag_mm, g_result.angle_error,
        static_cast<unsigned long>(g_result.edges[DiagEdge::left]),
        static_cast<unsigned long>(g_result.edges[DiagEdge::right]));
    LOG("  diagonal control: measured %lu / %lu ms, heading offset %+.2f deg\r\n",
        static_cast<unsigned long>(g_result.measured_ticks), static_cast<unsigned long>(g_result.active_ticks),
        g_result.control_offset);
    LOG("  log %lu samples (recordable %lu ms for %lu ms)%s\r\n", static_cast<unsigned long>(logger.sampleCount()),
        static_cast<unsigned long>(logger.recordableMs()), static_cast<unsigned long>(g_log_ms),
        logger.isFull() ? ", FULL: the end of the run is missing" : "");
}
