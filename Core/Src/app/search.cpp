#include "app/search.hpp"
#include <cstdio>
#include "adachi.hpp"
#include "common/debug.hpp"
#include "common/etc.hpp"
#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "device/uart.hpp"

namespace {
using config::maze::CELL_MM;
constexpr float HALF_MM = CELL_MM / 2.f;

// 置いた位置（車軸）からスタート区画の前の境界まで。ソルバーの ACT_MOVE_FIRST_HALF_CELL にあたる
constexpr float START_TO_EDGE = CELL_MM - config::maze::START_MM;

constexpr uint32_t TRACE_DECIMATION = 20;   // [tick] 時系列のログ。7列×3400サンプル＝68s（往復の探索が入る）
constexpr uint32_t SETTLE_MS = 500;

// ---- プリセットの検査（ビルド時）----
constexpr float accelDistance(const SearchPreset& p) {
    return p.speed * p.speed / (2.f * p.accel);
}

constexpr float pivotRamp(const PivotParam& p) {
    return p.omega * p.omega / (2.f * p.alpha);
}

// 集合のターン：使わない（nullptr）か，探索速度で設計されていて積める
constexpr bool turnUsable(const slalom::Param* t, float v) {
    return t == nullptr || (t->speed == v && slalom::validate(*t) == SegmentResult::ok);
}

constexpr bool turnsUsable(const SearchPreset& p) {
    const float v = p.speed;
    const OrthoTurns& o = p.turns;
    bool ok = o.s90 != nullptr && o.s90->entry == slalom::Anchor::edge && o.s90->exit == slalom::Anchor::edge
           && turnUsable(o.s90, v) && turnUsable(o.l90, v) && turnUsable(o.t180, v);
    if (p.diagonal != nullptr) {
        const DiagonalTurns& d = *p.diagonal;
        ok = ok && turnUsable(d.in45, v) && turnUsable(d.out45, v) && turnUsable(d.v90, v)
                && turnUsable(d.in135, v) && turnUsable(d.out135, v);
    }
    return ok;
}

constexpr bool presetRunnable(const SearchPreset& p) {
    using namespace config::profile_limit;
    const float v = p.speed;
    return turnsUsable(p)
        // 最初の半区画：accel で加速してから等速
        && p.accel <= MAX_ACCEL_X
        && accelDistance(p) < START_TO_EDGE
        && validateSegment(0.f, v, accelDistance(p), MAX_ACCEL_X, MAX_DECEL_X) == SegmentResult::ok
        // 行き止まり・ゴール：半区画で止まり，半区画で加速する
        && validateSegment(v, 0.f, HALF_MM, MAX_ACCEL_X, MAX_DECEL_X) == SegmentResult::ok
        && validateSegment(0.f, v, HALF_MM, MAX_ACCEL_X, MAX_DECEL_X) == SegmentResult::ok
        // 超信地旋回の180°に加速・減速が収まる
        && p.pivot.alpha <= MAX_ALPHA && 2.f * pivotRamp(p.pivot) < 180.f;
}

constexpr bool allPresetsRunnable() {
    for (const auto& p : config::search::PRESETS) {
        if (!presetRunnable(p)) return false;
    }
    return true;
}
static_assert(allPresetsRunnable(),
              "a search preset cannot run: check tools/search_presets.json against the profile limits");

// 壁を読む位置が1歩の中にある（最も短い1歩は行き止まりの後半の半区画）
constexpr float READ_LEAD = config::search::READ_LEAD_MM;
static_assert(READ_LEAD > 0.f && READ_LEAD < HALF_MM, "config::search::READ_LEAD_MM must be inside half a cell");

// ---- 壁を読むたびの記録（シミュレータの replay.py の LOG_COLUMNS と同じ列）----
struct SearchStep {
    uint8_t x, y, dir;   // 壁を読んだときの mousePos（これから入る区画）
    uint8_t walls;       // bit0: 左, bit1: 前, bit2: 右
    uint8_t action;      // ソルバーが返した動作（solver/core/action.h）
    uint8_t returning;   // 帰り探索中か
    int16_t ir[wall::POSITION_COUNT];
};

// CCMRAM（64KB，ほかに使っていない）に置く。スタートアップは CCMRAM を0にしないので，
// 件数（g_step_count，通常のRAM）だけで有効な範囲を表す
__attribute__((section(".ccmram"))) SearchStep g_steps[config::search::MAX_STEPS];
uint16_t g_step_count = 0;

constexpr const char* STEP_COLUMNS[] = {"step", "x", "y", "dir", "left", "front", "right", "action",
                                        "returning", "ir_l", "ir_fl", "ir_fr", "ir_r"};
constexpr uint32_t STEP_COLUMN_COUNT = sizeof(STEP_COLUMNS) / sizeof(STEP_COLUMNS[0]);

enum class Stop : uint8_t {
    finished,       // スタートに戻った（正常）
    frontWall,      // 壁のある向きへ進もうとした（壁の誤読か，ソルバーの経路がない）
    pushRejected,   // 区間を積めなかった（加速度の上限など）
    profileError,   // PlanProfile が区間を落とした，または読む位置の前に止まった
    tooManySteps,   // MAX_STEPS を超えた（ログが入らない）
    unknownAction,  // 探索では使わない動作が来た
};

const char* stopName(Stop s) {
    switch (s) {
    case Stop::finished:      return "finished";
    case Stop::frontWall:     return "wall ahead";
    case Stop::pushRejected:  return "segment rejected";
    case Stop::profileError:  return "profile error";
    case Stop::tooManySteps:  return "too many steps";
    default:                  return "unknown action";
    }
}

char g_log_name[24];
char g_trace_name[32];

void initTraceLog() {
    logger.initLoggedVal();
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&WallControl::omega>("wall_omega", wallControl);
    logger.add<&WallControl::offset>("wall_offset", wallControl);
    logger.setDecimation(TRACE_DECIMATION);
}

// ソルバーが返した列のうち最初の動作（SET_* とその引数，READ_WALL を飛ばす）。
// シミュレータの MazeSimulation._first_motion() と同じ
uint8_t firstMotion(const uint8_vector& actions) {
    for (std::size_t i = 0; i < actions.size(); ++i) {
        switch (actions[i]) {
        case SET_MOUSE_INFO: i += 3; break;
        case SET_VISITED:    i += 2; break;
        case SET_WALL:       i += 4; break;
        case READ_WALL:
        case ACT_NONE:       break;
        default:             return actions[i];
        }
    }
    return ACT_NONE;
}

// 超信地旋回（angle は符号つき，正で左）。並進が止まっていること（直前の区間が速度0で終わる）
bool pivot(const PivotParam& p, float angle) {
    float dir = (angle > 0.f) ? 1.f : -1.f;
    float ramp = pivotRamp(p);
    return planProfile.turn(dir * p.omega, dir * ramp) == SegmentResult::ok
        && planProfile.turn(dir * p.omega, angle - dir * 2.f * ramp) == SegmentResult::ok
        && planProfile.turn(0.f, dir * ramp) == SegmentResult::ok;
}

bool profileBroken() {
    return planProfile.rejectedCount() > 0 || planProfile.droppedCount() > 0;
}

// 探索の本体。PlanProfile に区間を積みながら，壁を読む位置ごとにソルバーを呼ぶ
Stop runSteps(const SearchPreset& p) {
    const float v = p.speed;

    solver_options_reset();
    solver_options.goal_x = config::search::GOAL_X;
    solver_options.goal_y = config::search::GOAL_Y;
    solver_options.search_return = true;
    uint8_vector first = adachi::solver_adachi_init();
    if (firstMotion(first) != ACT_MOVE_FIRST_HALF_CELL) return Stop::unknownAction;

    float d_acc = accelDistance(p);
    if (planProfile.straight(v, d_acc) != SegmentResult::ok ||
        planProfile.straight(v, START_TO_EDGE - d_acc) != SegmentResult::ok) {
        return Stop::pushRejected;
    }
    float step_end = START_TO_EDGE;   // 今積んでいる1歩が終わる位置（並進の目標位置，区画境界）

    while (true) {
        // 区画境界の READ_LEAD 手前まで待つ（今の1歩の残りが走っている間に次を積む）
        while (planProfile.getTargetPositionX() < step_end - READ_LEAD) {
            if (profileBroken() || planProfile.isIdle()) return Stop::profileError;
        }

        wall::Snapshot s = wall::read();
        bool left = wall::hasLeft(s);
        bool front = wall::hasFront(s);
        bool right = wall::hasRight(s);
        MousePos at = mousePos;   // 壁を読んだ区画（ソルバーが次の区画へ進める前）
        uint8_t action = firstMotion(adachi::solver_adachi(left, front, right));

        if (g_step_count >= config::search::MAX_STEPS) return Stop::tooManySteps;
        SearchStep& rec = g_steps[g_step_count++];
        rec = {at.x, at.y, at.dir,
               static_cast<uint8_t>((left ? 1 : 0) | (front ? 2 : 0) | (right ? 4 : 0)),
               action, static_cast<uint8_t>(adachi::is_returning() ? 1 : 0), {}};
        for (uint8_t i = 0; i < wall::POSITION_COUNT; ++i) rec.ir[i] = s.value[i];

        switch (action) {
        case ACT_MOVE_1CELL:
            if (front) return Stop::frontWall;
            if (planProfile.straight(v, CELL_MM) != SegmentResult::ok) return Stop::pushRejected;
            step_end += CELL_MM;
            break;
        case ACT_TURN_LEFT_MOVE:
        case ACT_TURN_RIGHT_MOVE: {
            bool to_left = (action == ACT_TURN_LEFT_MOVE);
            if (to_left ? left : right) return Stop::frontWall;
            auto dir = to_left ? slalom::TurnDir::left : slalom::TurnDir::right;
            if (slalom::push(planProfile, *p.turns.s90, dir) != SegmentResult::ok) return Stop::pushRejected;
            step_end += slalom::totalDistance(*p.turns.s90);
            break;
        }
        case ACT_TURN_BACK:
            // 区画中央で止まり，その場で180°回って，来た境界へ戻る
            if (planProfile.straight(0.f, HALF_MM) != SegmentResult::ok || !pivot(p.pivot, 180.f) ||
                planProfile.straight(v, HALF_MM) != SegmentResult::ok) {
                return Stop::pushRejected;
            }
            step_end += CELL_MM;
            break;
        case ACT_FINISH:
            // スタート区画の中央で止まる
            if (planProfile.straight(0.f, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
            return Stop::finished;
        default:
            return Stop::unknownAction;
        }
    }
}

void dumpSteps(const char* file) {
    bin_table::begin("search", file, false, g_step_count * STEP_COLUMN_COUNT * sizeof(float),
                     STEP_COLUMNS, STEP_COLUMN_COUNT);
    for (uint16_t i = 0; i < g_step_count; ++i) {
        const SearchStep& r = g_steps[i];
        float row[STEP_COLUMN_COUNT] = {
            static_cast<float>(i), static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.dir),
            static_cast<float>(r.walls & 1), static_cast<float>((r.walls >> 1) & 1),
            static_cast<float>((r.walls >> 2) & 1), static_cast<float>(r.action), static_cast<float>(r.returning),
            static_cast<float>(r.ir[wall::left]), static_cast<float>(r.ir[wall::front_left]),
            static_cast<float>(r.ir[wall::front_right]), static_cast<float>(r.ir[wall::right]),
        };
        uart_write(reinterpret_cast<const uint8_t*>(row), sizeof(row));
    }
    bin_table::end();
}

void blinkRefused() {
    for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s（runClosedLoopTestと同じ）
        ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
        HAL_Delay(500);
    }
    ledBar16.set(0x0000);
}
} // namespace

void runSearch(const SearchPreset& preset) {
    std::snprintf(g_log_name, sizeof(g_log_name), "%s", preset.name);
    std::snprintf(g_trace_name, sizeof(g_trace_name), "%s_trace", preset.name);
    LOG("search %s: %.0f mm/s, turn %s, wall control %s, goal (%u,%u)\r\n", preset.name, preset.speed,
        preset.turns.s90->name, preset.wall_control ? "on" : "off", config::search::GOAL_X, config::search::GOAL_Y);

    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    float v0 = battery.voltage();
    if (v0 < config::search::MIN_BATTERY_V) {
        LOG("search not started: battery %.2f V < %.2f V\r\n", v0, config::search::MIN_BATTERY_V);
        blinkRefused();
        return;
    }

    imu.calibrate();
    HAL_Delay(1100);

    initTraceLog();
    logger.setDirName("search");
    logger.setFileName(g_trace_name);
    logger.setIncludeTimestamp(false);

    // 原点の取り直し（runClosedLoopTest と同じ）と，横壁の補正の初期化
    odometry.reset();
    planProfile.reset();
    wallControl.reset();
    wallControl.enable(preset.wall_control);
    g_step_count = 0;

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.switchToVelocityX();

    Stop stop = runSteps(preset);
    if (stop == Stop::finished) {
        planProfile.waitUntilIdle();
        HAL_Delay(SETTLE_MS);
    }
    planProfile.stop();
    wallControl.enable(false);
    logger.stop();
    motorDriver.setBreak();

    LOG("search %s: %s after %u wall reads (rejected %lu, dropped %lu)\r\n", preset.name, stopName(stop),
        g_step_count, static_cast<unsigned long>(planProfile.rejectedCount()),
        static_cast<unsigned long>(planProfile.droppedCount()));
    if (stop != Stop::finished) {
        blinkRefused();
    }

    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    dumpSteps(g_log_name);
    logger.dump();
    ledBar16.set(0x0000);
}
