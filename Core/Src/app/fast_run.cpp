#include "app/fast_run.hpp"
#include <cstdint>
#include <cstdio>
#include "app/fast_plan.hpp"
#include "app/maze_store.hpp"
#include "app/wall_edge_log.hpp"
#include "common/debug.hpp"
#include "common/etc.hpp"
#include "device/device_instance.hpp"
#include "time_based_dijkstra.hpp"
#include "device/imu_calibration.hpp"

namespace {
using config::maze::CELL_MM;
constexpr float HALF_MM = CELL_MM / 2.f;

// 置いた位置（車軸）から (0,0) の区画中央まで。経路はそこから始まる
constexpr float START_TO_CENTER = HALF_MM - config::maze::START_MM;
static_assert(START_TO_CENTER > 0.f, "the axle must start behind the center of the start cell");

constexpr uint32_t TRACE_MS = 30000;   // [ms] 時系列のログの長さ
constexpr uint32_t SETTLE_MS = 500;

// [mm] 区画境界を WallEdge に教えるのは，実測の位置が境界のこれだけ手前に来てから。
// 壁切れは境界の約 −91 mm（config::wall_edge::OFFSET_*）で検出するので，窓の幅を足しても間に合う。
// 先に教えすぎると，WallEdge が同時に待てる境界（MAX_PENDING = 4）からあふれて古い境界が捨てられる
constexpr float EDGE_FEED_LEAD_MM = 300.f;

// ---- プリセットの検査（ビルド時）：候補が速い順で，同じファンの条件で設計されていて積める ----
// 種類ごとの入口・出口（slalom::Anchor）。fast_plan と ACT の幾何の約束（fast_plan.hpp）
constexpr bool anchorsMatch(const slalom::Param& t, uint8_t k) {
    using slalom::Anchor;
    switch (k) {
    case TURN_S90:    return t.entry == Anchor::edge && t.exit == Anchor::edge;
    case TURN_L90:
    case TURN_180:    return t.entry == Anchor::center && t.exit == Anchor::center;
    case TURN_IN45:
    case TURN_IN135:  return t.entry == Anchor::center && t.exit == Anchor::diagonal;
    case TURN_OUT45:
    case TURN_OUT135: return t.entry == Anchor::diagonal && t.exit == Anchor::center;
    case TURN_V90:    return t.entry == Anchor::diagonal && t.exit == Anchor::diagonal;
    default:          return false;
    }
}

constexpr bool ladderUsable(const TurnLadder& l, uint8_t k, const RunPreset& p) {
    for (uint8_t i = 0; i < l.count; ++i) {
        const slalom::Param* t = l.list[i];
        if (t == nullptr || t->fan != p.fan || !anchorsMatch(*t, k) || slalom::validate(*t) != SegmentResult::ok) return false;
        if (i > 0 && !(t->speed < l.list[i - 1]->speed)) return false;   // 速い順
    }
    return true;
}

constexpr bool isDiagonalKind(uint8_t k) {
    return k == TURN_IN45 || k == TURN_OUT45 || k == TURN_V90 || k == TURN_IN135 || k == TURN_OUT135;
}

constexpr bool presetUsable(const RunPreset& p) {
    bool ok = p.accel > 0.f && p.accel <= config::profile_limit::MAX_ACCEL_X
           && p.decel > 0.f && p.decel <= config::profile_limit::MAX_DECEL_X
           && p.max_speed >= p.turn_speed && p.max_speed_dia >= p.turn_speed
           // 大回り90°・180° は必須。小回り90°は斜めなしなら必須（ジグザグを曲がる），斜めありなら任意
           && p.turns[TURN_L90].count > 0 && p.turns[TURN_180].count > 0
           && (p.diagonal || p.turns[TURN_S90].count > 0)
           // ファンを回すなら duty は (0, 1]，回さないなら 0
           && (p.fan ? (p.fan_duty > 0.f && p.fan_duty <= 1.f) : p.fan_duty == 0.f);
    for (uint8_t k = 0; k < TURN_KIND_COUNT; ++k) {
        const TurnLadder& l = p.turns[k];
        ok = ok && ladderUsable(l, k, p);
        if (isDiagonalKind(k)) {
            // 斜めなしなら斜めのターンは持たない。斜めありなら V90 以外は必須
            ok = ok && (p.diagonal ? (k == TURN_V90 || l.count > 0) : l.count == 0);
        }
        // 直線はターンの速度で入って出るので，最高速度はターンの速度以上（斜めの直線は斜めのターンの隣だけ）
        if (l.count > 0) {
            ok = ok && p.max_speed >= l.list[0]->speed && (!isDiagonalKind(k) || p.max_speed_dia >= l.list[0]->speed);
        }
    }
    return ok;
}

constexpr bool allPresetsUsable() {
    for (const auto& p : config::run::PRESETS) {
        if (!presetUsable(p)) return false;
    }
    return true;
}
static_assert(allPresetsUsable(), "a run preset cannot run: check tools/run_presets.json against the slaloms and limits");

// 経路の手順（スタックに置かない）
fast_plan::Steps g_steps;

// 壁切れの補正に教える区画境界（区画中央から入るターンの手前と入口 + 90。fast_plan::edgeBoundaries）と，次に教える番号
constexpr std::size_t MAX_EDGE_BOUNDARIES = 64;
float g_edge_boundaries[MAX_EDGE_BOUNDARIES];
std::size_t g_edge_count = 0;
std::size_t g_edge_next = 0;

// 斜めの直線の範囲（斜めの姿勢制御 DiagControl に教える。fast_plan::diagonalRanges）
DiagControl::Range g_diag_ranges[DiagControl::MAX_RANGES];
uint8_t g_diag_count = 0;

// 縦横の直線の範囲（横壁の補正 WallControl に教える。fast_plan::orthogonalRanges）。直線とターンは交互なので
// 手順の半分＋1 あれば足りる。WallControl はコピーせずに読むので，走り終わるまで書き換えない
constexpr std::size_t MAX_ORTHO_RANGES = fast_plan::MAX_STEPS / 2 + 1;
float g_ortho_x0[MAX_ORTHO_RANGES];
float g_ortho_x1[MAX_ORTHO_RANGES];
uint16_t g_ortho_count = 0;

char g_trace_name[32];
char g_edge_name[32];

void initTraceLog(bool diagonal) {
    logger.initLoggedVal();
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&Odometry::velocityX>("current_velocity_x", odometry);
    logger.add<&Motor::getDuty>("left_duty", motorLeft);
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&WallEdge::totalShift>("edge_shift", wallEdge);
    logger.add<&WallControl::offset>("wall_offset", wallControl);
    if (diagonal) {
        logger.add<&DiagControl::lateral>("diag_lat", diagControl);
        logger.add<&DiagControl::offset>("diag_offset", diagControl);
    }
    logger.setDuration(TRACE_MS);
}

void blinkRefused() {
    for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s（runSearch と同じ）
        ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
        HAL_Delay(500);
    }
    ledBar16.set(0x0000);
}

// 経路を求めた迷路とソルバーの見積もり（経路の情報は走った後に出すので，それまで取っておく）
struct PlanInfo {
    char bank;
    uint32_t sequence;
    uint8_t goal_x;
    uint8_t goal_y;
    uint8_t goal_size;  // ゴール領域の一辺（1 なら goal_x/goal_y の1区画）
    uint8_t end_x;      // 経路が終わる区画（領域のときは到達の最も遅い区画）
    uint8_t end_y;
    bool complete;
    uint16_t solver_ms;
};
PlanInfo g_plan_info;

void printMaze(const RunPreset& p) {
    const PlanInfo& m = g_plan_info;
    LOG("fast %s: maze bank %c, sequence %lu, goal (%u,%u)-(%u,%u), end (%u,%u), %s\r\n", p.name, m.bank,
        static_cast<unsigned long>(m.sequence), m.goal_x, m.goal_y, m.goal_x + m.goal_size - 1,
        m.goal_y + m.goal_size - 1, m.end_x, m.end_y, m.complete ? "complete" : "partial");
}

// プリセットの速度と，種類ごとのターンの候補（速い順。走る経路で直線が短いところは下の候補に落ちる）
void printPreset(const RunPreset& p) {
    LOG("fast %s: turn %.0f mm/s, straight %.0f / dia %.0f mm/s, accel %.0f / decel %.0f, diagonal %s, fan %.2f\r\n",
        p.name, p.turn_speed, p.max_speed, p.max_speed_dia, p.accel, p.decel,
        p.diagonal ? "on" : "off", p.fan_duty);
    static const char* const KIND_NAMES[TURN_KIND_COUNT] = {"L90", "T180", "IN45", "OUT45", "IN135", "OUT135", "V90", "S90"};
    for (uint8_t k = 0; k < TURN_KIND_COUNT; ++k) {
        const TurnLadder& l = p.turns[k];
        if (l.count == 0) continue;
        LOG("  %s:", KIND_NAMES[k]);
        for (uint8_t i = 0; i < l.count; ++i) LOG(" %.0f", l.list[i]->speed);
        LOG("\r\n");
    }
}

// 保存した迷路とプリセットから経路を求めて g_steps にする。走れなければ理由を出して false。
// 走れるときは何も出さない（経路の情報は printPlan で走った後に出す。走る前の UART の出力を減らす）
bool plan(const RunPreset& p) {
    uint8_t bank = 0;
    const maze_store::Record* r = maze_store::latest(&bank);
    if (r == nullptr) {
        LOG("fast %s: no saved maze (search first)\r\n", p.name);
        return false;
    }
    // 本番のゴール (7,7) は (7,7)〜(8,8) の領域として解き、到達の最も遅い区画で止まる（減速は領域の中）
    const uint8_t goal_size = config::search::goalSize(r->goal_x, r->goal_y);
    g_plan_info = {static_cast<char>('A' + bank), r->sequence, r->goal_x, r->goal_y, goal_size,
                   r->goal_x, r->goal_y, (r->flags & maze_store::FLAG_COMPLETE) != 0, 0};

    // ソルバーの時間のコストを、このプリセットの速度とスラロームの経路長にそろえる
    solver_options_reset();
    solver_options.goal_x = r->goal_x;
    solver_options.goal_y = r->goal_y;
    solver_options.goal_size = goal_size;
    if (!fast_plan::applySolverCosts(p)) {
        printMaze(p);
        LOG("fast %s: invalid run profile\r\n", p.name);
        return false;
    }

    // 既知の壁だけで（wallone：未知は壁）最短時間の経路を求める
    maze_store::applyToSolver(*r);
    uint8_vector actions = time_based_dijkstra::solver_time_based_dijekstra_init();
    // 時間0でも経路はありうる（ゴール (0,1) はスタートから1区画進むだけ）ので、経路の有無で見る
    if (!time_based_dijkstra::last_path_found()) {
        printMaze(p);
        LOG("fast %s: no path to the goal with the known walls\r\n", p.name);
        return false;
    }
    g_plan_info.solver_ms = time_based_dijkstra::last_path_time_ms();
    time_based_dijkstra::last_goal_cell(&g_plan_info.end_x, &g_plan_info.end_y);

    fast_plan::Error e = fast_plan::build(actions, p, START_TO_CENTER, &g_steps);
    if (e != fast_plan::Error::none) {
        printMaze(p);
        LOG("fast %s: cannot run the path: %s\r\n", p.name, fast_plan::errorName(e));
        return false;
    }
    std::size_t bad = 0;
    SegmentResult v = fast_plan::validate(g_steps, p, &bad);
    if (v != SegmentResult::ok) {
        printMaze(p);
        LOG("fast %s: step %u cannot be pushed: %s\r\n", p.name, static_cast<unsigned>(bad), slalom::resultName(v));
        return false;
    }

    g_edge_count = fast_plan::edgeBoundaries(g_steps, p, config::wall_edge::FAST_BOUNDARIES_PER_TURN,
                                             config::wall_edge::FAST_CENTER_BOUNDARY, g_edge_boundaries,
                                             MAX_EDGE_BOUNDARIES);

    float x0[DiagControl::MAX_RANGES];
    float x1[DiagControl::MAX_RANGES];
    std::size_t nd = fast_plan::diagonalRanges(g_steps, x0, x1, DiagControl::MAX_RANGES);
    g_diag_count = static_cast<uint8_t>(nd);
    for (uint8_t i = 0; i < g_diag_count; ++i) g_diag_ranges[i] = {x0[i], x1[i]};
    g_ortho_count = static_cast<uint16_t>(
        fast_plan::orthogonalRanges(g_steps, g_ortho_x0, g_ortho_x1, MAX_ORTHO_RANGES));
    return true;
}

// plan() で求めた経路の情報（迷路・見積もりの時間・壁切れの境界・斜めの直線・手順）
void printPlan(const RunPreset& p) {
    printPreset(p);
    printMaze(p);
    LOG("fast %s: %u steps, solver estimate %u ms, profile estimate %.0f ms\r\n", p.name,
        static_cast<unsigned>(g_steps.size()), g_plan_info.solver_ms, 1000.f * fast_plan::estimatedTime(g_steps, p));
    LOG("fast %s: wall edge %s, %u boundaries around the center-entry turns:", p.name,
        p.wall_edge ? "correction" : "log only", static_cast<unsigned>(g_edge_count));
    for (std::size_t i = 0; i < g_edge_count; ++i) LOG(" %.0f", g_edge_boundaries[i]);
    LOG("\r\n");
    LOG("fast %s: wall control %s, %u orthogonal straights:", p.name, config::wall::FAST_RUN_CONTROL ? "on" : "off",
        static_cast<unsigned>(g_ortho_count));
    for (uint16_t i = 0; i < g_ortho_count; ++i) LOG(" %.0f-%.0f", g_ortho_x0[i], g_ortho_x1[i]);
    LOG("\r\n");
    if (p.diagonal) {
        LOG("fast %s: diagonal control %s, %u diagonal straights:", p.name, p.diag_control ? "on" : "log only",
            static_cast<unsigned>(g_diag_count));
        for (uint8_t i = 0; i < g_diag_count; ++i) LOG(" %.0f-%.0f", g_diag_ranges[i].x0, g_diag_ranges[i].x1);
        LOG("\r\n");
    }
    for (std::size_t i = 0; i < g_steps.size(); ++i) {
        const fast_plan::Step& s = g_steps[i];
        if (s.turn != nullptr) {
            LOG("  %2u %s %s\r\n", static_cast<unsigned>(i), s.turn->name, s.dir == slalom::TurnDir::left ? "L" : "R");
        } else {
            LOG("  %2u straight%s %.1f mm\r\n", static_cast<unsigned>(i), s.diagonal ? " dia" : "", s.distance);
        }
    }
}

bool profileBroken() {
    return planProfile.rejectedCount() > 0 || planProfile.droppedCount() > 0;
}

// 実測の位置が近づいた区画境界を WallEdge に教える（待っている間に何度も呼ぶ）
void feedEdges() {
    while (g_edge_next < g_edge_count &&
           odometry.positionX() >= g_edge_boundaries[g_edge_next] - EDGE_FEED_LEAD_MM) {
        if (!wallEdge.expect(g_edge_boundaries[g_edge_next])) return;   // キューが一杯なら次に呼ばれたとき
        ++g_edge_next;
    }
}

// 積めなかった手順とその理由（走行中は UART に出さず，持ち上げた後に出す）
constexpr std::size_t NO_REJECT = SIZE_MAX;
std::size_t g_reject_step = NO_REJECT;
SegmentResult g_reject_result = SegmentResult::ok;

// 手順を順に積む。キューの空きが1つの手順の区間ぶん（スラロームは最大5区間）あるまで待ってから積む
bool runSteps(const RunPreset& p) {
    constexpr std::size_t SLALOM_SEGMENTS = 5;   // 入口オフセット・角速度の加速・等角速度・減速・出口オフセット
    fast_plan::Segment seg[fast_plan::MAX_SEGMENTS_PER_STEP];
    g_reject_step = NO_REJECT;
    for (std::size_t i = 0; i < g_steps.size(); ++i) {
        uint8_t n = fast_plan::segments(g_steps, i, p, seg);
        for (uint8_t k = 0; k < n; ++k) {
            std::size_t need = (seg[k].turn != nullptr) ? SLALOM_SEGMENTS : 1;
            while (planProfile.freeSlots() < need) {
                if (profileBroken()) return false;
                feedEdges();
            }
            SegmentResult r = (seg[k].turn != nullptr) ? slalom::push(planProfile, *seg[k].turn, seg[k].dir)
                                                       : planProfile.straight(seg[k].v_end, seg[k].distance);
            if (r != SegmentResult::ok) {
                g_reject_step = i;
                g_reject_result = r;
                return false;
            }
        }
    }
    return true;
}
} // namespace

void runFastRun(const RunPreset& preset) {
    std::snprintf(g_trace_name, sizeof(g_trace_name), "%s_trace", preset.name);
    std::snprintf(g_edge_name, sizeof(g_edge_name), "%s_edges", preset.name);
    // 走る前は，走れないときの理由だけを出す（プリセット・経路の情報は haltByAccZ の後に printPlan で出す）
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    float v0 = battery.voltage();
    if (v0 < config::search::MIN_BATTERY_V) {
        LOG("fast not started: battery %.2f V < %.2f V\r\n", v0, config::search::MIN_BATTERY_V);
        blinkRefused();
        return;
    }
    if (!plan(preset)) {
        printPreset(preset);
        blinkRefused();
        return;
    }

    // ここから先は runSearch と同じ準備（IMU校正：ファンを回すなら，回して定常になってから）
    calibrateImuForRun(preset.fan_duty);

    initTraceLog(preset.diagonal);
    logger.setDirName("fast");
    logger.setFileName(g_trace_name);
    logger.setIncludeTimestamp(false);

    odometry.reset();
    planProfile.reset();
    // 横壁の補正：縦横の直線の範囲の中だけ（斜めの直線・スラロームのオフセットでは横壁を読めない）
    wallControl.reset();
    wallControl.setRanges(g_ortho_x0, g_ortho_x1, g_ortho_count);
    wallControl.enable(config::wall::FAST_RUN_CONTROL);
    // 斜めの直線：切れ目からの距離の表で向きを補正する（プリセットが false なら横のずれを記録するだけ）
    diagEdge.reset();
    diagControl.reset();
    if (preset.diagonal && g_diag_count > 0) {
        diagControl.setRanges(g_diag_ranges, g_diag_count);
        diagEdge.start();
        diagControl.start(preset.diag_control);
    }
    // 壁切れ：区画中央から入るターン（大回り・入45°・入135°）の手前の境界と入口 + 90 の境界だけ教える（教えていない壁切れは記録だけ）。プリセットが false なら補正しない
    wallEdge.reset();
    // 最短走行は足りない向き（予想より後ろの壁切れ）を FAST_WINDOW_LATE_MM まで補正する
    wallEdge.start(preset.wall_edge, config::wall_edge::WINDOW_MM, config::wall_edge::FAST_WINDOW_LATE_MM);
    g_edge_next = 0;

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.switchToVelocityX();

    bool ok = runSteps(preset);
    if (ok) {
        while (!planProfile.isIdle() && !profileBroken()) feedEdges();   // 最後の手順の境界も教える
        HAL_Delay(SETTLE_MS);
    }
    ok = ok && !profileBroken();
    planProfile.stop();
    fan.stop();
    wallEdge.stop();
    wallControl.enable(false);
    diagControl.stop();
    diagEdge.stop();
    logger.stop();
    motorDriver.setBreak();
    wallControl.reset();   // ブレーキの後に補正の向きを0へ戻し，範囲も外す（g_ortho_x0 / x1 を読まなくなる）

    // 結果は機体を持ち上げた後（haltByAccZ の後）に UART へ出す（runSearch と同じ）
    if (!ok) blinkRefused();

    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();

    printPlan(preset);
    if (g_reject_step != NO_REJECT) {
        LOG("fast %s: step %u rejected: %s\r\n", preset.name, static_cast<unsigned>(g_reject_step),
            slalom::resultName(g_reject_result));
    }
    LOG("fast %s: %s (rejected %lu, dropped %lu)\r\n", preset.name, ok ? "finished" : "stopped",
        static_cast<unsigned long>(planProfile.rejectedCount()), static_cast<unsigned long>(planProfile.droppedCount()));
    LOG("wall edge: %lu edges, correction %s, total shift %+.1f mm\r\n",
        static_cast<unsigned long>(wallEdge.eventCount()), preset.wall_edge ? "on" : "off", wallEdge.totalShift());
    if (preset.diagonal) {
        LOG("diagonal control: %s, measured %lu / %lu ms on the diagonals, heading offset %+.2f deg, "
            "edges L %lu / R %lu\r\n",
            preset.diag_control ? "on" : "log only", static_cast<unsigned long>(diagControl.measuredTicks()),
            static_cast<unsigned long>(diagControl.activeTicks()), diagControl.offset(),
            static_cast<unsigned long>(diagEdge.edgeCount(DiagEdge::left)),
            static_cast<unsigned long>(diagEdge.edgeCount(DiagEdge::right)));
    }
    wall_edge_log::dump("fast", g_edge_name);
    logger.dump();
    ledBar16.set(0x0000);
}
