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

// [ms] 小回り90°の無い（斜めありの）プリセットで，ソルバーの S90 のコストにする値。ほかの経路があれば必ずそちらを選ぶ。
// ソルバーの時間は uint16_t（未到達は 0xFFFF）なので，経路の時間と足しても 65535 を超えにくい大きさにとどめる。
// それでも S90 しかない経路になったら，fast_plan が turnMissing で走行を断る
constexpr uint16_t S90_DISABLED_MS = 30000;

// ---- プリセットの検査（ビルド時）：ターンがターンの速度・同じファンの条件で設計されていて積める ----
constexpr bool turnUsable(const slalom::Param* t, const RunPreset& p, bool required, float speed) {
    if (t == nullptr) return !required;
    return t->speed == speed && t->fan == p.fan && slalom::validate(*t) == SegmentResult::ok;
}
constexpr bool turnUsable(const slalom::Param* t, const RunPreset& p, bool required) {
    return turnUsable(t, p, required, p.turn_speed);
}

// S90 と他のターンの間の直線は最短で半区画。そこで2つの速度の間を加速・減速しきれるか（gen_run_presets.py と同じ検査）
constexpr bool speedsBridgeable(const RunPreset& p) {
    float dv2 = p.turn_speed * p.turn_speed - p.s90_speed * p.s90_speed;
    float lim = 2.f * ((p.accel < p.decel) ? p.accel : p.decel) * (config::maze::CELL_MM / 2.f);
    return dv2 <= lim && -dv2 <= lim;
}

constexpr bool presetUsable(const RunPreset& p) {
    // 小回り90°は斜めなしなら必須（ジグザグを曲がる），斜めありなら任意
    const slalom::Param* s90 = p.turns.s90;
    // S90 の速度を他のターンと変えてよいのは斜めなしのときだけ（斜めのターンとのつながりは確かめていない）
    bool ok = turnUsable(s90, p, p.diagonal == nullptr, p.s90_speed)
           && (s90 == nullptr || (s90->entry == slalom::Anchor::edge && s90->exit == slalom::Anchor::edge))
           && (p.s90_speed == p.turn_speed || (p.diagonal == nullptr && speedsBridgeable(p)))
           && turnUsable(p.turns.l90, p, true) && turnUsable(p.turns.t180, p, true)
           && p.max_speed >= p.turn_speed && p.max_speed >= p.s90_speed && p.max_speed_dia >= p.turn_speed
           && p.accel > 0.f && p.accel <= config::profile_limit::MAX_ACCEL_X
           && p.decel > 0.f && p.decel <= config::profile_limit::MAX_DECEL_X;
    if (p.diagonal != nullptr) {
        const DiagonalTurns& d = *p.diagonal;
        ok = ok && turnUsable(d.in45, p, true) && turnUsable(d.out45, p, true) && turnUsable(d.v90, p, true)
                && turnUsable(d.in135, p, true) && turnUsable(d.out135, p, true);
    }
    // スタート用のターン：start_speed で設計されていて，置いた位置から最初の区画中央までで start_speed まで加速できる
    if (p.start_speed > 0.f) {
        ok = ok && p.start_speed < p.turn_speed && p.start_turns.s90 == nullptr
                && p.start_speed * p.start_speed <= 2.f * config::profile_limit::MAX_ACCEL_X * START_TO_CENTER
                && turnUsable(p.start_turns.l90, p, true, p.start_speed)
                && turnUsable(p.start_turns.t180, p, true, p.start_speed)
                && ((p.diagonal == nullptr) == (p.start_diagonal == nullptr));
        if (p.start_diagonal != nullptr) {
            const DiagonalTurns& d = *p.start_diagonal;
            const float v = p.start_speed;
            ok = ok && turnUsable(d.in45, p, true, v) && turnUsable(d.out45, p, true, v) && turnUsable(d.v90, p, true, v)
                    && turnUsable(d.in135, p, true, v) && turnUsable(d.out135, p, true, v);
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

// 壁切れの補正に教える区画境界（区画中央から入るターンの手前。fast_plan::edgeBoundaries）と，次に教える番号
constexpr std::size_t MAX_EDGE_BOUNDARIES = 64;
float g_edge_boundaries[MAX_EDGE_BOUNDARIES];
std::size_t g_edge_count = 0;
std::size_t g_edge_next = 0;

char g_trace_name[32];
char g_edge_name[32];

void initTraceLog() {
    logger.initLoggedVal();
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&WallEdge::totalShift>("edge_shift", wallEdge);
    logger.setDuration(TRACE_MS);
}

void blinkRefused() {
    for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s（runSearch と同じ）
        ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
        HAL_Delay(500);
    }
    ledBar16.set(0x0000);
}

// ターンの経路長（左右の平均）。ソルバーの時間のコストに使う
float turnDist(const slalom::Param* t, float fallback) {
    if (t == nullptr) return fallback;
    return 0.5f * (slalom::totalDistance(*t, slalom::TurnDir::left) + slalom::totalDistance(*t, slalom::TurnDir::right));
}

// 保存した迷路とプリセットから経路を求めて g_steps にする。走れなければ理由を出して false
bool plan(const RunPreset& p) {
    uint8_t bank = 0;
    const maze_store::Record* r = maze_store::latest(&bank);
    if (r == nullptr) {
        LOG("fast %s: no saved maze (search first)\r\n", p.name);
        return false;
    }
    LOG("fast %s: maze bank %c, sequence %lu, goal (%u,%u), %s\r\n", p.name, 'A' + bank,
        static_cast<unsigned long>(r->sequence), r->goal_x, r->goal_y,
        (r->flags & maze_store::FLAG_COMPLETE) ? "complete" : "partial");

    // ソルバーの時間のコストを、このプリセットの速度とスラロームの経路長にそろえる
    solver_options_reset();
    solver_options.goal_x = r->goal_x;
    solver_options.goal_y = r->goal_y;
    solver_options.diagonal = (p.diagonal != nullptr);
    RunProfile prof;
    prof.cell_mm = CELL_MM;
    prof.turn_speed = p.turn_speed;
    prof.max_speed = p.max_speed;
    prof.max_speed_dia = p.max_speed_dia;
    prof.accel = p.accel;
    prof.decel = p.decel;
    prof.turn_dist[TURN_L90] = turnDist(p.turns.l90, prof.turn_dist[TURN_L90]);
    prof.turn_dist[TURN_180] = turnDist(p.turns.t180, prof.turn_dist[TURN_180]);
    prof.turn_dist[TURN_S90] = turnDist(p.turns.s90, prof.turn_dist[TURN_S90]);
    if (p.diagonal != nullptr) {
        const DiagonalTurns& d = *p.diagonal;
        prof.turn_dist[TURN_IN45] = turnDist(d.in45, prof.turn_dist[TURN_IN45]);
        prof.turn_dist[TURN_OUT45] = turnDist(d.out45, prof.turn_dist[TURN_OUT45]);
        prof.turn_dist[TURN_IN135] = turnDist(d.in135, prof.turn_dist[TURN_IN135]);
        prof.turn_dist[TURN_OUT135] = turnDist(d.out135, prof.turn_dist[TURN_OUT135]);
        prof.turn_dist[TURN_V90] = turnDist(d.v90, prof.turn_dist[TURN_V90]);
    }
    if (!solver_options_apply_profile(prof)) {
        LOG("fast %s: invalid run profile\r\n", p.name);
        return false;
    }
    if (p.turns.s90 == nullptr) {
        solver_options.turn_ms[TURN_S90] = S90_DISABLED_MS;   // 小回り90°を使わない（ジグザグは斜めで走る）
    } else if (p.s90_speed != p.turn_speed) {
        // コスト表は turn_speed で作るので，小回り90°だけ自分の速度で時間を見積もり直す
        // （S90 の前後の半区画の直線は turn_speed のままの近似）
        float ms = 1000.f * prof.turn_dist[TURN_S90] / p.s90_speed;
        solver_options.turn_ms[TURN_S90] = static_cast<uint16_t>((ms < 60000.f) ? ms + 0.5f : 60000.f);
    }

    // 既知の壁だけで（wallone：未知は壁）最短時間の経路を求める
    maze_store::applyToSolver(*r);
    uint8_vector actions = time_based_dijkstra::solver_time_based_dijekstra_init();
    // 時間0でも経路はありうる（ゴール (0,1) はスタートから1区画進むだけ）ので、経路の有無で見る
    if (!time_based_dijkstra::last_path_found()) {
        LOG("fast %s: no path to the goal with the known walls\r\n", p.name);
        return false;
    }

    fast_plan::Error e = fast_plan::build(actions, p, START_TO_CENTER, &g_steps);
    if (e != fast_plan::Error::none) {
        LOG("fast %s: cannot run the path: %s\r\n", p.name, fast_plan::errorName(e));
        return false;
    }
    std::size_t bad = 0;
    SegmentResult v = fast_plan::validate(g_steps, p, &bad);
    if (v != SegmentResult::ok) {
        LOG("fast %s: step %u cannot be pushed: %s\r\n", p.name, static_cast<unsigned>(bad), slalom::resultName(v));
        return false;
    }

    LOG("fast %s: %u steps, solver estimate %u ms, profile estimate %.0f ms\r\n", p.name,
        static_cast<unsigned>(g_steps.size()), time_based_dijkstra::last_path_time_ms(),
        1000.f * fast_plan::estimatedTime(g_steps, p));

    g_edge_count = fast_plan::edgeBoundaries(g_steps, p, config::wall_edge::FAST_BOUNDARIES_PER_TURN,
                                             g_edge_boundaries, MAX_EDGE_BOUNDARIES);
    LOG("fast %s: wall edge %s, %u boundaries before the center-entry turns:", p.name,
        p.wall_edge ? "correction" : "log only", static_cast<unsigned>(g_edge_count));
    for (std::size_t i = 0; i < g_edge_count; ++i) LOG(" %.0f", g_edge_boundaries[i]);
    LOG("\r\n");
    for (std::size_t i = 0; i < g_steps.size(); ++i) {
        const fast_plan::Step& s = g_steps[i];
        if (s.turn != nullptr) {
            LOG("  %2u %s %s\r\n", static_cast<unsigned>(i), s.turn->name, s.dir == slalom::TurnDir::left ? "L" : "R");
        } else {
            LOG("  %2u straight%s %.1f mm\r\n", static_cast<unsigned>(i), s.diagonal ? " dia" : "", s.distance);
        }
    }
    return true;
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
    LOG("fast %s: turn %.0f / s90 %.0f mm/s, straight %.0f / dia %.0f mm/s, accel %.0f / decel %.0f, diagonal %s, fan %s\r\n",
        preset.name, preset.turn_speed, preset.s90_speed, preset.max_speed, preset.max_speed_dia, preset.accel, preset.decel,
        preset.diagonal ? "on" : "off", preset.fan ? "on" : "off");

    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    float v0 = battery.voltage();
    if (v0 < config::search::MIN_BATTERY_V) {
        LOG("fast not started: battery %.2f V < %.2f V\r\n", v0, config::search::MIN_BATTERY_V);
        blinkRefused();
        return;
    }
    if (!plan(preset)) {
        blinkRefused();
        return;
    }

    // ここから先は runSearch と同じ準備（IMU校正：ファンを回すなら，回して定常になってから）
    calibrateImuForRun(preset.fan ? config::fan::RUN_DUTY : 0.f);

    initTraceLog();
    logger.setDirName("fast");
    logger.setFileName(g_trace_name);
    logger.setIncludeTimestamp(false);

    odometry.reset();
    planProfile.reset();
    wallControl.reset();
    wallControl.enable(false);   // 横壁の補正は使わない（斜めでは横壁を読めない）
    // 壁切れ：区画中央から入るターン（大回り・入45°・入135°）の手前の境界だけ教える（教えていない壁切れは記録だけ）。プリセットが false なら補正しない
    wallEdge.reset();
    wallEdge.start(preset.wall_edge);
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
    logger.stop();
    motorDriver.setBreak();

    // 結果は機体を持ち上げた後（haltByAccZ の後）に UART へ出す（runSearch と同じ）
    if (!ok) blinkRefused();

    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();

    if (g_reject_step != NO_REJECT) {
        LOG("fast %s: step %u rejected: %s\r\n", preset.name, static_cast<unsigned>(g_reject_step),
            slalom::resultName(g_reject_result));
    }
    LOG("fast %s: %s (rejected %lu, dropped %lu)\r\n", preset.name, ok ? "finished" : "stopped",
        static_cast<unsigned long>(planProfile.rejectedCount()), static_cast<unsigned long>(planProfile.droppedCount()));
    LOG("wall edge: %lu edges, correction %s, total shift %+.1f mm\r\n",
        static_cast<unsigned long>(wallEdge.eventCount()), preset.wall_edge ? "on" : "off", wallEdge.totalShift());
    wall_edge_log::dump("fast", g_edge_name);
    logger.dump();
    ledBar16.set(0x0000);
}
