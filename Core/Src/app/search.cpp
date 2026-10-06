#include "app/search.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "adachi.hpp"
#include "adachi_return.hpp"
#include "app/maze_store.hpp"
#include "app/search_lookahead.hpp"
#include "app/update.hpp"
#include "app/wall_edge_log.hpp"
#include "common/debug.hpp"
#include "common/etc.hpp"
#include "common/front_correction.hpp"
#include "common/trapezoid.hpp"
#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "device/uart.hpp"
#include "stm32f4xx_hal.h"

namespace {
using config::maze::CELL_MM;
using config::maze::WALL_HALF_MM;
constexpr float HALF_MM = CELL_MM / 2.f;

// 置いた位置（車軸）からスタート区画の前の境界まで。ソルバーの ACT_MOVE_FIRST_HALF_CELL にあたる
constexpr float START_TO_EDGE = CELL_MM - config::maze::START_MM;

constexpr uint32_t TRACE_MS = 68000;   // [ms] 時系列のログの長さ（往復の探索が入る。8列なら46msごと）
constexpr uint32_t SETTLE_MS = 500;

// ---- プリセットの検査（ビルド時）----
constexpr float accelDistance(const SearchPreset& p) {
    return p.speed * p.speed / (2.f * p.accel);
}

constexpr float pivotRamp(const PivotParam& p) {
    return p.omega * p.omega / (2.f * p.alpha);
}

// 集合のターン：使わない（nullptr）か，探索速度・同じファンの条件で設計されていて積める
constexpr bool turnUsable(const slalom::Param* t, float v, bool fan) {
    return t == nullptr || (t->speed == v && t->fan == fan && slalom::validate(*t) == SegmentResult::ok);
}

constexpr bool turnsUsable(const SearchPreset& p) {
    const float v = p.speed;
    const bool f = p.fan;
    const OrthoTurns& o = p.turns;
    bool ok = o.s90 != nullptr && o.s90->entry == slalom::Anchor::edge && o.s90->exit == slalom::Anchor::edge
           && turnUsable(o.s90, v, f) && turnUsable(o.l90, v, f) && turnUsable(o.t180, v, f);
    if (p.diagonal != nullptr) {
        const DiagonalTurns& d = *p.diagonal;
        ok = ok && turnUsable(d.in45, v, f) && turnUsable(d.out45, v, f) && turnUsable(d.v90, v, f)
                && turnUsable(d.in135, v, f) && turnUsable(d.out135, v, f);
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
        && p.pivot.alpha <= MAX_ALPHA && 2.f * pivotRamp(p.pivot) < 180.f
        // 既知の直進の加速：accel で上げ下げでき，最高速度から読んだ区画の中央より手前で止まれる（前壁に当たらない）
        && p.straight_speed >= v && config::search::BRAKE_DECEL <= MAX_DECEL_X
        && p.straight_speed * p.straight_speed / (2.f * config::search::BRAKE_DECEL) + config::search::READ_LEAD_MM
               < HALF_MM;
}

constexpr bool allPresetsRunnable() {
    for (const auto& p : config::search::PRESETS) {
        if (!presetRunnable(p)) return false;
    }
    for (const auto& p : config::search::TEST_PRESETS) {
        if (!presetRunnable(p)) return false;
    }
    return true;
}
static_assert(allPresetsRunnable(),
              "a search preset cannot run: check tools/search_presets.json against the profile limits");

// 前壁の補正で入口オフセットをこれより短くしない（1tick より短い区間は PlanProfile が tooShort で弾く）
constexpr float minPreMm(float v) {
    return 2.f * v * config::control::DT_S;
}

// 壁を読む位置が1歩の中にある（最も短い1歩は行き止まりの後半の半区画）
constexpr float READ_LEAD = config::search::READ_LEAD_MM;
static_assert(READ_LEAD > 0.f && READ_LEAD < HALF_MM, "config::search::READ_LEAD_MM must be inside half a cell");

// ---- 壁を読むたびの記録（シミュレータの replay.py の LOG_COLUMNS と同じ列）----
struct SearchStep {
    uint8_t x, y, dir;   // 壁を読んだときの mousePos（これから入る区画）
    uint8_t walls;       // bit0: 左, bit1: 前, bit2: 右（ソルバーに渡した壁）。bit4–6: センサーの壁（同じ並び）
    uint8_t action;      // ソルバーが返した動作（solver/core/action.h）
    uint8_t flags;       // bit0: 帰り探索中か, bit1: 地図の壁を使った, bit2: 読み直した後の読み,
                         // bit3–6: この読みで積んだ直進の区画数（加速した区間。積まなかった読みは0），
                         // bit7: この歩で迷路の保存を始めた（maze_store::journal に積んだ）
    int16_t ir[wall::POSITION_COUNT];
    int16_t front_err;    // [0.1 mm] S90 を積むときの前壁の距離による前後のずれ（common/front_correction.hpp）。
                          // 使えなかった・S90 でないときは FRONT_ERR_NONE。補正 δ は書き出すときに計算し直す
    float pos_target;     // [mm] 壁を読んだときの並進の目標位置（区画境界の READ_LEAD 手前に来たときの値）
    float pos_measured;   // [mm] そのときの実測（エンコーダ）。待つのはこちらで，差が追従遅れ
    uint16_t prepare_us;  // [us] この壁を読む前（前の動作を積んだ直後）に8通りの壁でソルバーを回した時間
    uint16_t take_us;     // [us] 壁を読んでから先読みの結果を取り出すまで（先読みがなければソルバーを呼んだ時間）
};

constexpr int16_t FRONT_ERR_NONE = INT16_MIN;
constexpr uint8_t FLAG_RETURNING = 1u << 0;
constexpr uint8_t FLAG_KNOWN = 1u << 1;
constexpr uint8_t FLAG_RECHECKED = 1u << 2;
constexpr uint8_t RUN_CELLS_SHIFT = 3;
constexpr uint8_t RUN_CELLS_MASK = 0x0F;
static_assert(MAZE_SIZE - 1 <= RUN_CELLS_MASK, "run cells must fit in SearchStep::flags");
constexpr uint8_t FLAG_SAVED = 1u << 7;
static_assert(sizeof(SearchStep) * config::search::MAX_STEPS <= 64 * 1024, "search step log exceeds CCMRAM");

// CCMRAM（64KB，ほかに使っていない）に置く。スタートアップは CCMRAM を0にしないので，
// 件数（g_step_count，通常のRAM）だけで有効な範囲を表す
__attribute__((section(".ccmram"))) SearchStep g_steps[config::search::MAX_STEPS];
uint16_t g_step_count = 0;

// 位置の2列は末尾に足している（replay.py は列を名前で読むので，知らない列は無視される）
constexpr const char* STEP_COLUMNS[] = {"step", "x", "y", "dir", "left", "front", "right", "action",
                                        "returning", "ir_l", "ir_fl", "ir_fr", "ir_r",
                                        "pos_target", "pos_measured", "front_err", "front_corr",
                                        "prepare_us", "take_us",
                                        "sensor_left", "sensor_front", "sensor_right", "known", "rechecked",
                                        "run_cells", "saved"};
constexpr uint32_t STEP_COLUMN_COUNT = sizeof(STEP_COLUMNS) / sizeof(STEP_COLUMNS[0]);

// ---- 走行中の迷路の保存（config::maze_save）----
struct MazeSave {
    uint8_t goal_x, goal_y;
    bool goal_reached = false;   // ゴールに着いた（帰り探索か，最短経路の確定の探索に入った）
    bool due = false;            // 次の機会に保存する
    uint16_t since = 0;          // 前に保存（または保存を見送り）してからの歩数
    uint16_t skipped = 0;        // 迷路が前の保存と同じで見送った回数
    uint16_t refused = 0;        // journal が受け付けなかった（面が一杯・open できなかった）回数
    uint32_t max_word_us = 0;    // 走行中に1語書くのにかかった最大 [us]（CPU が止まった時間）
};
MazeSave g_save;

// 1歩終えるごとに呼ぶ。ゴールに着いた歩で1回，その後は EVERY_STEPS 歩ごとに保存の機会にする
void updateSaveDue() {
    if (!adachi::search_returning() && !adachi::search_confirming()) return;
    if (!g_save.goal_reached) {
        g_save.goal_reached = true;
        g_save.due = true;
    } else if (++g_save.since >= config::maze_save::EVERY_STEPS) {
        g_save.due = true;
    }
}

// 今のソルバーの迷路を journal に積む（書くのは step()/flush()）。積んだら true
bool queueSave() {
    g_save.due = false;
    g_save.since = 0;
    maze_store::Record r = maze_store::capture(g_save.goal_x, g_save.goal_y, false);
    const maze_store::Record* prev = maze_store::journal::last();
    if (prev != nullptr && maze_store::sameMaze(*prev, r)) {
        ++g_save.skipped;
        return false;
    }
    if (!maze_store::journal::append(r)) {
        ++g_save.refused;
        return false;
    }
    return true;
}

// 走行中に1語だけ書く（CPU が止まる時間を測る）
void stepSave() {
    if (!maze_store::journal::busy()) return;
    uint32_t t0 = DWT->CYCCNT;
    maze_store::journal::step(1);
    uint32_t us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
    if (us > g_save.max_word_us) g_save.max_word_us = us;
}

enum class Stop : uint8_t {
    finished,       // スタートに戻った（正常）
    frontWall,      // 壁のある向きへ進もうとした（壁の誤読か，ソルバーの経路がない）
    pushRejected,   // 区間を積めなかった（加速度の上限など）
    profileError,   // PlanProfile が区間を落とした，または読む位置の前に止まった
    tooManySteps,   // MAX_STEPS を超えた（ログが入らない）
    unknownAction,  // 探索では使わない動作が来た
    planMismatch,   // 加速して積んだ区間の途中で，ソルバーが直進以外を返した（先読みと食い違った）
    positionLost,   // 読み直しても地図と食い違う壁が，1区画ずれた区画の地図と続けて一致した（自己位置を見失った）
};

const char* stopName(Stop s) {
    switch (s) {
    case Stop::finished:      return "finished";
    case Stop::frontWall:     return "wall ahead";
    case Stop::pushRejected:  return "segment rejected";
    case Stop::profileError:  return "profile error";
    case Stop::tooManySteps:  return "too many steps";
    case Stop::planMismatch:  return "plan mismatch";
    case Stop::positionLost:  return "position lost";
    default:                  return "unknown action";
    }
}

char g_log_name[24];
char g_trace_name[32];
char g_edge_name[32];

void initTraceLog() {
    logger.initLoggedVal();
    logger.add<&PlanProfile::getTargetPositionX>("target_distance_x", planProfile);
    logger.add<&Odometry::positionX>("current_distance_x", odometry);
    logger.add<&PlanProfile::getTargetVelocityX>("target_velocity_x", planProfile);
    logger.add<&PlanProfile::getTargetAngle>("target_angle", planProfile);
    logger.add<&Odometry::angle>("current_angle", odometry);
    logger.add<&WallControl::omega>("wall_omega", wallControl);
    logger.add<&WallControl::offset>("wall_offset", wallControl);
    logger.add<&WallEdge::totalShift>("edge_shift", wallEdge);
    logger.setDuration(TRACE_MS);
}

// 区間の計測に DWT のサイクルカウンタを使う（[us] で uint16_t に丸める）
void startCycleCounter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint16_t elapsedUs(uint32_t start_cycles) {
    uint32_t us = (DWT->CYCCNT - start_cycles) / (SystemCoreClock / 1000000u);
    return static_cast<uint16_t>(us > UINT16_MAX ? UINT16_MAX : us);
}

uint32_t clockUs() {
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

// 次に読む壁の仮定でソルバーを先に回し，かかった時間 [us] を返す。
// chains なら既知の区画が続く直進も数える（次の壁を読むまでの時間の CHAIN_BUDGET_RATIO まで）
uint16_t prepareNext(const SearchPreset& p, bool chains) {
    search_lookahead::PrepareOptions o;
    o.chains = chains;
    o.budget_us = static_cast<uint32_t>(config::search::CHAIN_BUDGET_RATIO * CELL_MM / p.speed * 1e6f);
    o.clock = clockUs;
    uint32_t t0 = DWT->CYCCNT;
    search_lookahead::prepare(o);
    return elapsedUs(t0);
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

// 台形（trapezoid::split）を並進の区間として積む。sign が負なら後ろへ（速度・距離の符号を反転）
bool pushTrapezoid(float d, float v_in, float v_out, float v_max, float accel, float sign) {
    trapezoid::Part parts[trapezoid::MAX_PARTS];
    uint8_t n = trapezoid::split(d, v_in, v_out, v_max, accel, accel, parts);
    if (planProfile.freeSlots() < n) return false;
    for (uint8_t k = 0; k < n; ++k) {
        if (planProfile.straight(sign * parts[k].v_end, sign * parts[k].distance) != SegmentResult::ok) return false;
    }
    return true;
}

// ---- 地図と食い違った壁の記録（終わりに LOG で出す）----
struct Mismatch {
    uint16_t step;          // その区画の行の番号（読み直した後の読みの行）
    uint8_t x, y, dir;
    uint8_t map_walls;      // 地図の壁（bit0: 左, bit1: 前, bit2: 右）
    uint8_t first_walls;    // 1回目のセンサーの壁
    uint8_t second_walls;   // 読み直したセンサーの壁（まだなら 0xFF）
    float speed;            // [mm/s] 1回目に読んだときの目標速度
};
Mismatch g_mismatches[config::search::MAX_MISMATCH_LOG];
uint16_t g_mismatch_count = 0;     // 食い違いの回数（記録しきれなかった分も数える）
uint16_t g_map_updates = 0;        // 読み直しても食い違い，センサーの壁で地図を書き換えた回数
uint16_t g_runs = 0;               // 加速して積んだ区間の数
uint16_t g_run_cells = 0;          // その区画数の合計

// ---- 前壁の読み落とし（config::search::MISSED_WALL_*）----
struct MissedWall {
    uint16_t step;              // 読み落とした歩の行の番号
    uint8_t x, y, dir;          // その区画（読み落とした前壁はこの区画の前）
    bool confirmed;             // 止まって測った前壁の距離で，壁ありと確かめた（false は誤検出として走り直した）
    int16_t trigger_fl, trigger_fr;   // 気づいたときの前左・前右の値
    int16_t still_fl, still_fr;       // 止まってからの値
    float speed;                // [mm/s] 気づいたときの目標速度
    float past_read;            // [mm] 気づいた実測位置が，読み落とした読みの位置から進んだ量
    float distance;             // [mm] 止まってから測った前壁の距離（換算表。NaN は表より遠い）
    float expected;             // [mm] 読み落とした境界に壁があるときの，止まった位置からの予想距離
    uint8_t action;             // やり直したソルバーの動作（誤検出なら ACT_NONE）
};
MissedWall g_missed[config::search::MAX_MISSED_WALL_LOG];
uint16_t g_missed_count = 0;   // 気づいた回数（記録しきれなかった分も数える）

// ---- 自己位置のずれの疑い（config::search::SHIFT_VOTES_TO_STOP）----
// 読み直しても食い違った壁が1区画先・手前の地図と，続けて読んだ区画で一致した回数（一致しなかった側と，
// そうでない読みで0に戻す）。hold_save は一致してから地図と一致する読みが来るまで（迷路を保存しない）
struct ShiftSuspect {
    uint8_t ahead = 0;
    uint8_t behind = 0;
    bool hold_save = false;
};
ShiftSuspect g_shift;
uint16_t g_shift_total = 0;   // 一致した回数の合計（ログ用）

// 3辺とも既知の区画で壁が地図と食い違った：止まって，1つ手前の区画の中央まで下がり，
// そこから探索速度まで加速して同じ境界へ向かう（戻るとループが同じ読む位置で壁を読み直す）。
// in_run：加速して積んだ区間の途中（その先まで積んである）か。
//
// 下がる先は今の境界 step_end の半区画手前。どの動作の後でも境界では区画の中心線の上にいて向きもそろっている
// （直進・小回り90°の出口・行き止まりの後の半区画）ので，まっすぐ下がればその区画の中央に着く
Stop recover(const SearchPreset& p, bool in_run, float step_end) {
    using namespace config::search;
    const float v = p.speed;
    if (in_run) {
        // 積んである加速区間を取り消し，今の速度から減速して止まる
        planProfile.brake(BRAKE_DECEL);
    } else if (planProfile.straight(0.f, v * v / (2.f * BRAKE_DECEL)) != SegmentResult::ok) {
        // 今の1歩は境界で終わり，その先は積んでいないので，境界の先で止まる区間を後ろに積む
        // （小回り90°の出口の途中でも回転を壊さない）
        return Stop::pushRejected;
    }
    planProfile.waitUntilIdle();
    if (profileBroken()) return Stop::profileError;

    // 下がっている間は横壁の制御・壁切れの検出を止める（待っている境界 step_end は残るので教え直さない）
    wallControl.enable(false);
    wallEdge.stop();
    HAL_Delay(RECHECK_SETTLE_MS);

    float back = planProfile.getTargetPositionX() - (step_end - HALF_MM);
    if (!(back > 0.f) || !pushTrapezoid(back, 0.f, 0.f, BACK_SPEED, BACK_ACCEL, -1.f)) return Stop::pushRejected;
    planProfile.waitUntilIdle();
    if (profileBroken()) return Stop::profileError;

    // 止まっている間に，読み直す区画の8通りで先読みを作り直す（時間があるので既知の直進も数える）
    search_lookahead::PrepareOptions o;
    o.chains = true;
    o.all = true;
    search_lookahead::prepare(o);
    HAL_Delay(RECHECK_SETTLE_MS);

    wallControl.enable(p.wall_control);
    wallEdge.start(config::wall_edge::SEARCH_CORRECTION);
    if (planProfile.straight(v, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
    return Stop::finished;   // 続けてよい
}

// 前壁の読み落とし：直進を選んだ歩の後，次の読む位置（境界 step_end の READ_LEAD 手前）に着く前に前左・前右の
// 両方が近く見えた（trigger）。急停止して前壁の距離を測り，境界 step_end に壁があると確かめたら，読んだ区画の中央へ
// 寄せ，前壁ありでソルバーをやり直して，超信地旋回で曲がる（戻る）。壁がなければ誤検出として同じ読む位置へ向かい直す。
// step_end・committed は書き換える。Stop::finished なら探索を続ける（ACT_FINISH のときは done も true）。
// wall_found は壁ありと確かめたか（false なら同じ直進の続き）
Stop handleMissedWall(const SearchPreset& p, const wall::Snapshot& trigger, float* step_end, uint8_t* committed,
                      bool* done, bool* wall_found) {
    using namespace config::search;
    const float v = p.speed;
    MissedWall m{};
    const SearchStep& last = g_steps[g_step_count - 1];   // 読み落とした歩（直進を選んだ）
    m.step = static_cast<uint16_t>(g_step_count - 1);
    m.x = last.x;
    m.y = last.y;
    m.dir = last.dir;
    m.trigger_fl = trigger.value[wall::front_left];
    m.trigger_fr = trigger.value[wall::front_right];
    m.speed = planProfile.getTargetVelocityX();
    m.past_read = odometry.positionX() - last.pos_measured;
    m.action = ACT_NONE;

    // 積んである区間（加速した直進の残りも）を取り消して止まる。壁の制御・壁切れは止め，教えた境界も消す
    planProfile.brake(MISSED_WALL_DECEL);
    planProfile.waitUntilIdle();
    if (profileBroken()) return Stop::profileError;
    *committed = 0;
    wallControl.enable(false);
    wallEdge.stop();
    wallEdge.dropPending();
    HAL_Delay(RECHECK_SETTLE_MS);

    wall::Snapshot s = wall::read();
    m.still_fl = s.value[wall::front_left];
    m.still_fr = s.value[wall::front_right];
    const float here = planProfile.getTargetPositionX();
    m.distance = front_correction::frontWallMm(s.value[wall::front_left], s.value[wall::front_right]);
    m.expected = *step_end - here - WALL_HALF_MM;
    m.confirmed = m.distance < m.expected + MISSED_WALL_CONFIRM_MM;   // NaN（表より遠い）は false
    *wall_found = m.confirmed;
    auto log = [&m]() {
        if (g_missed_count < MAX_MISSED_WALL_LOG) g_missed[g_missed_count] = m;
        ++g_missed_count;
    };

    if (!m.confirmed) {
        // 誤検出：読んだ区画の中央へ戻り（進み），同じ読む位置へ向かい直す（壁切れの境界は教え直す）
        log();
        float move = (*step_end - HALF_MM) - here;
        if (std::fabs(move) > 1.f && !pushTrapezoid(std::fabs(move), 0.f, 0.f, BACK_SPEED, BACK_ACCEL,
                                                    move > 0.f ? 1.f : -1.f)) {
            return Stop::pushRejected;
        }
        planProfile.waitUntilIdle();
        if (profileBroken()) return Stop::profileError;
        HAL_Delay(RECHECK_SETTLE_MS);
        wallControl.enable(p.wall_control);
        wallEdge.expect(*step_end);
        wallEdge.start(config::wall_edge::SEARCH_CORRECTION);
        if (planProfile.straight(v, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
        return Stop::finished;
    }

    // 壁あり：前壁の距離で読んだ区画の中央（前壁の面から半区画 − 壁の厚さの半分）へ寄せる（前後のずれも消える）
    float move = m.distance - (HALF_MM - WALL_HALF_MM);
    if (std::fabs(move) > 1.f && !pushTrapezoid(std::fabs(move), 0.f, 0.f, BACK_SPEED, BACK_ACCEL,
                                                move > 0.f ? 1.f : -1.f)) {
        return Stop::pushRejected;
    }
    planProfile.waitUntilIdle();
    if (profileBroken()) return Stop::profileError;
    const float center = planProfile.getTargetPositionX();

    // 読み落とした区画で，左右はその歩の壁・前は壁ありとしてソルバーをやり直す（地図の前壁も有りになる）
    const bool left = (last.walls & 1) != 0;
    const bool right = (last.walls & 4) != 0;
    const uint8_t action = search_lookahead::redoWithFrontWall(left, right);
    m.action = action;
    log();
    if (g_step_count < MAX_STEPS) {
        SearchStep& rec = g_steps[g_step_count++];
        const uint8_t sensor = search_lookahead::wallBits(wall::hasLeft(s), wall::hasFront(s), wall::hasRight(s));
        rec = {last.x, last.y, last.dir, static_cast<uint8_t>(search_lookahead::wallBits(left, true, right) |
                                                              (sensor << 4)),
               action, static_cast<uint8_t>(adachi_return::is_returning() ? FLAG_RETURNING : 0), {}, FRONT_ERR_NONE,
               planProfile.getTargetPositionX(), odometry.positionX(), 0, 0};
        for (uint8_t i = 0; i < wall::POSITION_COUNT; ++i) rec.ir[i] = s.value[i];
    }

    // 区画の中央にいるので，曲がる・戻るは超信地旋回の後に半区画で境界へ（行き止まりの後と同じく境界は教えない）
    float angle = 0.f;
    switch (action) {
    case ACT_TURN_LEFT_MOVE:  angle = 90.f; break;
    case ACT_TURN_RIGHT_MOVE: angle = -90.f; break;
    case ACT_TURN_BACK:       angle = 180.f; break;
    case ACT_FINISH:          *done = true; return Stop::finished;   // スタート区画の中央で止まっている
    default:                  return Stop::frontWall;
    }
    HAL_Delay(RECHECK_SETTLE_MS);
    wallControl.enable(p.wall_control);
    wallEdge.start(config::wall_edge::SEARCH_CORRECTION);
    if (!pivot(p.pivot, angle) || planProfile.straight(v, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
    *step_end = center + HALF_MM;   // 旋回では経路に沿った距離は変わらない
    return Stop::finished;
}

// 止まって書くとき（config::maze_save::WHILE_RUNNING = false）：区画中央まで減速して止まり，迷路を書く。
// 呼んだ後は速度0から積み直す。saved には保存を始めたか（迷路が変わらず見送れば false）を返す
bool stopAndSave(bool* saved) {
    if (planProfile.straight(0.f, HALF_MM) != SegmentResult::ok) return false;
    while (!planProfile.isIdle()) {
        if (profileBroken()) return false;
    }
    // 止まっていても制御の割り込みは回っているので，1語ずつ書く（まとめて書くと最大 4.5ms 割り込みが止まる）
    *saved = queueSave();
    while (maze_store::journal::busy()) stepSave();
    return true;
}

// 探索の本体。PlanProfile に区間を積みながら，壁を読む位置ごとに先読みしておいたソルバーの結果を取り出す
Stop runSteps(const SearchPreset& p) {
    const float v = p.speed;
    const bool accelerate = p.straight_speed > v;

    solver_options_reset();
    solver_options.goal_x = p.goal_x;
    solver_options.goal_y = p.goal_y;
    uint8_vector first = adachi_return::solver_adachi_return_init();
    if (search_lookahead::firstMotion(first) != ACT_MOVE_FIRST_HALF_CELL) return Stop::unknownAction;

    float d_acc = accelDistance(p);
    if (planProfile.straight(v, d_acc) != SegmentResult::ok ||
        planProfile.straight(v, START_TO_EDGE - d_acc) != SegmentResult::ok) {
        return Stop::pushRejected;
    }
    float step_end = START_TO_EDGE;   // 今積んでいる1歩が終わる位置（並進の目標位置，区画境界）
    wallEdge.expect(step_end);
    uint8_t committed = 0;    // 加速して積み済みで，動作を積まない読みの残り
    bool rechecked = false;   // 今の区画は止まって読み直した後か
    int16_t mismatch_slot = -1;   // 読み直した区画の g_mismatches の番号
    bool watch_front = false;     // 直進を選んだ歩の後：次の読む位置までに前壁が近く見えたら読み落とし
    // 最初の区画 (0,1) で読む壁の8通りを，最初の半区画を走っている間に計算しておく
    uint16_t prepare_us = prepareNext(p, accelerate);

    while (true) {
        // 機体（実測）が区画境界の READ_LEAD 手前に来るまで待つ（今の1歩の残りが走っている間に次を積む）。
        // 目標位置で待つと，追従遅れのぶん実際の機体より先で壁を読んだことになる。
        // 実測も目標も経路に沿った距離（超信地旋回では変わらない）で，step_end と同じ座標
        // 待っている間に，積んである迷路を1語ずつフラッシュに書く（config::maze_save::WHILE_RUNNING）
        bool missed = false;
        wall::Snapshot trigger{};
        while (odometry.positionX() < step_end - READ_LEAD) {
            if (profileBroken() || planProfile.isIdle()) return Stop::profileError;
            stepSave();
            if (watch_front) {
                trigger = wall::read();
                if (front_correction::bothCloserThan(trigger.value[wall::front_left],
                                                     trigger.value[wall::front_right],
                                                     config::search::MISSED_WALL_NEAR_MM)) {
                    missed = true;
                    break;
                }
            }
        }
        if (missed) {
            bool done = false, wall_found = false;
            Stop r = handleMissedWall(p, trigger, &step_end, &committed, &done, &wall_found);
            if (r != Stop::finished || done) return r;
            // 誤検出なら同じ読む位置へ向かい直すが，同じ所でまた止まり続けないよう，この歩はもう見張らない
            watch_front = false;
            rechecked = false;
            if (wall_found) prepare_us = prepareNext(p, accelerate);   // ソルバーをやり直したので先読みし直す
            continue;
        }
        float pos_target = planProfile.getTargetPositionX();
        float pos_measured = odometry.positionX();

        wall::Snapshot s = wall::read();
        const uint8_t sensor = search_lookahead::wallBits(wall::hasLeft(s), wall::hasFront(s), wall::hasRight(s));
        // 3辺とも既知なら，探索は地図の壁を使う（先読みの既知の直進もこの約束で数えている）
        bool map_l = false, map_f = false, map_r = false;
        const bool known = search_lookahead::knownWalls(&map_l, &map_f, &map_r);
        const uint8_t map = search_lookahead::wallBits(map_l, map_f, map_r);
        MousePos at = mousePos;   // 壁を読んだ区画（ソルバーが次の区画へ進める前）

        search_lookahead::WallDecision decision = search_lookahead::decide(known, map, sensor, rechecked);
        if (decision == search_lookahead::WallDecision::recheck) {
            if (g_mismatch_count < config::search::MAX_MISMATCH_LOG) {
                mismatch_slot = static_cast<int16_t>(g_mismatch_count);
                g_mismatches[mismatch_slot] = {g_step_count, at.x, at.y, at.dir, map, sensor, 0xFF,
                                               planProfile.getTargetVelocityX()};
            } else {
                mismatch_slot = -1;
            }
            ++g_mismatch_count;
            Stop r = recover(p, committed > 0, step_end);
            if (r != Stop::finished) return r;
            committed = 0;
            rechecked = true;
            continue;   // 同じ読む位置まで来たら読み直す
        }
        const bool use_map = (decision == search_lookahead::WallDecision::useMap);
        const uint8_t walls = use_map ? map : sensor;
        const bool left = (walls & 1) != 0;
        const bool front = (walls & 2) != 0;
        const bool right = (walls & 4) != 0;
        if (rechecked) {
            if (mismatch_slot >= 0) g_mismatches[mismatch_slot].second_walls = sensor;
            if (!use_map) {
                // 読み直しても食い違う壁が1区画ずれた区画の地図と一致するなら，自己位置がずれている疑い。
                // 同じ向きに続けて一致したら止まる（地図を書き換える前に。その後の迷路は保存しない）
                const uint8_t m = search_lookahead::shiftMatch(sensor);
                g_shift.ahead = (m & search_lookahead::SHIFT_AHEAD) ? g_shift.ahead + 1 : 0;
                g_shift.behind = (m & search_lookahead::SHIFT_BEHIND) ? g_shift.behind + 1 : 0;
                if (m != 0) {
                    ++g_shift_total;
                    g_shift.hold_save = true;
                }
                if (g_shift.ahead >= config::search::SHIFT_VOTES_TO_STOP ||
                    g_shift.behind >= config::search::SHIFT_VOTES_TO_STOP) {
                    planProfile.brake(config::search::BRAKE_DECEL);
                    planProfile.waitUntilIdle();
                    return Stop::positionLost;
                }
                ++g_map_updates;   // センサーの壁で地図が書き換わる（search_step の set_wall）
            }
        }
        if (!rechecked || use_map) {   // 続けて一致しなかった
            g_shift.ahead = 0;
            g_shift.behind = 0;
        }
        if (use_map) g_shift.hold_save = false;   // 地図と一致した：位置は合っている

        // 先に計算しておいた仮定から，渡す壁の結果を取り出す（ソルバーの状態もその1歩の後になる）
        uint32_t t_take = DWT->CYCCNT;
        uint8_t action = search_lookahead::take(left, front, right);
        uint16_t take_us = elapsedUs(t_take);

        if (g_step_count >= config::search::MAX_STEPS) return Stop::tooManySteps;
        SearchStep& rec = g_steps[g_step_count++];
        uint8_t flags = static_cast<uint8_t>((adachi_return::is_returning() ? FLAG_RETURNING : 0) |
                                             (use_map ? FLAG_KNOWN : 0) | (rechecked ? FLAG_RECHECKED : 0));
        rec = {at.x, at.y, at.dir, static_cast<uint8_t>(walls | (sensor << 4)), action, flags, {}, FRONT_ERR_NONE,
               pos_target, pos_measured, prepare_us, take_us};
        for (uint8_t i = 0; i < wall::POSITION_COUNT; ++i) rec.ir[i] = s.value[i];
        rechecked = false;

        // 加速した区間の中の読みも1歩と数える（保存を積むのは，その区間を抜けて動作を積む歩）
        updateSaveDue();
        watch_front = (action == ACT_MOVE_1CELL);
        if (committed > 0) {
            // 加速して積んだ区間の途中：動作は積まず，境界を進めるだけ（ソルバーの答えは直進のはず）
            if (action != ACT_MOVE_1CELL) return Stop::planMismatch;
            step_end += CELL_MM;
            wallEdge.expect(step_end);
            --committed;
            prepare_us = prepareNext(p, accelerate && committed == 0);
            continue;
        }

        // 止まって書くなら，直進・行き止まりの歩で区画中央に止まって書く（ターンの歩なら次の機会へ）
        bool stop_to_save = !config::maze_save::WHILE_RUNNING && g_save.due && !g_shift.hold_save &&
                            (action == ACT_MOVE_1CELL || action == ACT_TURN_BACK);

        switch (action) {
        case ACT_MOVE_1CELL: {
            if (front) return Stop::frontWall;
            // 続く区画の壁が既知で直進が決まっていれば，その区画数を1本の台形で積む（止まって書く歩は加速しない）
            uint8_t cells = (accelerate && !stop_to_save) ? search_lookahead::straightCells() : 1;
            if (stop_to_save) {
                bool saved = false;
                if (!stopAndSave(&saved)) return Stop::pushRejected;
                if (saved) rec.flags |= FLAG_SAVED;
                if (planProfile.straight(v, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
            } else if (cells >= 2) {
                if (!pushTrapezoid(cells * CELL_MM, v, v, p.straight_speed, p.accel, 1.f)) return Stop::pushRejected;
                committed = static_cast<uint8_t>(cells - 1);
                rec.flags = static_cast<uint8_t>(rec.flags | ((cells & RUN_CELLS_MASK) << RUN_CELLS_SHIFT));
                ++g_runs;
                g_run_cells = static_cast<uint16_t>(g_run_cells + cells);
            } else if (planProfile.straight(v, CELL_MM) != SegmentResult::ok) {
                return Stop::pushRejected;
            }
            step_end += CELL_MM;
            wallEdge.expect(step_end);   // 直進で着く境界だけ（ターンの出口では壁切れを使わない）
            break;
        }
        case ACT_TURN_LEFT_MOVE:
        case ACT_TURN_RIGHT_MOVE: {
            bool to_left = (action == ACT_TURN_LEFT_MOVE);
            if (to_left ? left : right) return Stop::frontWall;
            auto dir = to_left ? slalom::TurnDir::left : slalom::TurnDir::right;
            const slalom::Param& s90 = *p.turns.s90;
            // 前壁があれば，前のターンの出口のずれを前壁の距離から求めて入口（pre-offset）を直す。
            // 補正しないプリセットでもずれは記録する
            float pre_adjust = 0.f;
            if (front) {
                float error = front_correction::estimateError(
                    s.value[wall::front_left], s.value[wall::front_right], pos_measured - (step_end - READ_LEAD));
                if (error == error) rec.front_err = static_cast<int16_t>(std::lround(error * 10.f));
                float delta = front_correction::correction(error);
                if (p.front_correction && delta != 0.f) {
                    front_correction::Split sp =
                        front_correction::split(delta, s90.motion(dir).pre_offset, minPreMm(v));
                    pre_adjust = sp.pre_adjust;
                    if (sp.position_shift != 0.f && !odometry.requestShiftX(sp.position_shift)) {
                        return Stop::profileError;
                    }
                }
            }
            if (slalom::push(planProfile, s90, dir, pre_adjust) != SegmentResult::ok) return Stop::pushRejected;
            step_end += slalom::totalDistance(s90, dir) + pre_adjust;
            break;
        }
        case ACT_TURN_BACK:
            // 区画中央で止まり，その場で180°回って，来た境界へ戻る（止まって書くなら，止まったところで書く）
            if (stop_to_save) {
                bool saved = false;
                if (!stopAndSave(&saved)) return Stop::pushRejected;
                if (saved) rec.flags |= FLAG_SAVED;
            } else if (planProfile.straight(0.f, HALF_MM) != SegmentResult::ok) {
                return Stop::pushRejected;
            }
            if (!pivot(p.pivot, 180.f) || planProfile.straight(v, HALF_MM) != SegmentResult::ok) {
                return Stop::pushRejected;
            }
            step_end += CELL_MM;
            // 戻った先の境界は教えない：横のセンサーは約 100 mm 先を見ているので，その境界の壁切れは
            // 旋回する区画中央（境界の半区画手前）より手前で起き，旋回の後には来ない。教えると，中央へ止まりに
            // 行く間の壁切れが対応づいて間違った補正になる（2026-10-03 の探索で −23.8 mm）
            break;
        case ACT_FINISH:
            // スタート区画の中央で止まる
            if (planProfile.straight(0.f, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
            return Stop::finished;
        default:
            return Stop::unknownAction;
        }

        // 次の区画へ走っている間に，そこで読む壁の仮定でソルバーを回しておく
        // （加速して積んだ区間の中の読みは答えが決まっているので，直進を数えない）
        prepare_us = prepareNext(p, accelerate && committed == 0);
        // 走りながら書くなら，ここで迷路を積み，次の壁を読むまでの待ちで1語ずつ書く。前の記録を書き終えて
        // いなければ次の歩に回す（prepare() はソルバーの状態を元に戻すので，積むのはこの歩の後の迷路）
        if (config::maze_save::WHILE_RUNNING && g_save.due && !g_shift.hold_save && !maze_store::journal::busy() &&
            queueSave()) {
            rec.flags |= FLAG_SAVED;
        }
    }
}

// corrected: 前壁の補正をかけたか（プリセットの front_correction）。かけなければ front_corr 列は 0
void dumpSteps(const char* file, bool corrected) {
    bin_table::begin("search", file, false, g_step_count * STEP_COLUMN_COUNT * sizeof(float),
                     STEP_COLUMNS, STEP_COLUMN_COUNT);
    for (uint16_t i = 0; i < g_step_count; ++i) {
        const SearchStep& r = g_steps[i];
        float front_err = (r.front_err == FRONT_ERR_NONE) ? NAN : static_cast<float>(r.front_err) / 10.f;
        float row[STEP_COLUMN_COUNT] = {
            static_cast<float>(i), static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.dir),
            static_cast<float>(r.walls & 1), static_cast<float>((r.walls >> 1) & 1),
            static_cast<float>((r.walls >> 2) & 1), static_cast<float>(r.action),
            static_cast<float>(r.flags & FLAG_RETURNING),
            static_cast<float>(r.ir[wall::left]), static_cast<float>(r.ir[wall::front_left]),
            static_cast<float>(r.ir[wall::front_right]), static_cast<float>(r.ir[wall::right]),
            r.pos_target, r.pos_measured,
            front_err, corrected ? front_correction::correction(front_err) : 0.f,
            static_cast<float>(r.prepare_us), static_cast<float>(r.take_us),
            static_cast<float>((r.walls >> 4) & 1), static_cast<float>((r.walls >> 5) & 1),
            static_cast<float>((r.walls >> 6) & 1), static_cast<float>((r.flags & FLAG_KNOWN) ? 1 : 0),
            static_cast<float>((r.flags & FLAG_RECHECKED) ? 1 : 0),
            static_cast<float>((r.flags >> RUN_CELLS_SHIFT) & RUN_CELLS_MASK),
            static_cast<float>((r.flags & FLAG_SAVED) ? 1 : 0),
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
    std::snprintf(g_edge_name, sizeof(g_edge_name), "%s_edges", preset.name);
    LOG("search %s: %.0f mm/s (known straights %.0f mm/s), turn %s, fan %s, wall control %s, front correction %s, goal (%u,%u)\r\n",
        preset.name, preset.speed, preset.straight_speed, preset.turns.s90->name, preset.fan ? "on" : "off",
        preset.wall_control ? "on" : "off", preset.front_correction ? "on" : "off", preset.goal_x, preset.goal_y);

    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    float v0 = battery.voltage();
    if (v0 < config::search::MIN_BATTERY_V) {
        LOG("search not started: battery %.2f V < %.2f V\r\n", v0, config::search::MIN_BATTERY_V);
        blinkRefused();
        return;
    }

    // 迷路を追記する面を用意する。空きが足りなければ面を消す（1〜2s CPU が止まる。止まっている今のうちに）
    g_save = MazeSave{};
    g_save.goal_x = preset.goal_x;
    g_save.goal_y = preset.goal_y;
    bool erased = false;
    ledBar16.set(0xFFFF);
    maze_store::Result opened = maze_store::journal::open(config::maze_save::RESERVE_SLOTS, &erased);
    ledBar16.set(0x0000);
    LOG("maze journal: %s%s, %lu free slots\r\n", maze_store::resultName(opened), erased ? " (bank erased)" : "",
        static_cast<unsigned long>(maze_store::journal::freeSlots()));

    // IMU校正はファンを回す前に行う（振動がジャイロのオフセット推定に乗らないように。runClosedLoopTest と同じ）
    imu.calibrate();
    HAL_Delay(1100);
    if (preset.fan) {
        fan.setDuty(config::fan::RUN_DUTY);
        HAL_Delay(config::fan::SPINUP_MS);
    }

    initTraceLog();
    logger.setDirName("search");
    logger.setFileName(g_trace_name);
    logger.setIncludeTimestamp(false);

    // 原点の取り直し（runClosedLoopTest と同じ）と，横壁の補正の初期化
    odometry.reset();
    planProfile.reset();
    wallControl.reset();
    wallControl.enable(preset.wall_control);
    wallEdge.reset();
    wallEdge.start(config::wall_edge::SEARCH_CORRECTION);
    g_step_count = 0;
    g_mismatch_count = 0;
    g_map_updates = 0;
    g_runs = 0;
    g_run_cells = 0;
    g_missed_count = 0;
    g_shift = ShiftSuspect{};
    g_shift_total = 0;
    startCycleCounter();
    control_timing::reset();

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
    fan.stop();
    wallControl.enable(false);
    wallEdge.stop();
    logger.stop();
    motorDriver.setBreak();

    // 結果は機体を持ち上げた後（haltByAccZ の後）に UART へ出す。止まった直後はまだ床の上で，
    // 持ち上げて置くまで受け手がいない／つないでいないことがあるので，ここでは保存と LED の合図だけにする
    // 書きかけの記録を書き終える（モーターは止めてある）。途中で止まったときは，ゴールの後に保存した
    // 最後の迷路が最短走行で使われる（止まった歩は壁の誤読があり得るので，その後の迷路は保存しない）
    uint32_t gap_us = control_timing::maxGapUs();
    maze_store::Result pending = maze_store::journal::flush();
    if (stop == Stop::finished) {
        // スタートまで戻った迷路を「探索し終えた」として保存する。面が一杯などで積めなければ，面を消して書く
        ledBar16.set(0xFFFF);
        maze_store::Record done = maze_store::capture(preset.goal_x, preset.goal_y, true);
        pending = maze_store::journal::append(done) ? maze_store::journal::flush() : maze_store::save(done);
        ledBar16.set(0x0000);
    }
    maze_store::journal::close();
    if (stop != Stop::finished || pending != maze_store::Result::ok) blinkRefused();

    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();

    LOG("search %s: %s after %u wall reads (rejected %lu, dropped %lu)\r\n", preset.name, stopName(stop),
        g_step_count, static_cast<unsigned long>(planProfile.rejectedCount()),
        static_cast<unsigned long>(planProfile.droppedCount()));
    LOG("wall edge: %lu edges, correction %s, total shift %+.1f mm\r\n",
        static_cast<unsigned long>(wallEdge.eventCount()), config::wall_edge::SEARCH_CORRECTION ? "on" : "off",
        wallEdge.totalShift());
    LOG("known straights: %u runs, %u cells; wall mismatches %u (map rewritten %u)\r\n", g_runs, g_run_cells,
        g_mismatch_count, g_map_updates);
    for (uint16_t i = 0; i < g_mismatch_count && i < config::search::MAX_MISMATCH_LOG; ++i) {
        const Mismatch& m = g_mismatches[i];
        // 壁は LFR の順に 1/0。読み直しの前に止まったときは second が "-"
        auto bits = [](uint8_t w, char* out) {
            if (w == 0xFF) { out[0] = out[1] = out[2] = '-'; }
            else { out[0] = (w & 1) ? '1' : '0'; out[1] = (w & 2) ? '1' : '0'; out[2] = (w & 4) ? '1' : '0'; }
            out[3] = '\0';
        };
        char mw[4], fw[4], sw[4];
        bits(m.map_walls, mw);
        bits(m.first_walls, fw);
        bits(m.second_walls, sw);
        LOG("  mismatch step %u (%u,%u) dir %u at %.0f mm/s: map %s, first %s, second %s\r\n", m.step, m.x, m.y,
            m.dir, m.speed, mw, fw, sw);
    }
    LOG("missed front walls: %u; position shift matches %u (ahead %u, behind %u in a row at the end)\r\n",
        g_missed_count, g_shift_total, g_shift.ahead, g_shift.behind);
    for (uint16_t i = 0; i < g_missed_count && i < config::search::MAX_MISSED_WALL_LOG; ++i) {
        const MissedWall& m = g_missed[i];
        LOG("  missed wall step %u (%u,%u) dir %u at %.0f mm/s, %.0f mm past read: FL/FR %d/%d, still %d/%d, "
            "front %.1f mm (expected %.1f) -> %s, action %u\r\n",
            m.step, m.x, m.y, m.dir, m.speed, m.past_read, m.trigger_fl, m.trigger_fr, m.still_fl, m.still_fr,
            m.distance, m.expected, m.confirmed ? "wall" : "no wall", m.action);
    }
    uint8_t bank = 0;
    const maze_store::Record* r = maze_store::latest(&bank);
    LOG("maze save: %s, latest bank %c sequence %lu %s; %lu records (%lu failed, %u unchanged, %u refused), "
        "word max %lu us, control gap max %lu us\r\n",
        maze_store::resultName(pending), 'A' + bank, static_cast<unsigned long>(r != nullptr ? r->sequence : 0),
        r == nullptr ? "-" : ((r->flags & maze_store::FLAG_COMPLETE) ? "complete" : "partial"),
        static_cast<unsigned long>(maze_store::journal::appendedCount()),
        static_cast<unsigned long>(maze_store::journal::failedCount()), g_save.skipped, g_save.refused,
        static_cast<unsigned long>(g_save.max_word_us), static_cast<unsigned long>(gap_us));
    dumpSteps(g_log_name, preset.front_correction);
    wall_edge_log::dump("search", g_edge_name);
    logger.dump();
    ledBar16.set(0x0000);
}
