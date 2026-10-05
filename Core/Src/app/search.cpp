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
#include "common/wall_sensor.hpp"
#include "device/device_instance.hpp"
#include "device/uart.hpp"
#include "stm32f4xx_hal.h"

namespace {
using config::maze::CELL_MM;
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
        && p.pivot.alpha <= MAX_ALPHA && 2.f * pivotRamp(p.pivot) < 180.f;
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
    uint8_t walls;       // bit0: 左, bit1: 前, bit2: 右
    uint8_t action;      // ソルバーが返した動作（solver/core/action.h）
    uint8_t flags;       // STEP_RETURNING | STEP_SAVED
    int16_t ir[wall::POSITION_COUNT];
    int16_t front_err;    // [0.1 mm] S90 を積むときの前壁の距離による前後のずれ（common/front_correction.hpp）。
                          // 使えなかった・S90 でないときは FRONT_ERR_NONE。補正 δ は書き出すときに計算し直す
    float pos_target;     // [mm] 壁を読んだときの並進の目標位置（区画境界の READ_LEAD 手前に来たときの値）
    float pos_measured;   // [mm] そのときの実測（エンコーダ）。待つのはこちらで，差が追従遅れ
    uint16_t prepare_us;  // [us] この壁を読む前（前の動作を積んだ直後）に8通りの壁でソルバーを回した時間
    uint16_t take_us;     // [us] 壁を読んでから先読みの結果を取り出すまで（先読みがなければソルバーを呼んだ時間）
};

constexpr int16_t FRONT_ERR_NONE = INT16_MIN;
constexpr uint8_t STEP_RETURNING = 1;   // 帰り探索中
constexpr uint8_t STEP_SAVED = 2;       // この歩で迷路の保存を始めた（maze_store::journal に積んだ）
static_assert(sizeof(SearchStep) * config::search::MAX_STEPS <= 64 * 1024, "search step log exceeds CCMRAM");

// CCMRAM（64KB，ほかに使っていない）に置く。スタートアップは CCMRAM を0にしないので，
// 件数（g_step_count，通常のRAM）だけで有効な範囲を表す
__attribute__((section(".ccmram"))) SearchStep g_steps[config::search::MAX_STEPS];
uint16_t g_step_count = 0;

// 位置の2列は末尾に足している（replay.py は列を名前で読むので，知らない列は無視される）
constexpr const char* STEP_COLUMNS[] = {"step", "x", "y", "dir", "left", "front", "right", "action",
                                        "returning", "ir_l", "ir_fl", "ir_fr", "ir_r",
                                        "pos_target", "pos_measured", "front_err", "front_corr",
                                        "prepare_us", "take_us", "saved"};
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

// 次に読む壁の8通りでソルバーを先に回し，かかった時間 [us] を返す
uint16_t prepareNext() {
    uint32_t t0 = DWT->CYCCNT;
    search_lookahead::prepare();
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
    // 最初の区画 (0,1) で読む壁の8通りを，最初の半区画を走っている間に計算しておく
    uint16_t prepare_us = prepareNext();

    while (true) {
        // 機体（実測）が区画境界の READ_LEAD 手前に来るまで待つ（今の1歩の残りが走っている間に次を積む）。
        // 目標位置で待つと，追従遅れのぶん実際の機体より先で壁を読んだことになる。
        // 実測も目標も経路に沿った距離（超信地旋回では変わらない）で，step_end と同じ座標
        // 待っている間に，積んである迷路を1語ずつフラッシュに書く（config::maze_save::WHILE_RUNNING）
        while (odometry.positionX() < step_end - READ_LEAD) {
            if (profileBroken() || planProfile.isIdle()) return Stop::profileError;
            stepSave();
        }
        float pos_target = planProfile.getTargetPositionX();
        float pos_measured = odometry.positionX();

        wall::Snapshot s = wall::read();
        bool left = wall::hasLeft(s);
        bool front = wall::hasFront(s);
        bool right = wall::hasRight(s);
        MousePos at = mousePos;   // 壁を読んだ区画（ソルバーが次の区画へ進める前）
        // 先に計算しておいた8通りから，読んだ壁の結果を取り出す（ソルバーの状態もその1歩の後になる）
        uint32_t t_take = DWT->CYCCNT;
        uint8_t action = search_lookahead::take(left, front, right);
        uint16_t take_us = elapsedUs(t_take);

        if (g_step_count >= config::search::MAX_STEPS) return Stop::tooManySteps;
        SearchStep& rec = g_steps[g_step_count++];
        rec = {at.x, at.y, at.dir,
               static_cast<uint8_t>((left ? 1 : 0) | (front ? 2 : 0) | (right ? 4 : 0)),
               action, static_cast<uint8_t>(adachi_return::is_returning() ? STEP_RETURNING : 0), {}, FRONT_ERR_NONE,
               pos_target, pos_measured, prepare_us, take_us};
        for (uint8_t i = 0; i < wall::POSITION_COUNT; ++i) rec.ir[i] = s.value[i];

        updateSaveDue();
        // 止まって書くなら，直進・行き止まりの歩で区画中央に止まって書く（ターンの歩なら次の機会へ）
        bool stop_to_save = !config::maze_save::WHILE_RUNNING && g_save.due &&
                            (action == ACT_MOVE_1CELL || action == ACT_TURN_BACK);

        switch (action) {
        case ACT_MOVE_1CELL:
            if (front) return Stop::frontWall;
            if (stop_to_save) {
                bool saved = false;
                if (!stopAndSave(&saved)) return Stop::pushRejected;
                if (saved) rec.flags |= STEP_SAVED;
                if (planProfile.straight(v, HALF_MM) != SegmentResult::ok) return Stop::pushRejected;
            } else if (planProfile.straight(v, CELL_MM) != SegmentResult::ok) {
                return Stop::pushRejected;
            }
            step_end += CELL_MM;
            wallEdge.expect(step_end);   // 直進で着く境界だけ（ターンの出口では壁切れを使わない）
            break;
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
                if (saved) rec.flags |= STEP_SAVED;
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

        // 次の区画へ走っている間に，そこで読む壁の8通りでソルバーを回しておく
        prepare_us = prepareNext();
        // 走りながら書くなら，ここで迷路を積み，次の壁を読むまでの待ちで1語ずつ書く。前の記録を書き終えて
        // いなければ次の歩に回す（prepare() はソルバーの状態を元に戻すので，積むのはこの歩の後の迷路）
        if (config::maze_save::WHILE_RUNNING && g_save.due && !maze_store::journal::busy() && queueSave()) {
            rec.flags |= STEP_SAVED;
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
            static_cast<float>((r.flags & STEP_RETURNING) ? 1 : 0),
            static_cast<float>(r.ir[wall::left]), static_cast<float>(r.ir[wall::front_left]),
            static_cast<float>(r.ir[wall::front_right]), static_cast<float>(r.ir[wall::right]),
            r.pos_target, r.pos_measured,
            front_err, corrected ? front_correction::correction(front_err) : 0.f,
            static_cast<float>(r.prepare_us), static_cast<float>(r.take_us),
            static_cast<float>((r.flags & STEP_SAVED) ? 1 : 0),
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
    LOG("search %s: %.0f mm/s, turn %s, fan %s, wall control %s, front correction %s, goal (%u,%u)\r\n",
        preset.name, preset.speed, preset.turns.s90->name, preset.fan ? "on" : "off",
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

    LOG("search %s: %s after %u wall reads (rejected %lu, dropped %lu)\r\n", preset.name, stopName(stop),
        g_step_count, static_cast<unsigned long>(planProfile.rejectedCount()),
        static_cast<unsigned long>(planProfile.droppedCount()));
    LOG("wall edge: %lu edges, correction %s, total shift %+.1f mm\r\n",
        static_cast<unsigned long>(wallEdge.eventCount()), config::wall_edge::SEARCH_CORRECTION ? "on" : "off",
        wallEdge.totalShift());
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
    uint8_t bank = 0;
    const maze_store::Record* r = maze_store::latest(&bank);
    LOG("maze save: %s, latest bank %c sequence %lu %s; %lu records (%lu failed, %u unchanged, %u refused), "
        "word max %lu us, control gap max %lu us\r\n",
        maze_store::resultName(pending), 'A' + bank, static_cast<unsigned long>(r != nullptr ? r->sequence : 0),
        r == nullptr ? "-" : ((r->flags & maze_store::FLAG_COMPLETE) ? "complete" : "partial"),
        static_cast<unsigned long>(maze_store::journal::appendedCount()),
        static_cast<unsigned long>(maze_store::journal::failedCount()), g_save.skipped, g_save.refused,
        static_cast<unsigned long>(g_save.max_word_us), static_cast<unsigned long>(gap_us));
    maze_store::journal::close();
    if (stop != Stop::finished || pending != maze_store::Result::ok) blinkRefused();

    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    dumpSteps(g_log_name, preset.front_correction);
    wall_edge_log::dump("search", g_edge_name);
    logger.dump();
    ledBar16.set(0x0000);
}
