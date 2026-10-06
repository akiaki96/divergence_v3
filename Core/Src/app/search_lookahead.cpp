#include "app/search_lookahead.hpp"
#include <cstddef>
#include "adachi.hpp"

namespace search_lookahead {
namespace {
constexpr uint8_t HYPOTHESIS_COUNT = 8;
// 直進が続くのは迷路の端から端まで
constexpr uint8_t MAX_STRAIGHT_CELLS = MAZE_SIZE - 1;

struct Entry {
    adachi::SearchState after;   // その壁の組で1歩進めた後のソルバーの状態
    uint8_t action;              // その最初の動作
    uint8_t straight_cells;      // その動作から続く直進の区画数（直進でなければ0）
    bool valid;                  // この仮定を回したか
};

// 合わせて約1.2KB。CCMRAM は NOLOAD（スタートアップで0にしない）だが，prepare() で書いてから読むので使える。
// 有効かどうかは通常のRAMの g_ready（と prepare() が書く valid）で表す（ホストの試験ではふつうの変数）
#ifdef __arm__
#define LOOKAHEAD_CCMRAM __attribute__((section(".ccmram")))
#else
#define LOOKAHEAD_CCMRAM
#endif
LOOKAHEAD_CCMRAM Entry g_entries[HYPOTHESIS_COUNT];
LOOKAHEAD_CCMRAM adachi::SearchState g_base;   // prepare() の前の状態（仮定ごとにここへ戻してから回す）
bool g_ready = false;
uint8_t g_taken_straight = 0;

bool timeUp(const PrepareOptions& o, uint32_t t0) {
    return o.clock != nullptr && o.clock() - t0 >= o.budget_us;
}

// g_base から仮定 i の壁で1歩回して entry i に取っておく。chains なら続く直進を数える
void run(uint8_t i, const PrepareOptions& o, uint32_t t0) {
    adachi::search_restore(g_base);
    Entry& e = g_entries[i];
    e.action = firstMotion(adachi_return::solver_adachi_return((i & 1) != 0, (i & 2) != 0, (i & 4) != 0));
    adachi::search_save(&e.after);
    e.valid = true;
    e.straight_cells = (e.action == ACT_MOVE_1CELL) ? 1 : 0;
    if (!o.chains || e.action != ACT_MOVE_1CELL) return;
    // 次の区画からも，壁が既知なら探索はその値を使うので，ソルバーの答えは決まっている
    while (e.straight_cells < MAX_STRAIGHT_CELLS && !timeUp(o, t0)) {
        bool l, f, r;
        if (!knownWalls(&l, &f, &r)) break;
        if (firstMotion(adachi_return::solver_adachi_return(l, f, r)) != ACT_MOVE_1CELL) break;
        ++e.straight_cells;
    }
}
} // namespace

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

bool knownWalls(bool* left, bool* front, bool* right) {
    if (mousePos.x >= MAZE_SIZE || mousePos.y >= MAZE_SIZE) return false;
    AbsDir dir = static_cast<AbsDir>(mousePos.dir);
    const RelDir rel[3] = {Rl90, R0, Rr90};
    bool* out[3] = {left, front, right};
    for (int i = 0; i < 3; ++i) {
        AbsDir d = relToAbsDir(dir, rel[i]);
        bool one = get_wall_abs(&wallone, mousePos.x, mousePos.y, d);
        if (one != get_wall_abs(&wallzero, mousePos.x, mousePos.y, d)) return false;
        *out[i] = one;
    }
    return true;
}

void prepare(const PrepareOptions& options) {
    uint32_t t0 = options.clock != nullptr ? options.clock() : 0;
    adachi::search_save(&g_base);
    for (Entry& e : g_entries) e.valid = false;
    bool l, f, r;
    if (!options.all && knownWalls(&l, &f, &r)) {
        run(wallBits(l, f, r), options, t0);
    } else {
        for (uint8_t i = 0; i < HYPOTHESIS_COUNT; ++i) run(i, options, t0);
    }
    adachi::search_restore(g_base);
    g_ready = true;
}

uint8_t take(bool left, bool front, bool right) {
    const Entry& e = g_entries[wallBits(left, front, right)];
    if (!g_ready || !e.valid) {
        g_ready = false;
        uint8_t action = firstMotion(adachi_return::solver_adachi_return(left, front, right));
        g_taken_straight = (action == ACT_MOVE_1CELL) ? 1 : 0;
        return action;
    }
    adachi::search_restore(e.after);
    g_ready = false;
    g_taken_straight = e.straight_cells;
    return e.action;
}

uint8_t straightCells() {
    return g_taken_straight;
}

bool ready() {
    return g_ready;
}

WallDecision decide(bool known, uint8_t map_walls, uint8_t sensor_walls, bool rechecked) {
    if (!known) return WallDecision::useSensor;
    if (map_walls == sensor_walls) return WallDecision::useMap;
    return rechecked ? WallDecision::useSensor : WallDecision::recheck;
}

} // namespace search_lookahead
