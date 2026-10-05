#include "app/search_lookahead.hpp"
#include <cstddef>
#include "adachi.hpp"

namespace search_lookahead {
namespace {
constexpr uint8_t HYPOTHESIS_COUNT = 8;

// 壁の組の番号（bit0: 左, bit1: 前, bit2: 右。探索のログの walls 列と同じ）
constexpr uint8_t wallIndex(bool left, bool front, bool right) {
    return static_cast<uint8_t>((left ? 1 : 0) | (front ? 2 : 0) | (right ? 4 : 0));
}

struct Entry {
    adachi::SearchState after;   // その壁の組で1歩進めた後のソルバーの状態
    uint8_t action;              // その最初の動作
};

// 合わせて約1.2KB。CCMRAM は NOLOAD（スタートアップで0にしない）だが，prepare() で書いてから読むので使える。
// 有効かどうかは通常のRAMの g_ready で表す（ホストの試験ではふつうの変数）
#ifdef __arm__
#define LOOKAHEAD_CCMRAM __attribute__((section(".ccmram")))
#else
#define LOOKAHEAD_CCMRAM
#endif
LOOKAHEAD_CCMRAM Entry g_entries[HYPOTHESIS_COUNT];
LOOKAHEAD_CCMRAM adachi::SearchState g_base;   // prepare() の前の状態（仮定ごとにここへ戻してから回す）
bool g_ready = false;
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

void prepare() {
    adachi::search_save(&g_base);
    for (uint8_t i = 0; i < HYPOTHESIS_COUNT; ++i) {
        adachi::search_restore(g_base);
        Entry& e = g_entries[i];
        e.action = firstMotion(adachi_return::solver_adachi_return((i & 1) != 0, (i & 2) != 0, (i & 4) != 0));
        adachi::search_save(&e.after);
    }
    adachi::search_restore(g_base);
    g_ready = true;
}

uint8_t take(bool left, bool front, bool right) {
    if (!g_ready) return firstMotion(adachi_return::solver_adachi_return(left, front, right));
    const Entry& e = g_entries[wallIndex(left, front, right)];
    adachi::search_restore(e.after);
    g_ready = false;
    return e.action;
}

bool ready() {
    return g_ready;
}

} // namespace search_lookahead
