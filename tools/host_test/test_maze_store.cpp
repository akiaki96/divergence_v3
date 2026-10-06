// app/maze_store の追記（journal）の試験。device/flash_bank を RAM の偽物に置き換える：
// 消去で 0xFF にし，書き込みはビットを 1→0 にしか変えられない（本物と同じ）。
// 書いた語の数を数え，決めた語数で書き込みを失敗させて「途中で電源が落ちた」を作る
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "app/maze_store.hpp"
#include "device/flash_bank.hpp"

namespace {
alignas(4) uint8_t g_flash[flash_bank::COUNT][flash_bank::SIZE];
uint32_t g_erase_count = 0;
uint32_t g_program_words = 0;
uint32_t g_fail_after = UINT32_MAX;   // この語数を書いたら以後の書き込みを失敗させる
int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            ++g_failures;                                                 \
        }                                                                 \
    } while (0)
} // namespace

namespace flash_bank {
const uint8_t* address(uint8_t bank) {
    return g_flash[bank % COUNT];
}
bool erase(uint8_t bank) {
    if (bank >= COUNT) return false;
    std::memset(g_flash[bank], 0xFF, SIZE);
    ++g_erase_count;
    return true;
}
bool program(uint8_t bank, uint32_t offset, const uint32_t* words, uint32_t count) {
    if (bank >= COUNT || offset % 4 != 0 || offset + count * 4 > SIZE) return false;
    for (uint32_t i = 0; i < count; ++i) {
        if (g_program_words >= g_fail_after) return false;
        uint32_t w;
        std::memcpy(&w, &g_flash[bank][offset + i * 4], 4);
        w &= words[i];
        std::memcpy(&g_flash[bank][offset + i * 4], &w, 4);
        ++g_program_words;
    }
    return true;
}
} // namespace flash_bank

namespace {
using namespace maze_store;

void resetFlash() {
    std::memset(g_flash, 0xFF, sizeof(g_flash));
    g_erase_count = 0;
    g_program_words = 0;
    g_fail_after = UINT32_MAX;
    journal::close();
}

// 区画 (x, 0) の北に壁を置いた迷路（x で区別する）
Record maze(uint8_t x, bool complete = false) {
    Record r{};
    r.goal_x = 7;
    r.goal_y = 7;
    r.flags = complete ? FLAG_COMPLETE : 0;
    r.wallzero.wall_hor[0] = static_cast<uint16_t>(1u << x);
    r.wallone.wall_hor[0] = 0xFFFF;
    r.visited[0] = static_cast<uint16_t>(1u << x);
    return r;
}

uint8_t markOf(const Record* r) {
    for (uint8_t x = 0; x < 16; ++x) {
        if (r->wallzero.wall_hor[0] == (1u << x)) return x;
    }
    return 0xFF;
}

void testEmpty() {
    resetFlash();
    CHECK(latest() == nullptr);
    CHECK(usedSlots(0) == 0 && usedSlots(1) == 0);
    CHECK(!journal::append(maze(1)));   // open() 前は書けない
}

void testAppendWhileRunning() {
    resetFlash();
    bool erased = true;
    CHECK(journal::open(100, &erased) == Result::ok);
    CHECK(!erased);   // 消したままの面は消さない
    CHECK(journal::freeSlots() == SLOT_COUNT);
    for (uint8_t i = 1; i <= 5; ++i) {
        CHECK(journal::append(maze(i)));
        CHECK(journal::busy());
        CHECK(!journal::append(maze(15)));   // 書きかけの間は次を受けない
        // 走行中のように1語ずつ書く。CRC を書くまで読む側には見えない（前の記録が最新のまま）
        uint32_t words = 0;
        while (journal::busy()) {
            const Record* before = latest();
            CHECK(before == nullptr || markOf(before) == i - 1);
            journal::step(1);
            ++words;
        }
        CHECK(words == sizeof(Record) / 4);
        CHECK(journal::flush() == Result::ok);
        const Record* r = latest();
        CHECK(r != nullptr && markOf(r) == i && r->sequence == i);
    }
    CHECK(usedSlots(0) == 5 && usedSlots(1) == 0);
    CHECK(journal::appendedCount() == 5 && journal::failedCount() == 0);
    CHECK(g_erase_count == 0);

    // 次の探索：同じ面の続きに書く（消さない）。sequence は続く
    CHECK(journal::open(100, &erased) == Result::ok && !erased);
    CHECK(journal::append(maze(6, true)) && journal::flush() == Result::ok);
    const Record* r = latest();
    CHECK(r != nullptr && markOf(r) == 6 && r->sequence == 6 && (r->flags & FLAG_COMPLETE));
    CHECK(usedSlots(0) == 6);
}

void testPowerLossMidRecord() {
    resetFlash();
    CHECK(journal::open(10) == Result::ok);
    CHECK(journal::append(maze(1)) && journal::flush() == Result::ok);
    // 2件目の途中（CRC の前）で止まる
    g_fail_after = g_program_words + 20;
    CHECK(journal::append(maze(2)));
    CHECK(journal::flush() == Result::programFailed);
    CHECK(journal::failedCount() == 1);
    g_fail_after = UINT32_MAX;
    const Record* r = latest();
    CHECK(r != nullptr && markOf(r) == 1);   // 書きかけは無視され，前の記録が最新
    CHECK(usedSlots(0) == 2);                // 書きかけの枠は使用中（上書きしない）

    // 電源を入れ直した後：書きかけの枠を飛ばして次の枠に書く
    journal::close();
    CHECK(journal::open(10) == Result::ok);
    CHECK(journal::append(maze(3)) && journal::flush() == Result::ok);
    r = latest();
    CHECK(r != nullptr && markOf(r) == 3 && r->sequence == 2);
    CHECK(usedSlots(0) == 3);
}

void testBankSwitch() {
    resetFlash();
    // A面を残り min_free 枠未満まで埋める
    constexpr uint32_t MIN_FREE = 50;
    CHECK(journal::open(1) == Result::ok);
    for (uint32_t i = 0; i < SLOT_COUNT - MIN_FREE + 1; ++i) {
        CHECK(journal::append(maze(static_cast<uint8_t>(i % 8))) && journal::flush() == Result::ok);
    }
    uint32_t seq_a = latest()->sequence;
    CHECK(usedSlots(0) == SLOT_COUNT - MIN_FREE + 1);

    // B面が消したままなら，消さずに B面へ
    bool erased = true;
    CHECK(journal::open(MIN_FREE, &erased) == Result::ok && !erased);
    CHECK(journal::append(maze(9)) && journal::flush() == Result::ok);
    uint8_t bank = 0;
    const Record* r = latest(&bank);
    CHECK(r != nullptr && bank == 1 && markOf(r) == 9 && r->sequence == seq_a + 1);

    // A面に空きがあっても，最新は B面なので B面の続きへ
    CHECK(journal::open(MIN_FREE, &erased) == Result::ok && !erased);
    CHECK(journal::append(maze(10)) && journal::flush() == Result::ok);
    CHECK(usedSlots(1) == 2);

    // B面を埋めると，最新のない A面を消してそちらへ（最新の B面は消さない）
    while (journal::freeSlots() > 0) {
        CHECK(journal::append(maze(11)) && journal::flush() == Result::ok);
    }
    CHECK(!journal::append(maze(12)));   // 面が一杯
    uint32_t seq_b = latest(&bank)->sequence;
    CHECK(bank == 1);
    uint32_t erases = g_erase_count;
    CHECK(journal::open(MIN_FREE, &erased) == Result::ok && erased);
    CHECK(g_erase_count == erases + 1);
    CHECK(usedSlots(0) == 0);
    CHECK(latest(&bank)->sequence == seq_b && bank == 1);   // 消した直後も最新は残る
    CHECK(journal::append(maze(13)) && journal::flush() == Result::ok);
    r = latest(&bank);
    CHECK(r != nullptr && bank == 0 && markOf(r) == 13 && r->sequence == seq_b + 1);
}

void testSaveAndSameMaze() {
    resetFlash();
    CHECK(save(maze(4, true)) == Result::ok);
    CHECK(save(maze(5, true)) == Result::ok);
    const Record* r = latest();
    CHECK(r != nullptr && markOf(r) == 5 && r->sequence == 2);
    CHECK(g_erase_count == 0);
    CHECK(sameMaze(*r, maze(5, true)));
    CHECK(!sameMaze(*r, maze(5, false)));
    CHECK(!sameMaze(*r, maze(6, true)));
    CHECK(recordIn(1) == nullptr);
}

// 以前の形式（面の先頭に1件だけ）の記録はそのまま読め，その続きに書ける
void testOldLayout() {
    resetFlash();
    Record old = maze(3, true);
    old.magic = MAGIC;
    old.version = VERSION;
    old.length = sizeof(Record);
    old.sequence = 7;
    old.crc = crc32(&old, offsetof(Record, crc));
    std::memcpy(g_flash[1], &old, sizeof(old));
    uint8_t bank = 0;
    const Record* r = latest(&bank);
    CHECK(r != nullptr && bank == 1 && r->sequence == 7);
    CHECK(journal::open(100) == Result::ok);
    CHECK(journal::append(maze(4)) && journal::flush() == Result::ok);
    r = latest(&bank);
    CHECK(r != nullptr && bank == 1 && r->sequence == 8 && usedSlots(1) == 2);
}
} // namespace

int main() {
    testEmpty();
    testAppendWhileRunning();
    testPowerLossMidRecord();
    testBankSwitch();
    testSaveAndSameMaze();
    testOldLayout();
    if (g_failures == 0) std::printf("maze_store: all tests passed (%u slots per bank)\n", SLOT_COUNT);
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
