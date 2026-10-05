#include "app/maze_store.hpp"
#include <cstdio>
#include <cstring>
#include "device/flash_bank.hpp"

namespace maze_store {

namespace {
constexpr uint32_t WORDS = sizeof(Record) / 4;
constexpr uint32_t ERASED = 0xFFFFFFFFu;

bool valid(const Record& r) {
    return r.magic == MAGIC && r.version == VERSION && r.length == sizeof(Record) &&
           r.crc == crc32(&r, offsetof(Record, crc));
}

const Record* slotAt(uint8_t bank, uint32_t slot) {
    return reinterpret_cast<const Record*>(flash_bank::address(bank) + slot * sizeof(Record));
}

// 枠が消したまま（全語 0xFF）か。書きかけで止まった枠は 0xFF でない語があるので使用中になる
bool blank(uint8_t bank, uint32_t slot) {
    const uint32_t* w = reinterpret_cast<const uint32_t*>(slotAt(bank, slot));
    for (uint32_t i = 0; i < WORDS; ++i) {
        if (w[i] != ERASED) return false;
    }
    return true;
}

// ---- journal の状態 ----
struct Journal {
    bool open = false;
    uint8_t bank = 0;
    uint32_t next_slot = 0;       // 次に書き始める枠
    uint32_t next_sequence = 1;
    union {                       // 書きかけ（または最後に書いた）記録。フラッシュへは語で書く
        Record record;
        uint32_t words[WORDS];
    } buf = {};
    bool has_last = false;
    uint32_t writing_slot = 0;
    uint32_t written = WORDS;     // words[] のうち書いた語の数（WORDS なら書きかけはない）
    Result result = Result::ok;   // 最後に書き終えた記録の結果
    uint32_t appended = 0;
    uint32_t failed = 0;
};
Journal g_journal;

// 書き終えた記録を読み直して確かめる
void finishRecord(Journal& j) {
    const Record* r = slotAt(j.bank, j.writing_slot);
    if (j.result == Result::ok && (std::memcmp(r, &j.buf.record, sizeof(Record)) != 0 || !valid(*r))) {
        j.result = Result::verifyFailed;
    }
    if (j.result != Result::ok) ++j.failed;
}
} // namespace

const char* resultName(Result r) {
    switch (r) {
    case Result::ok:            return "ok";
    case Result::eraseFailed:   return "erase failed";
    case Result::programFailed: return "program failed";
    case Result::verifyFailed:  return "verify failed";
    case Result::full:          return "bank full";
    default:                    return "journal not open";
    }
}

uint32_t crc32(const void* data, std::size_t size) {
    // CRC-32（IEEE 802.3，反転形 0xEDB88320）。表を持たない1bitずつの計算（180byteなら十分速い）
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

Record capture(uint8_t goal_x, uint8_t goal_y, bool complete) {
    Record r{};
    r.goal_x = goal_x;
    r.goal_y = goal_y;
    r.flags = complete ? FLAG_COMPLETE : 0;
    r.wallzero = ::wallzero;   // ソルバーのグローバル（global.h）
    r.wallone = ::wallone;
    for (uint8_t y = 0; y < MAZE_SIZE; ++y) {
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
            if (::visited[y][x]) r.visited[y] |= static_cast<uint16_t>(1u << x);
        }
    }
    return r;
}

void applyToSolver(const Record& r) {
    ::wallzero = r.wallzero;
    ::wallone = r.wallone;
    for (uint8_t y = 0; y < MAZE_SIZE; ++y) {
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) ::visited[y][x] = isVisited(r, x, y);
    }
}

uint32_t usedSlots(uint8_t bank) {
    // 枠は先頭から順に書くので，後ろから見て最初に消したままでない枠の次が使っている数
    uint32_t n = SLOT_COUNT;
    while (n > 0 && blank(bank, n - 1)) --n;
    return n;
}

const Record* recordIn(uint8_t bank) {
    for (uint32_t n = usedSlots(bank); n > 0; --n) {
        const Record* r = slotAt(bank, n - 1);
        if (valid(*r)) return r;
    }
    return nullptr;
}

const Record* latest(uint8_t* bank) {
    const Record* best = nullptr;
    for (uint8_t b = 0; b < flash_bank::COUNT; ++b) {
        const Record* r = recordIn(b);
        if (r != nullptr && (best == nullptr || r->sequence > best->sequence)) {
            best = r;
            if (bank != nullptr) *bank = b;
        }
    }
    return best;
}

bool sameMaze(const Record& a, const Record& b) {
    return a.goal_x == b.goal_x && a.goal_y == b.goal_y && a.flags == b.flags &&
           std::memcmp(&a.wallzero, &b.wallzero, sizeof(Wall)) == 0 &&
           std::memcmp(&a.wallone, &b.wallone, sizeof(Wall)) == 0 &&
           std::memcmp(a.visited, b.visited, sizeof(a.visited)) == 0;
}

Result save(Record r) {
    Result opened = journal::open(1);
    if (opened != Result::ok) return opened;
    if (!journal::append(r)) return Result::full;
    return journal::flush();
}

bool clear() {
    journal::close();
    bool ok = true;
    for (uint8_t b = 0; b < flash_bank::COUNT; ++b) ok = flash_bank::erase(b) && ok;
    return ok;
}

namespace journal {

Result open(uint32_t min_free, bool* erased) {
    Journal& j = g_journal;
    j = Journal{};
    if (erased != nullptr) *erased = false;
    if (min_free < 1) min_free = 1;
    if (min_free > SLOT_COUNT) min_free = SLOT_COUNT;

    uint8_t newest_bank = 0;
    const Record* newest = latest(&newest_bank);
    // 最新の記録がある面の続きに書く。空きが足りなければもう一方の面（記録がなければA面）
    uint8_t target = 0;
    if (newest != nullptr) {
        target = (SLOT_COUNT - usedSlots(newest_bank) >= min_free) ? newest_bank
                                                                   : static_cast<uint8_t>(1 - newest_bank);
    }
    uint32_t used = usedSlots(target);
    if (SLOT_COUNT - used < min_free) {
        // ここに来るのは最新の記録がない面だけ（最新の記録は消さない）
        if (!flash_bank::erase(target)) return Result::eraseFailed;
        if (erased != nullptr) *erased = true;
        used = 0;
    }
    j.open = true;
    j.bank = target;
    j.next_slot = used;
    j.next_sequence = (newest != nullptr) ? newest->sequence + 1 : 1;
    return Result::ok;
}

bool isOpen() {
    return g_journal.open;
}

void close() {
    g_journal.open = false;
}

bool append(Record r) {
    Journal& j = g_journal;
    if (!j.open || busy() || j.next_slot >= SLOT_COUNT) return false;
    r.magic = MAGIC;
    r.version = VERSION;
    r.length = sizeof(Record);
    r.sequence = j.next_sequence++;
    r.crc = crc32(&r, offsetof(Record, crc));
    j.buf.record = r;
    j.has_last = true;
    j.writing_slot = j.next_slot++;
    j.written = 0;
    j.result = Result::ok;
    ++j.appended;
    return true;
}

bool busy() {
    return g_journal.written < WORDS;
}

void step(uint32_t words) {
    Journal& j = g_journal;
    // 語の順に書くので，CRC（最後の語）が最後になる
    for (uint32_t n = 0; n < words && j.written < WORDS; ++n) {
        if (!flash_bank::program(j.bank, j.writing_slot * sizeof(Record) + j.written * 4, &j.buf.words[j.written], 1)) {
            j.result = Result::programFailed;
            j.written = WORDS;   // この枠は捨てる（CRC が合わないので読むときに無視される）。次は次の枠へ
            finishRecord(j);
            return;
        }
        if (++j.written == WORDS) finishRecord(j);
    }
}

Result flush() {
    step(WORDS);
    return g_journal.result;
}

const Record* last() {
    return g_journal.has_last ? &g_journal.buf.record : nullptr;
}

uint32_t appendedCount() {
    return g_journal.appended;
}

uint32_t failedCount() {
    return g_journal.failed;
}

uint32_t freeSlots() {
    return g_journal.open ? SLOT_COUNT - g_journal.next_slot : 0;
}

} // namespace journal

WallState wallState(const Record& r, uint8_t x, uint8_t y, AbsDir dir) {
    if (get_wall_abs(&r.wallzero, x, y, dir)) return WallState::wall;
    if (!get_wall_abs(&r.wallone, x, y, dir)) return WallState::open;
    return WallState::unknown;
}

bool isVisited(const Record& r, uint8_t x, uint8_t y) {
    return (r.visited[y] >> x) & 1u;
}

void print(const Record& r) {
    // 1区画は横3文字。上から y = 15 → 0 の順に，北の壁の行と区画の行を出す
    auto horizontal = [&](uint8_t y, bool north) {
        std::printf("+");
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
            WallState s = wallState(r, x, y, north ? Nth : Sth);
            std::printf("%s+", s == WallState::wall ? "---" : (s == WallState::open ? "   " : " ? "));
        }
        std::printf("\r\n");
    };
    for (int y = MAZE_SIZE - 1; y >= 0; --y) {
        horizontal(static_cast<uint8_t>(y), true);
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
            WallState w = wallState(r, x, static_cast<uint8_t>(y), Wst);
            char mark = ' ';
            if (x == 0 && y == 0) mark = 'S';
            else if (x == r.goal_x && y == r.goal_y) mark = 'G';
            else if (isVisited(r, x, static_cast<uint8_t>(y))) mark = '.';
            std::printf("%c %c ", w == WallState::wall ? '|' : (w == WallState::open ? ' ' : ':'), mark);
        }
        WallState e = wallState(r, MAZE_SIZE - 1, static_cast<uint8_t>(y), Est);
        std::printf("%c\r\n", e == WallState::wall ? '|' : (e == WallState::open ? ' ' : ':'));
    }
    horizontal(0, false);
}

} // namespace maze_store
