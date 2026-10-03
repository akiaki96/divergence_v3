#include "app/maze_store.hpp"
#include <cstdio>
#include <cstring>
#include "device/flash_bank.hpp"

namespace maze_store {

namespace {
constexpr uint32_t WORDS = sizeof(Record) / 4;

bool valid(const Record& r) {
    return r.magic == MAGIC && r.version == VERSION && r.length == sizeof(Record) &&
           r.crc == crc32(&r, offsetof(Record, crc));
}
} // namespace

const char* resultName(Result r) {
    switch (r) {
    case Result::ok:            return "ok";
    case Result::eraseFailed:   return "erase failed";
    case Result::programFailed: return "program failed";
    default:                    return "verify failed";
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

const Record* recordIn(uint8_t bank) {
    const Record* r = reinterpret_cast<const Record*>(flash_bank::address(bank));
    return valid(*r) ? r : nullptr;
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

Result save(Record r) {
    uint8_t current_bank = 0;
    const Record* current = latest(&current_bank);
    // 最新の記録が入っていない方へ書く（どちらにもなければA面）
    uint8_t target = (current != nullptr && current_bank == 0) ? 1 : 0;

    r.magic = MAGIC;
    r.version = VERSION;
    r.length = sizeof(Record);
    r.sequence = (current != nullptr) ? current->sequence + 1 : 1;
    r.crc = crc32(&r, offsetof(Record, crc));

    uint32_t words[WORDS];
    std::memcpy(words, &r, sizeof(words));
    if (!flash_bank::erase(target)) return Result::eraseFailed;
    // CRC の語を最後に書く：途中で止まれば CRC が 0xFFFFFFFF のままで，この面は無効になる
    if (!flash_bank::program(target, 0, words, WORDS - 1) ||
        !flash_bank::program(target, (WORDS - 1) * 4, &words[WORDS - 1], 1)) {
        return Result::programFailed;
    }
    if (std::memcmp(flash_bank::address(target), &r, sizeof(Record)) != 0 || recordIn(target) == nullptr) {
        return Result::verifyFailed;
    }
    return Result::ok;
}

bool clear() {
    bool ok = true;
    for (uint8_t b = 0; b < flash_bank::COUNT; ++b) ok = flash_bank::erase(b) && ok;
    return ok;
}

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
