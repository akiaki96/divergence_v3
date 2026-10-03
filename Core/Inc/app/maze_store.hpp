#pragma once

#include <cstddef>
#include <cstdint>
#include "global.h"   // ソルバー（external/micromouse_simulator/solver/core）の Wall, MAZE_SIZE, wallzero/wallone, visited

// 探索した迷路のフラッシュへの保存（device/flash_bank の A面・B面に交互に書く）。
//
// 1面に1つの記録（Record）を置く。保存は「最新の正しい記録が入っていない方」の面を消してから書くので，
// 書いている途中で電源が落ちても，もう一方の面の最新の記録は残る。CRC は最後に書くので，
// 途中で止まった記録は CRC が合わず，読むときに無視される。
// 読むときは，正しい（magic・version・長さ・CRC が合う）記録のうち sequence が最も大きいものを使う。
namespace maze_store {

// ソルバーの壁は2枚で持つ：wallzero は未知を「壁なし」，wallone は未知を「壁あり」とした迷路。
//   wallzero=1             … 壁がある（読んだ）
//   wallone=0              … 壁がない（読んだ）
//   wallzero=0, wallone=1  … 未知
struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t length;                // sizeof(Record)（形式を変えたら version を上げる）
    uint32_t sequence;              // 保存するたびに1増える。最も大きいものが最新
    uint8_t goal_x, goal_y;
    uint8_t flags;                  // FLAG_*
    uint8_t reserved;
    Wall wallzero;
    Wall wallone;
    uint16_t visited[MAZE_SIZE];    // visited[y] の bit x：区画 (x, y) を通った
    uint32_t crc;                   // ここより前の CRC32（最後に書く）
};
static_assert(sizeof(Record) % 4 == 0, "the record is written 32 bits at a time");

inline constexpr uint32_t MAGIC = 0x315A414Du;   // "MAZ1"
inline constexpr uint16_t VERSION = 1;
inline constexpr uint8_t FLAG_COMPLETE = 1;      // スタートに戻るまで探索し終えた

enum class Result : uint8_t { ok, eraseFailed, programFailed, verifyFailed };
const char* resultName(Result r);

enum class WallState : uint8_t { open, wall, unknown };

// 今のソルバーの迷路（wallzero / wallone / visited）とゴールから記録を作る（magic・sequence・CRC は save() が入れる）
Record capture(uint8_t goal_x, uint8_t goal_y, bool complete);

// 記録をソルバーに戻す。アルゴリズムの *_init() は迷路を消すので，その後に呼ぶ
void applyToSolver(const Record& r);

// 最新の正しい記録（フラッシュ上を直接指す）。どちらの面にもなければ nullptr。bank には面の番号を返す
const Record* latest(uint8_t* bank = nullptr);

// 面 bank の記録が正しければそれを，そうでなければ nullptr
const Record* recordIn(uint8_t bank);

// 保存する。消去・書き込みで CPU が1〜2s 止まるので，モーターを止めてから呼ぶ
Result save(Record r);

// 両面を消す（保存した迷路がなくなる）
bool clear();

WallState wallState(const Record& r, uint8_t x, uint8_t y, AbsDir dir);
bool isVisited(const Record& r, uint8_t x, uint8_t y);

// 迷路を文字で表示する（北が上）。壁 "---" / "|"，未知 " ? " / ":"，区画は S スタート，G ゴール，. 通った
void print(const Record& r);

uint32_t crc32(const void* data, std::size_t size);

} // namespace maze_store
