#pragma once

#include <cstddef>
#include <cstdint>
#include "device/flash_bank.hpp"
#include "global.h"   // ソルバー（external/micromouse_simulator/solver/core）の Wall, MAZE_SIZE, wallzero/wallone, visited

// 探索した迷路のフラッシュへの保存（device/flash_bank の A面・B面）。
//
// 1面を Record の大きさの枠（slot）に区切り，先頭から順に記録を追記する（ジャーナル）。
// 消去（1〜2s CPU が止まる）は走る前の止まっているときだけにし，走行中は消した後の枠へ1語ずつ書く
// （journal）。CRC は記録の最後の語で最後に書くので，途中で止まった記録は CRC が合わず，読むときに無視される。
// 消すのは「最新の記録が入っていない方」の面だけなので，消している途中で電源が落ちても最新の記録は残る。
// 読むときは，正しい（magic・version・長さ・CRC が合う）記録のうち sequence が最も大きいものを使う。
// 1つの面の中では後ろの枠ほど新しいので，各面の最後の正しい記録を比べればよい
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

enum class Result : uint8_t { ok, eraseFailed, programFailed, verifyFailed, full, notOpen };
const char* resultName(Result r);

enum class WallState : uint8_t { open, wall, unknown };

// 今のソルバーの迷路（wallzero / wallone / visited）とゴールから記録を作る（magic・sequence・CRC は save() が入れる）
Record capture(uint8_t goal_x, uint8_t goal_y, bool complete);

// 記録をソルバーに戻す。アルゴリズムの *_init() は迷路を消すので，その後に呼ぶ
void applyToSolver(const Record& r);

inline constexpr uint32_t SLOT_COUNT = flash_bank::SIZE / sizeof(Record);   // 1面の枠の数（128KB / 180byte = 728）

// 最新の正しい記録（フラッシュ上を直接指す）。どちらの面にもなければ nullptr。bank には面の番号を返す
const Record* latest(uint8_t* bank = nullptr);

// 面 bank の最新の正しい記録（最後の正しい枠）。なければ nullptr
const Record* recordIn(uint8_t bank);

// 面 bank で使っている枠の数（最後に書いた枠の次の番号。そこから後ろは消したまま）
uint32_t usedSlots(uint8_t bank);

// 迷路の中身（壁2枚・通った区画・ゴール・flags）が同じか（magic・sequence・CRC は比べない）
bool sameMaze(const Record& a, const Record& b);

// 1件を保存し終えるまで書く（journal::open(1) → append → flush）。空きがなければ面を消すので，
// モーターを止めてから呼ぶ
Result save(Record r);

// 両面を消す（保存した迷路がなくなる）
bool clear();

// ---- 走行中の追記 ----
// open()（止まっているとき）→ append()（記録を RAM に写すだけ）→ step()（走行中に少しずつ書く）→ flush()
namespace journal {

// 追記する面を決める。最新の記録がある面に min_free 枠以上の空きがあれば，その続きに書く。
// なければもう一方の面を使い，そこにも min_free 枠の空きがなければ消す（1〜2s CPU が止まるので止まっているときに呼ぶ）。
// erased には消したかを返す
Result open(uint32_t min_free, bool* erased = nullptr);
bool isOpen();
void close();

// 記録を書く順番に並べる（sequence・CRC を入れて RAM に写す。フラッシュにはまだ書かない）。
// 前の記録を書き終えていない・面が一杯・open() していなければ false
bool append(Record r);

// 書きかけの記録があるか
bool busy();

// 書きかけの記録を最大 words 語書く。1語で CPU が典型 16us（最大 100us）止まる。書き終えた記録は読み直して確かめる
void step(uint32_t words);

// 書きかけの記録を最後まで書き，最後に書き終えた記録の結果を返す（何も書いていなければ ok）
Result flush();

// 最後に append() した記録（なければ nullptr）。間引き（sameMaze）に使う
const Record* last();

uint32_t appendedCount();   // open() からの append() の数
uint32_t failedCount();     // open() から書けなかった・読み直して違った記録の数
uint32_t freeSlots();       // 書いている面の残りの枠

} // namespace journal

WallState wallState(const Record& r, uint8_t x, uint8_t y, AbsDir dir);
bool isVisited(const Record& r, uint8_t x, uint8_t y);

// 迷路を文字で表示する（北が上）。壁 "---" / "|"，未知 " ? " / ":"，区画は S スタート，G ゴール，. 通った
void print(const Record& r);

uint32_t crc32(const void* data, std::size_t size);

} // namespace maze_store
