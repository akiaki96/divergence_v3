#pragma once

#include <cstdint>
#include "adachi_return.hpp"

// 探索のソルバー（adachi_return）を1歩先に回しておく。
//
// 壁を読んでからソルバーを呼ぶと，計算が区画境界の READ_LEAD_MM 手前から境界までに終わらないといけない。
// そこで，次の区画で読む左・前・右の壁の有無の8通りすべてについて，前の動作を積んだ直後（その区画へ
// 走っている間）にソルバーを回して結果の状態を取っておき，壁を読んだら合う仮定の状態を戻すだけにする。
// ソルバーの状態の保存・復元は adachi::search_save/search_restore（search_step が書き換えるものすべて）。
//
// 次の区画の左・前・右の壁がすでに分かっていれば（knownWalls()），探索はその値を使うので，その1通りだけ回す。
// さらに chains なら，仮定ごとの結果から「左・前・右が既知の区画が続く間」ソルバーを既知の壁で進め，
// 直進（ACT_MOVE_1CELL）が何区画続くかを数える（straightCells()）。既知の壁は読んでも変わらないので，
// その区間の動作は決まっていて，探索は1本の台形（加速・減速）で走れる。
//
// 使い方：solver_adachi_return_init() の後と，動作を積んだ後に prepare()，壁を読んだら take()
namespace search_lookahead {

// ソルバーが返した列のうち最初の動作（SET_* とその引数，READ_WALL を飛ばす）。
// シミュレータの MazeSimulation._first_motion() と同じ
uint8_t firstMotion(const uint8_vector& actions);

// 今のソルバーの位置（mousePos：次に壁を読む区画と向き）の左・前・右の壁がすべて既知なら，
// その値を入れて true（未知の壁は wallone では有り，wallzero では無しなので，両者が一致すれば既知）
bool knownWalls(bool* left, bool* front, bool* right);

// 時間の予算を測る時計 [us]（実機は DWT。nullptr なら予算なし）
using ClockUs = uint32_t (*)();

struct PrepareOptions {
    bool chains = false;       // 既知の区画が続く直進の数を数える
    bool all = false;          // 次の区画が既知でも8通りすべて回す（壁が地図と食い違って読み直すとき）
    uint32_t budget_us = 0;    // chains にかけてよい時間（clock があるときだけ。超えたら数えるのをやめる）
    ClockUs clock = nullptr;
};

// 今のソルバーの状態から，次に読む壁の仮定でソルバーを回して結果を取っておく。ソルバーの状態は元に戻す
void prepare(const PrepareOptions& options = PrepareOptions{});

// 読んだ壁に合う仮定の結果をソルバーの状態に戻し，その最初の動作を返す
// （solver_adachi_return(left, front, right) を呼んで firstMotion() を取るのと同じ）。
// prepare() していないか，その仮定を回していなければソルバーをその場で呼ぶ
uint8_t take(bool left, bool front, bool right);

// 直前の take() の動作から続く直進の区画数（動作が直進でなければ0）。
// prepare() で chains にしていなければ（またはその場でソルバーを呼んだら）直進でも1
uint8_t straightCells();

// prepare() の結果が残っていて，次の take() で使えるか
bool ready();

// 読んだ壁（bit0: 左, bit1: 前, bit2: 右）をどう使うか
enum class WallDecision : uint8_t {
    useMap,      // 3辺とも既知で，センサーと一致：地図の壁を使う
    useSensor,   // 未知の壁がある，または読み直した後：センサーの壁を使う（地図はその値で書き換わる）
    recheck,     // 3辺とも既知なのにセンサーと食い違う：止まって読み直す
};
WallDecision decide(bool known, uint8_t map_walls, uint8_t sensor_walls, bool rechecked);

// 前壁を読み落として直進を選んだ歩のやり直し。直前の take() が直進で，ソルバーは次の区画（mousePos）に
// 進んでいるとき，mousePos を1区画戻して（向きはそのまま），その区画の壁を前壁ありで渡し直し，最初の動作を返す
// （その区画で読んだ left・right と，読み落とした前壁。地図の前壁も有りに書き換わる）。
// prepare() の結果は使えなくなる（呼んだ後に prepare() し直す）
uint8_t redoWithFrontWall(bool left, bool right);

// 3辺とも既知の区画で，読み直しても地図と食い違ったセンサーの壁 sensor_walls が，同じ向きのまま1区画先・
// 1区画手前の区画の地図の壁と一致するか（その区画も3辺とも既知のときだけ）。機体が実際は1区画ずれた所にいる
// （自己位置を見失った）手がかり。両方と一致することもある（廊下はどこも 101）ので，別々に返す
constexpr uint8_t SHIFT_AHEAD = 1;    // 1区画先と一致
constexpr uint8_t SHIFT_BEHIND = 2;   // 1区画手前と一致
uint8_t shiftMatch(uint8_t sensor_walls);

// 壁の組の番号（bit0: 左, bit1: 前, bit2: 右。探索のログの walls 列と同じ）
constexpr uint8_t wallBits(bool left, bool front, bool right) {
    return static_cast<uint8_t>((left ? 1 : 0) | (front ? 2 : 0) | (right ? 4 : 0));
}

} // namespace search_lookahead
