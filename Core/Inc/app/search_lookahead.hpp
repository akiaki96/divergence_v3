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
// 使い方：solver_adachi_return_init() の後と，動作を積んだ後に prepare()，壁を読んだら take()
namespace search_lookahead {

// ソルバーが返した列のうち最初の動作（SET_* とその引数，READ_WALL を飛ばす）。
// シミュレータの MazeSimulation._first_motion() と同じ
uint8_t firstMotion(const uint8_vector& actions);

// 今のソルバーの状態から，次に読む壁の8通りでソルバーを回して結果を取っておく。ソルバーの状態は元に戻す
void prepare();

// 読んだ壁に合う仮定の結果をソルバーの状態に戻し，その最初の動作を返す
// （solver_adachi_return(left, front, right) を呼んで firstMotion() を取るのと同じ）。
// prepare() していなければソルバーをその場で呼ぶ
uint8_t take(bool left, bool front, bool right);

// prepare() の結果が残っていて，次の take() で使えるか
bool ready();

} // namespace search_lookahead
