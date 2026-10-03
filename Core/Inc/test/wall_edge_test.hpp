#pragma once

#include <cstdint>

// 壁切れによる距離の補正（common/wall_edge.hpp）の試験。手順・判定は tools/WALL_EDGE.md。
//
// 置き方は探索と同じ：機体の後端をスタート区画の後壁に当て，北へ向ける。区画の列に沿って
// N_CELLS 区画まっすぐ走り，後端が N_CELLS 区画先の区画境界（柱の中心の線）に来る位置で止まる。
// 通る区画境界（k = 0 … N_CELLS−1，経路に沿った距離 (k+1)·180 − START_MM）を WallEdge に教える。
// 横壁は途中で切れるように置く（どこで切れてもよい。ログに境界ごとの壁切れが出る）。
//
//   calib  … 補正しない。壁切れの位置（境界からのずれ）を記録する → tools/wall_edge.py で OFFSET_* / LAG_S を出す
//   verify … 補正する。較正が合っていれば補正量はどれも 0 付近
//   inject … 補正する。教える境界を INJECT_MM 先へずらす：最初の壁切れで +INJECT_MM 補正され，
//            後端が線の INJECT_MM 手前で止まる（補正が実際の止まる位置を動かすことの確認）
//
// ログ：wall_edge/<mode>_<v>（時系列）と wall_edge/<mode>_<v>_edges（壁切れごと）
enum class WallEdgeMode : uint8_t { calib, verify, inject };

void runWallEdgeTest(WallEdgeMode mode, float velocity);

template <WallEdgeMode M, int V>
void wall_edge_test_onenter() {
    runWallEdgeTest(M, static_cast<float>(V));
}
