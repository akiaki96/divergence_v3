#pragma once

#include <cstddef>
#include "app/run_preset.hpp"
#include "config/run_presets.hpp"

// 最短走行（time_based_dijkstra の経路を、external/micromouse_simulator/solver のまま使う）。
//
// 置き方は探索と同じ：機体の後端をスタート区画 (0,0) の後壁に当て，北（前）へ向ける。
// フラッシュに保存した最新の迷路（app/maze_store.hpp、探索で保存したもの）の既知の壁だけを使い、
// その記録のゴールまで、プリセットの速度で最短時間の経路を走り、ゴール区画で止まる。
// 走る前に経路をすべて区間にして検査し（fast_plan::validate）、積めない区間があれば走らない。
//
// 入口が区画中央のターン（L90・T180・IN45・IN135）の手前の区画境界を WallEdge に教え，壁切れで並進の位置を合わせる
// （プリセットの wall_edge。false でも壁切れは記録する）。
//
// 走り終わったら機体を持ち上げて（haltByAccZ）待ち，置くと壁切れの記録（fast/<preset>_edges）と
// 時系列のログ（fast/<preset>_trace，edge_shift 列は補正の累計）を送る
void runFastRun(const RunPreset& preset);

// メニューから呼ぶ（config::run::PRESETS[I] で走る）
template <std::size_t I>
void fast_onenter() {
    runFastRun(config::run::PRESETS[I]);
}
