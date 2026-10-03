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
// 走り終わったら機体を持ち上げて（haltByAccZ）待ち，置くと時系列のログ（fast/<preset>_trace）を送る
void runFastRun(const RunPreset& preset);

// メニューから呼ぶ（config::run::PRESETS[I] で走る）
template <std::size_t I>
void fast_onenter() {
    runFastRun(config::run::PRESETS[I]);
}
