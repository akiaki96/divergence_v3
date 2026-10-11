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
//
// 直線（最高速度・加減速度）は，メニューの Run → Fast → straight で選んでいればそれ（config::run::STRAIGHTS），
// 選んでいなければプリセットの既定の直線（RunPreset::straight）で走る。選んだ直線はログのファイル名にも付く
// （fast/<preset>_<straight>_trace）
void runFastRun(const RunPreset& preset);

// 最短走行の直線を選ぶ（index は config::run::STRAIGHTS の番号，負ならプリセットの既定の直線に戻す）。電源を切るまで覚えておく
void selectFastStraight(int index);

// メニューから呼ぶ（S = -1 は "preset"＝既定の直線）
template <int S>
void fast_straight_onenter() {
    selectFastStraight(S);
}

// メニューから呼ぶ（config::run::PRESETS[I] で走る）
template <std::size_t I>
void fast_onenter() {
    runFastRun(config::run::PRESETS[I]);
}
