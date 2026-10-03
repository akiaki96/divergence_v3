#pragma once

#include <cstddef>
#include "app/search_preset.hpp"
#include "config/search_presets.hpp"

// 足立法の往復探索（external/micromouse_simulator/solver の adachi_return をそのまま使う）。
//
// 置き方はスラロームの試験と同じ：機体の後端をスタート区画 (0,0) の後壁に当て，北（前）へ向ける。
// ゴール（プリセットの goal_x/goal_y）に着いたらスタートへ戻る探索を続け，スタート区画の中央で止まる。
//
// 1歩の流れ（シミュレータの ACT_* と同じ単位）：区画境界の READ_LEAD_MM 手前で壁を読み，ソルバーに
// 渡して次の動作を受け取り，PlanProfile に積む。今の動作の残り（READ_LEAD_MM ぶん）を走っている間に
// 積み終わるので，止まらずに続けて走る。
//
// 直進で着く区画境界は WallEdge に教え，壁切れを記録する（config::wall_edge::SEARCH_CORRECTION なら補正もする）。
//
// 走り終わったら機体を持ち上げて（haltByAccZ）待ち，置くと3つのログを送る：
//   search/<preset>       … 壁を読むたびに1行（シミュレータの replay.py でそのまま再生できる）
//   search/<preset>_edges … 壁切れごとに1行（app/wall_edge_log.hpp，tools/wall_edge.py で読む）
//   search/<preset>_trace … 走行中の目標・実測と壁の補正（時系列）
void runSearch(const SearchPreset& preset);

// メニューから呼ぶ（config::search::PRESETS[I] で探索する）
template <std::size_t I>
void search_onenter() {
    runSearch(config::search::PRESETS[I]);
}

// 試験用のプリセット（"menu": "test"，Test → Search）をメニューから呼ぶ
template <std::size_t I>
void test_search_onenter() {
    runSearch(config::search::TEST_PRESETS[I]);
}
