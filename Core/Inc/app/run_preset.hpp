#pragma once

#include "app/search_preset.hpp"   // DiagonalTurns
#include "common/slalom.hpp"

// 最短走行のプリセット。tools/run_presets.json から tools/gen_run_presets.py が
// config/run_presets.hpp（config::run::PRESETS）を生成する

// 区画に沿ったターン。最短走行の経路（time_based_dijkstra）は、直線が続くところは大回り（区画中央から）、
// 1区画ずつ曲がるジグザグは小回り（区画の辺から）で曲がる
struct RunTurns {
    const slalom::Param* s90;    // 小回り90°（区画境界 → 区画境界）。ACT_TURN_*_MOVE
    const slalom::Param* l90;    // 大回り90°（区画中央 → 区画中央）。ACT_S90_*
    const slalom::Param* t180;   // 180°（区画中央 → 区画中央）。ACT_S180_*
};

struct RunPreset {
    const char* name;                // メニューの表示とログのファイル名（例 "500"）
    float turn_speed;                // [mm/s] ターンの速度（直線の始点・終点の速度）
    float max_speed;                 // [mm/s] 縦横の直線の最高速度
    float max_speed_dia;             // [mm/s] 斜めの直線の最高速度
    float accel;                     // [mm/s^2] 直線の加速度
    float decel;                     // [mm/s^2] 直線の減速度（大きさ）
    RunTurns turns;
    const DiagonalTurns* diagonal;   // 斜めのターン。nullptr なら斜めの経路を使わない（solver_options.diagonal = false）
    bool fan;                        // ファンを config::fan::RUN_DUTY で回して走る（ターンもファンONの設計を使う）
    bool wall_edge;                  // 大回り（L90・T180）の前の直線で壁切れの補正をかける（false でも壁切れは記録する）
};
