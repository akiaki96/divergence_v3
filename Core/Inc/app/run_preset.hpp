#pragma once

#include <cstdint>
#include "common/slalom.hpp"
#include "solver_options.h"   // TurnKind（TURN_L90 … TURN_S90。external/micromouse_simulator/solver/core）

// 最短走行のプリセット。tools/run_presets.json から tools/gen_run_presets.py が
// config/run_presets.hpp（config::run::PRESETS）を生成する

// 1種類のターンの候補。速い順に並べ，list[0] がプリセットのその種類の速度。
// ターンごとに速度が違ってよいので，あいだの直線が短くて速度を変えきれない（ターンどうしが直線なしで続くなら同じ速度で
// なければならない）ところや，スタート直後・ゴール直前で加速・減速しきれないところは，fast_plan が下の候補に落とす。
// count が 0 なら，この種類のターンは使わない（ソルバーに選ばせないよう，そのコストを大きくする）
struct TurnLadder {
    const slalom::Param* const* list;
    uint8_t count;

    constexpr const slalom::Param* top() const { return (count > 0) ? list[0] : nullptr; }
};

// 最短走行の経路（time_based_dijkstra）は、直線が続くところは大回り（区画中央から）、1区画ずつ曲がるジグザグは
// 小回り（区画の辺から）で曲がる。斜めありなら斜めのターンも使う
struct RunPreset {
    const char* name;                       // メニューの表示とログのファイル名（例 "500"）
    float turn_speed;                       // [mm/s] 代表のターンの速度（ソルバーの直線のコストの始点・終点の速度。表示用）
    float max_speed;                        // [mm/s] 縦横の直線の最高速度
    float max_speed_dia;                    // [mm/s] 斜めの直線の最高速度
    float accel;                            // [mm/s^2] 直線の加速度
    float decel;                            // [mm/s^2] 直線の減速度（大きさ）
    TurnLadder turns[TURN_KIND_COUNT];      // ターンの種類（TurnKind）ごとの候補。斜めなしなら斜めの5種類は空
    bool diagonal;                          // 斜めの経路を使う（solver_options.diagonal）
    bool fan;                               // ファンを config::fan::RUN_DUTY で回して走る（ターンもファンONの設計を使う）
    bool wall_edge;                         // 区画中央から入るターン（L90・T180・IN45・IN135）の前の直線で壁切れの補正をかける（false でも壁切れは記録する）
    bool diag_control;                      // 斜めの直線で切れ目からの距離の表で向きを補正する（common/diag_control.hpp。false でも横のずれは記録する）

    constexpr const slalom::Param* top(TurnKind k) const { return turns[k].top(); }
};
