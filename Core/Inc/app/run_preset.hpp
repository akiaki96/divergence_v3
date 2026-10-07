#pragma once

#include "app/search_preset.hpp"   // DiagonalTurns
#include "common/slalom.hpp"

// 最短走行のプリセット。tools/run_presets.json から tools/gen_run_presets.py が
// config/run_presets.hpp（config::run::PRESETS）を生成する

// 区画に沿ったターン。最短走行の経路（time_based_dijkstra）は、直線が続くところは大回り（区画中央から）、
// 1区画ずつ曲がるジグザグは小回り（区画の辺から）で曲がる
struct RunTurns {
    const slalom::Param* s90;    // 小回り90°（区画境界 → 区画境界）。ACT_TURN_*_MOVE。斜めありなら nullptr でよい
                                 // （ジグザグを斜めで走る。ソルバーに選ばせないよう S90 のコストを大きくする）
    const slalom::Param* l90;    // 大回り90°（区画中央 → 区画中央）。ACT_S90_*
    const slalom::Param* t180;   // 180°（区画中央 → 区画中央）。ACT_S180_*
};

struct RunPreset {
    const char* name;                // メニューの表示とログのファイル名（例 "500"）
    float turn_speed;                // [mm/s] 小回り90°以外のターンの速度
    float s90_speed;                 // [mm/s] 小回り90°の速度（斜めなしのとき turn_speed と違ってよい。直線の始点・終点は
                                     // 隣のターンの速度になる。S90 と他のターンの間には必ず半区画以上の直線がある）
    float max_speed;                 // [mm/s] 縦横の直線の最高速度
    float max_speed_dia;             // [mm/s] 斜めの直線の最高速度
    float accel;                     // [mm/s^2] 直線の加速度
    float decel;                     // [mm/s^2] 直線の減速度（大きさ）
    RunTurns turns;
    const DiagonalTurns* diagonal;   // 斜めのターン。nullptr なら斜めの経路を使わない（solver_options.diagonal = false）
    bool fan;                        // ファンを config::fan::RUN_DUTY で回して走る（ターンもファンONの設計を使う）
    bool wall_edge;                  // 区画中央から入るターン（L90・T180・IN45・IN135）の前の直線で壁切れの補正をかける（false でも壁切れは記録する）
    // スタート直後の置き換え。置いた位置から最初の区画中央までは短い（START_TO_CENTER）ので，すぐに速いターンが来ると
    // 加速しきれない。そのときだけ，最初のターンと，直線を挟まずに続くターンを start_speed の同じ種類のターンに置き換える。
    // start_speed が 0 なら置き換えない（start_turns・start_diagonal は使わない）
    float start_speed;               // [mm/s]
    RunTurns start_turns;            // s90 は使わない（小回り90°は区画中央のターンの直後に来ない）
    const DiagonalTurns* start_diagonal;
};
