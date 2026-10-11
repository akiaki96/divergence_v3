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

// 直線のプロファイル（最高速度・加減速度）。ターンの速度（RunPreset）とは別に選べる（Run → Fast → straight）。
// プリセットごとに既定の直線を1つ持ち（RunPreset::straight），メニューで選べばどのプリセットにも組み合わせられる。
// 最高速度はターンの速度を下回れないので，組み合わせたときにターンの速度まで引き上げる（withStraight）
struct RunStraight {
    const char* name;      // メニューの表示とログのファイル名（例 "2000_a8k"）
    float max_speed;       // [mm/s] 縦横の直線の最高速度
    float max_speed_dia;   // [mm/s] 斜めの直線の最高速度
    float accel;           // [mm/s^2] 直線の加速度
    float decel;           // [mm/s^2] 直線の減速度（大きさ）
    bool fan_only;         // ファンを回すプリセットとだけ組み合わせる（吸引で滑りを抑える前提の加速度）
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
    bool fan;                               // ファンを回して走る（ターンはファンON＝config::fan::RUN_DUTY で設計したものを使う）
    float fan_duty;                         // 実際に回すファンの duty（fan が false なら 0。ふつうは RUN_DUTY，0.4 などで強く吸うこともある）
    bool wall_edge;                         // 区画中央から入るターン（L90・T180・IN45・IN135）の前の直線で壁切れの補正をかける（false でも壁切れは記録する）
    bool diag_control;                      // 斜めの直線で切れ目からの距離の表で向きを補正する（common/diag_control.hpp。false でも横のずれは記録する）
    const RunStraight* straight = nullptr;  // 直線のプロファイル（max_speed〜decel はこれを withStraight で写したもの）。nullptr なら直に書いた値

    constexpr const slalom::Param* top(TurnKind k) const { return turns[k].top(); }
};

constexpr bool isDiagonalKind(uint8_t k) {
    return k == TURN_IN45 || k == TURN_OUT45 || k == TURN_V90 || k == TURN_IN135 || k == TURN_OUT135;
}

// いちばん速いターンの速度（diagonal_only なら斜めの直線の隣に来る斜めのターンだけ）。turn_speed を下限にする
constexpr float fastestTurn(const RunPreset& p, bool diagonal_only) {
    float v = p.turn_speed;
    for (uint8_t k = 0; k < TURN_KIND_COUNT; ++k) {
        const slalom::Param* t = p.turns[k].top();
        if (t != nullptr && (!diagonal_only || isDiagonalKind(k)) && t->speed > v) v = t->speed;
    }
    return v;
}

// プリセットの直線を s にしたもの。直線はターンの速度で入って出るので，最高速度はターンの速度まで引き上げる
// （縦横は全部のターン，斜めは斜めのターン。入45° などは縦横の直線からも入るため）
constexpr RunPreset withStraight(RunPreset p, const RunStraight& s) {
    const float ortho = fastestTurn(p, false);
    const float dia = fastestTurn(p, true);
    p.max_speed = (s.max_speed > ortho) ? s.max_speed : ortho;
    p.max_speed_dia = (s.max_speed_dia > dia) ? s.max_speed_dia : dia;
    p.accel = s.accel;
    p.decel = s.decel;
    p.straight = &s;
    return p;
}
