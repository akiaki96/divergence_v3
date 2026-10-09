#pragma once

#include <cstdint>
#include "common/slalom.hpp"

// 探索のプリセット。tools/search_presets.json から tools/gen_search_presets.py が
// config/search_presets.hpp（config::search::PRESETS）を生成する

// 区画に沿って走る（斜めに入らない）ときのターン。どれも探索速度で曲がる。
// 探索中でも既知区間では大回りを使うので，小回り以外も持てるようにしている
struct OrthoTurns {
    const slalom::Param* s90;    // 小回り90°（区画境界 → 区画境界）。必須：未知区間の1歩はこれで曲がる
    const slalom::Param* l90;    // 大回り90°（区画中央 → 区画中央）。nullptr なら使わない
    const slalom::Param* t180;   // 180°（区画中央 → 区画中央）。nullptr なら使わない
};

// 斜めに入る・斜めで曲がる・斜めから出るターン。どれも nullptr なら使わない
struct DiagonalTurns {
    const slalom::Param* in45;
    const slalom::Param* out45;
    const slalom::Param* v90;
    const slalom::Param* in135;
    const slalom::Param* out135;
};

// 超信地旋回（行き止まりでの180°など）。角速度は 0 → omega → 0 の台形
struct PivotParam {
    float omega;   // [dps] 最大角速度
    float alpha;   // [dps/s] 角加速度（加速・減速とも）
};

struct SearchPreset {
    const char* name;                // メニューの表示とログのファイル名（例 "500"）
    float speed;                     // [mm/s] 探索速度（= ターンの速度）
    float straight_speed;            // [mm/s] 既知の区画が続く直進で加速する最高速度（JSONで省略すると speed＝加速しない）。
                                     // 加速・減速は accel。区間の始めと終わりは speed
    float accel;                     // [mm/s^2] 直線の加速度・減速度
    OrthoTurns turns;                // 区画に沿ったターン
    const DiagonalTurns* diagonal;   // 斜めのターン。nullptr なら斜めに入らない
    PivotParam pivot;
    bool fan;                        // ファンを config::fan::RUN_DUTY で回して走る（ターンもファンONの設計を使う）
    bool wall_control;               // [実験中] 直進中に横壁で向きを補正するか（JSONで省略すると false）
    bool front_correction;           // [実験中] S90 の入口（pre-offset）を前壁の距離で補正するか（省略で false）。
                                     // false でも推定したずれは探索のログに残す（config::front_correction）
    uint8_t goal_x;                  // ゴール区画（JSONで省略すると config::search::GOAL_X/Y）。
    uint8_t goal_y;                  // 試験用に近いゴールで往復させるときに変える
    bool one_way;                    // ゴールに着いたらそこで止まる（片道）。false ならスタートへ戻る探索を続ける（往復）
    bool reset_walls;                // 始めに壁を消す。false なら保存した最新の迷路（maze_store）を引き継ぐ
};
