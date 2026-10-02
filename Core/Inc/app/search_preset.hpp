#pragma once

#include "common/slalom.hpp"

// 探索のプリセット。tools/search_presets.json から tools/gen_search_presets.py が
// config/search_presets.hpp（config::search::PRESETS）を生成する
struct SearchPreset {
    const char* name;          // メニューの表示とログのファイル名（例 "500"）
    float speed;               // [mm/s] 探索速度（= スラロームの速度）
    float accel;               // [mm/s^2] 直線の加速度・減速度
    float read_lead;           // [mm] 区画境界のこれだけ手前で壁を読む
    float pivot_omega;         // [dps] 超信地旋回の最大角速度
    float pivot_alpha;         // [dps/s] 超信地旋回の角加速度
    bool wall_control;         // 直進中に横壁で向きを補正するか
    const slalom::Param* turn; // 左右の旋回に使うスラローム（入口・出口とも区画境界）
};
