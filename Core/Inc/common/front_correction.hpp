#pragma once

#include <cstddef>
#include <cstdint>
#include "config/front_distance_table.hpp"

// 前壁の距離による S90 の入口（pre-offset）の補正。考え方と定数は config::front_correction（mouse_config.hpp）。
//
// S90 は区画境界から区画境界へ曲がる。前のターンの出口のずれ（新しい向きの前後）がそのまま次の S90 の入口に
// 入るので，S90 が続くと積み重なる。壁を読む位置（境界の READ_LEAD 手前）では前のターンが終わっていて，
// 前のセンサーは新しい向きの前壁を見ている。そこで前壁までの距離から前後のずれを求め，これから積む S90 の
// pre-offset を伸び縮みさせる（目標軌道は区間の長さが変わるだけで段差は出ない）。
//
// デバイスに依存しない（ホストの単体試験 tools/host_test/test_front_correction.cpp でも使う）
namespace front_correction {

// 換算表（値の大きい順）で値 → 車軸から前の壁の面までの距離 [mm]。表の範囲の外は NaN
float tableDistance(const config::front_distance::Point* table, std::size_t count, float value);
float frontLeftMm(float value);
float frontRightMm(float value);

// 読み位置での前後のずれ e [mm]（正：機体は実際は後ろにいる）。
// value_* は前左・前右の値，past_read は実測位置が読む位置（境界 − READ_LEAD）を過ぎた量 [mm]。
// どちらかの距離が config::front_correction の範囲の外なら NaN
float estimateError(float value_left, float value_right, float past_read);

// ずれ e → 補正 δ [mm]（不感帯・ゲイン・上限）。e が NaN なら 0
float correction(float error);

// 補正 δ の効かせ方。pre_adjust は pre-offset に足す量，position_shift は実測位置に足す量
// （pre を 0 にしても足りない，機体が前に出すぎている分。正）。
// 補正後の pre が min_pre（1tick で進む距離より長く。短い区間は PlanProfile が tooShort で弾く）より
// 短くなるなら pre を 0 にし，その端数も実測位置で足す（このときだけ position_shift は負にもなる）
struct Split {
    float pre_adjust;
    float position_shift;
};
Split split(float delta, float pre_offset, float min_pre);

} // namespace front_correction
