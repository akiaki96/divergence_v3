#pragma once

#include <cstdint>

// 直線の台形（加速 → 等速 → 減速）を PlanProfile::straight に積む区間に分ける。HAL に依存しない。
// 最短走行（app/fast_plan）と探索の既知区間の直線（app/search）で使う
namespace trapezoid {

// PlanProfile::straight(v_end, distance) で積む1つの区間
struct Part {
    float v_end;      // [mm/s] 終速
    float distance;   // [mm]
};
inline constexpr uint8_t MAX_PARTS = 3;

// これより短い加速・減速の区間は作らない（PlanProfile は1tickに満たない区間を tooShort で弾く）
inline constexpr float MIN_PART_MM = 5.f;

// 長さ d を、速度 v_in から加速度 a で v_max まで上げ、減速度 b で v_out まで下げる台形にして out に入れ、数を返す。
// 短くて v_max まで届かなければ三角、加速・減速の区間がごく短くなるなら1区間で v_in から v_out へ
uint8_t split(float d, float v_in, float v_out, float v_max, float a, float b, Part out[MAX_PARTS]);

} // namespace trapezoid
