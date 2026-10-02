#pragma once

#include <cstdint>

// config::mouse::BACK_TO_AXLE_MM（機体の後端から車軸まで）の確認。手順はrunClosedLoopTest()を参照。
//
// 車軸（左右の車輪の中心）を床の線（区画の境界・中心線など，90mm間隔の線）の真上に合わせて置き，
// 低速で BACK_TO_AXLE_MM + 90·n [mm] まっすぐ走って止まる。
//   止まった車軸の位置 = 線 + BACK_TO_AXLE_MM + 90·n
//   止まった後端の位置 = 線 + 90·n + (BACK_TO_AXLE_MM − 本当の値)
// なので，値が正しければ後端がちょうど90·n先の線の上で止まり，線より先に止まった量δがそのまま
// BACK_TO_AXLE_MM − 本当の値 になる（本当の値 = BACK_TO_AXLE_MM − δ）。
// エンコーダの距離の誤差もδに乗るが，距離に比例するので，nを変えて測ると分けられる：
//   δ(n) ≈ (BACK_TO_AXLE_MM − 本当の値) + s·(BACK_TO_AXLE_MM + 90·n)   （sは距離の比の誤差）
void runAxleCheck(uint32_t n);

template <uint32_t N>
void axle_check_onenter() {
    runAxleCheck(N);
}
