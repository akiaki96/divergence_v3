#pragma once

#include <cstddef>
#include <cstdint>
#include "common/slalom.hpp"
#include "config/slalom_params.hpp"

// スラロームの実機試験（パラメータは生成ヘッダ config/slalom_params.hpp）。手順はrunClosedLoopTest()を参照。
// 機体の後端を区画の後壁に当てて置き，入口の基準点まで加速して等速で進み，スラロームして，
// 出口の基準点から次の区画中央で止まる。止まった位置・向きのずれを見てtools/slalom_tuning.jsonを調整する。
//
// straightは同じ置き方・速度・距離（助走＋入口〜出口＋停止）を旋回せずに直進する。
// 旋回を除いて，置き方（BACK_TO_AXLE_MM）とエンコーダの距離だけを確かめる（スラロームのずれの切り分け用）
enum class SlalomTestMode : uint8_t { left, right, straight };

void runSlalomTest(const slalom::Param& param, SlalomTestMode mode);

// メニューから呼ぶ（config::slalom::ALL[I]をmodeで走らせる）
template <SlalomTestMode Mode, std::size_t I>
void slalom_test_onenter() {
    runSlalomTest(config::slalom::ALL[I], Mode);
}
