#pragma once

#include <cstddef>
#include "common/slalom.hpp"
#include "config/slalom_params.hpp"

// スラロームの実機試験（パラメータは生成ヘッダ config/slalom_params.hpp）。手順はrunClosedLoopTest()を参照。
// 機体の後端を区画の後壁に当てて置き，入口の基準点まで加速して等速で進み，スラロームして，
// 出口の基準点から次の区画中央で止まる。止まった位置・向きのずれを見てtools/slalom_tuning.jsonを調整する
void runSlalomTest(const slalom::Param& param, slalom::TurnDir dir);

// メニューから呼ぶ（config::slalom::ALL[I]をdirへ旋回）
template <slalom::TurnDir Dir, std::size_t I>
void slalom_test_onenter() {
    runSlalomTest(config::slalom::ALL[I], Dir);
}
