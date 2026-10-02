#pragma once

#include <cstdint>

// 前の壁までの距離とIRセンサーの値の関係を記録する（距離推定の校正用）。手順はrunClosedLoopTest()を参照。
//
// メニューで決定してから3秒（LEDバーが消えていく間）に，機体の後端を区画の後壁に当てて置く（スラロームの試験と同じ）。
// 前の壁は cells 区画先の境界にある（1区画先だと前のセンサーがふさがれてメニューを操作できないので，壁から離して決定する）。
// 前の壁へ低速で近づき（前端と壁のすき間 MIN_GAP_MM まで），少し止まってから同じ速さで戻る。
// 車軸から前の壁の面までの距離 = 180·cells − 12 − BACK_TO_AXLE_MM − 進んだ距離（エンコーダ）を正解として，
// 4つのIRセンサーの値と一緒に記録する（tools/log/ir_sweep/front_<cells>cell.csv）。
// 換算の当てはめは tools/fit_ir.py で行う
void runIrFrontSweep(uint32_t cells);

template <uint32_t Cells>
void ir_front_sweep_onenter() {
    runIrFrontSweep(Cells);
}
