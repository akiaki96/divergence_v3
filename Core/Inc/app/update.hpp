#pragma once


void update_1kHz();
#include <cstdint>

// 制御の割り込み（htim6，1kHz）の間隔の計測。フラッシュへの書き込みで CPU が止まると間隔が延びる。
// DWT のサイクルカウンタを使うので，それを動かしてから reset() する
namespace control_timing {
void reset();
uint32_t maxGapUs();   // reset() からの割り込みの間隔の最大 [us]（名目 1000us）
}
