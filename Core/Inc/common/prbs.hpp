#pragma once
#include <cstdint>

class PRBS {
public:
    PRBS();

    // seed: 非ゼロ推奨（0は固定点のため自動的に1に補正）
    // duty_min/duty_max: 印加するduty [0,1]
    // ticks_per_clock: PRBSの1ビットが何tick持続するか（Tc / 制御周期）
    // total_ticks: 信号全体の長さ（走行時間 / 制御周期）
    void configure(uint16_t seed, float duty_min, float duty_max,
                   uint32_t ticks_per_clock, uint32_t total_ticks);

    void reset();

    // 制御周期ごとに1回呼ぶ。現在tickで印加すべきdutyを返す。
    float update();

    bool     isFinished() const;
    uint32_t totalTicks() const;

private:
    bool stepLfsr();

    uint16_t seed_;
    uint16_t lfsr_;
    float    duty_min_;
    float    duty_max_;
    uint32_t ticks_per_clock_;
    uint32_t total_ticks_;
    uint32_t tick_counter_;
    uint32_t clock_counter_;
    bool     current_bit_;
    bool     finished_;
};