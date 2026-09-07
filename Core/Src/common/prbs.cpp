#include "common/prbs.hpp"

PRBS::PRBS()
    : seed_(1), lfsr_(1), duty_min_(0.f), duty_max_(0.f),
      ticks_per_clock_(1), total_ticks_(0),
      tick_counter_(0), clock_counter_(0),
      current_bit_(false), finished_(false) {}

void PRBS::configure(uint16_t seed, float duty_min, float duty_max,
                      uint32_t ticks_per_clock, uint32_t total_ticks) {
    seed_            = (seed == 0) ? 1 : seed;
    duty_min_        = duty_min;
    duty_max_        = duty_max;
    ticks_per_clock_ = (ticks_per_clock == 0) ? 1 : ticks_per_clock;
    total_ticks_     = total_ticks;
    reset();
}

void PRBS::reset() {
    lfsr_          = seed_;
    clock_counter_ = ticks_per_clock_;  // 呼び出し1回目で即座にビット更新させる
    tick_counter_  = 0;
    current_bit_   = false;
    finished_      = false;
}

bool PRBS::stepLfsr() {
    bool     out = lfsr_ & 0x0001;
    uint16_t lsb = lfsr_ & 0x0001;
    lfsr_ >>= 1;
    if (lsb) lfsr_ ^= 0xB8;   // n=8用タップ多項式
    return out;
}

float PRBS::update() {
    if (finished_) return 0.f;

    if (clock_counter_ >= ticks_per_clock_) {
        current_bit_   = stepLfsr();
        clock_counter_ = 0;
    }
    clock_counter_++;

    tick_counter_++;
    if (tick_counter_ >= total_ticks_) {
        finished_ = true;
    }

    return current_bit_ ? duty_max_ : duty_min_;
}

bool PRBS::isFinished() const {
    return finished_;
}

uint32_t PRBS::totalTicks() const {
    return total_ticks_;
}