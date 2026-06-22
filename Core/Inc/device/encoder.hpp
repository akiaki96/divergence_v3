#pragma once
#include <cstdint>
#include "stm32f4xx_hal.h"

enum class Direction {
    Normal,
    Reversed
};

class Encoder {
public:
    Encoder() = delete;
    Encoder(TIM_HandleTypeDef* htim, Direction direction);

    void init();
    void update();
    void reset();

    float distance() const { return distance_; }
    float velocity() const { return velocity_; }

private:
    int32_t readRaw() const;
    float count2distance(int32_t count) const;

private:
    TIM_HandleTypeDef* const htim_;
    Direction const direction_;

    int16_t prev_count_;

    float distance_;
    float velocity_;
};