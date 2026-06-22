#include "device/encoder.hpp"
#include "config/mouse_config.hpp"
#include <numbers>

Encoder::Encoder(TIM_HandleTypeDef* htim, Direction direction)
    : htim_(htim), direction_(direction) {
        reset();
}

void Encoder::init() {
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
}

void Encoder::update() {
    uint16_t now_count = static_cast<uint16_t>(readRaw());

    int16_t delta_count = static_cast<int16_t>(now_count - prev_count_);
    prev_count_ = now_count;

    float delta_distance = count2distance(delta_count);
    distance_ += delta_distance;
    
    velocity_ = delta_distance / config::control::CONTROL_PEROID_S;
}


void Encoder::reset() {
    __HAL_TIM_SET_COUNTER(htim_, 0);
    prev_count_ = 0;
    distance_ = 0.f;
    velocity_ = 0.f;
}

int32_t Encoder::readRaw() const {
    // return htim_->Instance->CNT;
    return  (int32_t)__HAL_TIM_GET_COUNTER(htim_);
}

float Encoder::count2distance(int32_t delta_count) const {
    if (direction_ == Direction::Reversed) {
        delta_count = -delta_count;
    }
    return static_cast<float>(delta_count) / (config::mouse::ENCODER_RES * 4) * config::mouse::GEAR_RATIO * config::mouse::WHEEL_RADIUS_MM * 2.f * std::numbers::pi_v<float>;
}