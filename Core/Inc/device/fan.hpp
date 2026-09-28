#pragma once

#include "stm32f4xx_hal.h"

// 吸引用DCモーター。TIM3_CH1(PB4)のPWMで駆動する（64kHz, ARR=1000）
class Fan {
public:
    Fan(TIM_HandleTypeDef* htim, uint32_t channel);

    // PWMを開始する（duty=0で開始するのでファンは回らない）
    void init();

    // 0.0(停止)〜1.0(全開)。範囲外はクランプする
    void setDuty(float duty);
    void stop() { setDuty(0.f); }

    // PWM周期 [timer tick]。既定は1000（64kHz）。dutyは周期に対する比で保持されるので，変更後にsetDuty()し直す
    // ADC(1kHz)がPWMと位相ロックしてリプルの同じ位相を拾い続けるのを避けるため，
    // 試験中は1000と互いに素な値（997等）にして位相を巡回させる
    void setPwmPeriod(uint32_t period_ticks);
    uint32_t getPwmPeriod() const { return __HAL_TIM_GET_AUTORELOAD(htim_) + 1; }

    float getDuty() const { return duty_; }

private:
    TIM_HandleTypeDef* const htim_;
    uint32_t const channel_;
    float duty_ = 0.f;
};
