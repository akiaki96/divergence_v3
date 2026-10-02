#pragma once
#include <cstdint>

class IrSensor {
public:
    // ログ用：外乱光を引いた値（発光時 − 消灯時，AdcValue::filter()）
    float value() const {
        return filtered_;
    }

    int16_t raw_on_;
    int16_t raw_off_;
    int16_t filtered_;
};