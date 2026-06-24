#pragma once
#include <cstdint>

class IrSensor {
public:
    int16_t raw_on_;
    int16_t raw_off_;
    int16_t filtered_;
};