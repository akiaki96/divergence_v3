#pragma once
#include <cstdint>

class Battery {
public:
    float voltage() const { return filtered_; }
    uint16_t raw_;
    float filtered_;
};