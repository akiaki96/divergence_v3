#pragma once
#include <cstdint>

class Battery {
public:
    float filtered() const { return filtered_; }
    uint16_t raw_;
    float filtered_;
};