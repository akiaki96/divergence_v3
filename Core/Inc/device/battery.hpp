#pragma once
#include <cstdint>

class Battery {
public:
    uint16_t filtered() const { return filtered_; }
    uint16_t raw_;
    uint16_t filtered_;
};