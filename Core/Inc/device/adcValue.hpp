#pragma once

#include "device/battery.hpp"
#include "device/ir.hpp"
#include <cstdint>


class AdcValue {
public:
    void init();
    void update();
    
private:
    void tim6_wait_us(uint32_t us);
    void filter();
};