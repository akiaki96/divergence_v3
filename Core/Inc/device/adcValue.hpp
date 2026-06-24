#pragma once

#include "device/battery.hpp"
#include "device/ir.hpp"
#include <cstdint>


class AdcValue {
public:
    void init();
    void update();
    
    IrSensor irR;
    IrSensor irL;
    IrSensor irFR;
    IrSensor irFL;
    Battery batt;

private:
    void tim6_wait_us(uint32_t us);
    void filter();
};