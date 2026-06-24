#pragma once

#include "device/battery.hpp"
#include <cstdint>

class IrSensor {
public:
    void init();

    int16_t raw_on_;
    int16_t raw_off_;
    int16_t filtered_;
};

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