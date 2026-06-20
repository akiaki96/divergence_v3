#pragma once

#include "device/led.hpp"
#include "device/led_bar.hpp"

class LedManager {
public:
    LedManager();

    Led lOut;
    Led rOut;
    Led lIn;
    Led rIn;
    Led rFront;
    Led lFront;

    LedBar16 bar;
};