#pragma once

#include "device/led.hpp"

class LedManager {
public:
    LedManager();

    void setall(bool blOut, bool brOut, bool blIn, bool brIn, bool brFront, bool blFront);

    Led lOut;
    Led rOut;
    Led lIn;
    Led rIn;
    Led rFront;
    Led lFront;
};