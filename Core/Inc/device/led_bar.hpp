#pragma once
#include <cstdint>

class LedBar16 {
public:
    void set(uint16_t pattern);
    void setBit(uint8_t index, bool on);
    void clear();

private:
    uint16_t pattern_ = 0;
    void apply();
};