#pragma once

#include "device/encoder.hpp"
#include "device/imu.hpp"
#include "device/led_bar.hpp"

extern Encoder encoderLeft;
extern Encoder encoderRight;

extern Imu imu;

extern LedBar16 ledBar16;