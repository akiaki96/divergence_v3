#include "device/device_instance.hpp"
#include "tim.h"

Encoder encoderLeft(&htim8, Direction::Normal);
Encoder encoderRight(&htim4, Direction::Reversed);

Imu imu;

LedBar16 ledBar16;

AdcValue adcValue;