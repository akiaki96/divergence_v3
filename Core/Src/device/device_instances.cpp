#include "device/device_instance.hpp"
#include "spi.h"
#include "tim.h"

Encoder encoderLeft(&htim8, Direction::Normal);
Encoder encoderRight(&htim4, Direction::Reversed);

Imu imu;

LedBar16 ledBar16;