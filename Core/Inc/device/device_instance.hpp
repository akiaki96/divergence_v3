#pragma once

#include "device/encoder.hpp"
#include "device/imu.hpp"
#include "device/led_bar.hpp"
#include "device/adcValue.hpp"
#include "device/motorDriver.hpp"
#include "app/logger.hpp"

extern Encoder encoderLeft;
extern Encoder encoderRight;

extern Imu imu;

extern LedBar16 ledBar16;

extern AdcValue adcValue;

extern Motor motorLeft;
extern Motor motorRight;
extern MotorDriver motor;

extern Logger logger;