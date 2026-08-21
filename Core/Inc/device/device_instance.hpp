#pragma once

#include "device/encoder.hpp"
#include "device/imu.hpp"
#include "device/led_bar.hpp"
#include "device/led_manager.hpp"
#include "device/adcValue.hpp"
#include "device/motorDriver.hpp"
#include "app/logger.hpp"

extern float globalTime;

extern Encoder encoderLeft;
extern Encoder encoderRight;

extern Imu imu;

extern LedBar16 ledBar16;
extern LedManager ledManager;

extern AdcValue adcValue;

extern Motor motorLeft;
extern Motor motorRight;
extern MotorDriver motor;

extern Logger logger;