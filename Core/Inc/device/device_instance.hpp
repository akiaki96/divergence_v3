#pragma once

#include "device/encoder.hpp"
#include "device/imu.hpp"
#include "device/led_bar.hpp"
#include "device/led_manager.hpp"
#include "device/adcValue.hpp"
#include "device/motorDriver.hpp"
#include "device/odometry.hpp"
#include "device/planProfile.hpp"
#include "device/fan.hpp"
#include "app/logger.hpp"
#include "common/wall_control.hpp"

extern float globalTime;

extern Encoder encoderLeft;
extern Encoder encoderRight;

extern Imu imu;

extern LedBar16 ledBar16;
extern LedManager ledManager;

extern AdcValue adcValue;

extern IrSensor irR;
extern IrSensor irL;
extern IrSensor irFR;
extern IrSensor irFL;
extern Battery battery;

extern Motor motorLeft;
extern Motor motorRight;
extern MotorDriver motorDriver;
extern Odometry odometry;
extern PlanProfile planProfile;

extern Fan fan;

extern Logger logger;

extern WallControl wallControl;