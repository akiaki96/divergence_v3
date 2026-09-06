#include "device/device_instance.hpp"
#include "tim.h"

float globalTime = 0.f; // sec

Encoder encoderLeft(&htim8, Direction::Normal);
Encoder encoderRight(&htim4, Direction::Reversed);

Imu imu;

LedBar16 ledBar16;
LedManager ledManager;

IrSensor irR;
IrSensor irL;
IrSensor irFR;
IrSensor irFL;
Battery battery;

AdcValue adcValue;

Motor motorLeft(&htim2, TIM_CHANNEL_4, MOTOR_L_CW_GPIO_Port, MOTOR_L_CW_Pin, MOTOR_L_CCW_GPIO_Port, MOTOR_L_CCW_Pin, Direction::Normal);
Motor motorRight(&htim2, TIM_CHANNEL_1, MOTOR_R_CW_GPIO_Port, MOTOR_R_CW_Pin, MOTOR_R_CCW_GPIO_Port, MOTOR_R_CCW_Pin, Direction::Reversed);
MotorDriver motorDriver(motorLeft, motorRight);

Logger logger;