#include "device/device_instance.hpp"
#include "tim.h"

Encoder encoderLeft(&htim8, Direction::Normal);
Encoder encoderRight(&htim4, Direction::Reversed);

Imu imu;

LedBar16 ledBar16;

AdcValue adcValue;

Motor motorLeft(&htim2, TIM_CHANNEL_4, MOTOR_L_CW_GPIO_Port, MOTOR_L_CW_Pin, MOTOR_L_CCW_GPIO_Port, MOTOR_L_CCW_Pin, Direction::Normal);
Motor motorRight(&htim2, TIM_CHANNEL_1, MOTOR_R_CW_GPIO_Port, MOTOR_R_CW_Pin, MOTOR_R_CCW_GPIO_Port, MOTOR_R_CCW_Pin, Direction::Reversed);
MotorDriver motor(motorLeft, motorRight);

Logger logger;