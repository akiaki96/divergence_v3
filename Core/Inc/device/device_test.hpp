#pragma once

#include "device/device_instance.hpp"
#include "config/node_func_maker.hpp"

void log_wait_onenter();

void imu_acc_onenter();
void imu_gyro_onenter();
void encoder_right_onenter();
void encoder_left_onenter();
void battery_onenter();