#pragma once

#include "device/device_instance.hpp"

#define onselect(nodeName, func)  \
void nodeName##_onselect() { \
    func \
}

#define onenter(nodeName, func) \
void nodeName##_onenter() { \
    func \
}

void log_wait_onenter();

void imu_acc_onenter();
void imu_gyro_onenter();
void encoder_right_onenter();
void encoder_left_onenter();