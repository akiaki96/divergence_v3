#pragma once

#include "device/motor.hpp"

class MotorDriver {
public:
    MotorDriver(Motor& left, Motor& right);

    void init();
    void update();

    void enable();
    void disable();

    void setDuty(float left, float right) {
        motorLeft_.setDuty(left);
        motorRight_.setDuty(right);
    }
    void setBreak() {
        motorLeft_.setBreak();
        motorRight_.setBreak();
    }

private:
    Motor& motorLeft_;
    Motor& motorRight_;

};