#pragma once

#include "device/motor.hpp"
#include "common/prbs.hpp"

enum MotorDriverState {
    off,
    modeSelecting,
    setDuty,
    lampDuty,
    prbsDuty,
    setVoltage,    // 追加：電圧指令（毎tickでVbatt補償）
    prbsVoltage,   // 追加：電圧PRBS（毎tickでVbatt補償）
};

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

    void setVoltage(float voltage_L, float voltage_R) {
        target_voltage_L_ = voltage_L;
        target_voltage_R_ = voltage_R;
        state = MotorDriverState::setVoltage;
    }

    void setBreak() {
        state = MotorDriverState::setDuty;
        motorLeft_.setBreak();
        motorRight_.setBreak();
    }

    void setPRBS(PRBS* prbs) {
        prbs_ = prbs;
        state = MotorDriverState::prbsDuty;
    }

    bool isPRBSFinished() const {
        return (state != MotorDriverState::prbsDuty);
    }


    float getLeftDuty(void) const;
    float getRightDuty(void) const;
    void setLampGrad(float lamp);

    MotorDriverState state = MotorDriverState::setDuty;
private:
    Motor& motorLeft_;
    Motor& motorRight_;
    float target_voltage_L_ = 0.f;
    float target_voltage_R_ = 0.f;
    PRBS* prbs_ = nullptr;

    float lamp_grad_ = 0.f;

    float dutyFromVoltage(float voltage) const;
};