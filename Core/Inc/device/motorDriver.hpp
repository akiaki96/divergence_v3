#pragma once

#include "common/etc.hpp"
#include "device/motor.hpp"
#include "common/prbs.hpp"
#include "config/mouse_config.hpp"
#include "common/pid.hpp"

enum MotorDriverState {
    off,
    modeSelecting,
    setDuty,
    lampDuty,
    prbsDuty,
    setVelocity 
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

    void switchToVelocityX();

    void setTargetVelocityX(float velocity_x) {
        velocity_x_ = velocity_x;
    }

    // 追従性検証用ログ（target_velocity_x）で参照する
    float getTargetVelocityX() const {
        return velocity_x_;
    }

    // PI+FF診断用ログ：積分ワインドアップ・feedforward寄与・飽和状態を確認するため
    float getVelocityXIntegralTerm() const {
        return pid_velocity_x_.getIntegralTerm();
    }
    float getVelocityXFeedforward() const {
        return pid_velocity_x_.getLastFeedforward();
    }
    float getVelocityXSaturated() const {
        return velocity_pid_saturated_ ? 1.f : 0.f;
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

    float velocity_x_ = 0.f;
    PIDController pid_velocity_x_;
    bool velocity_pid_saturated_ = false;
};