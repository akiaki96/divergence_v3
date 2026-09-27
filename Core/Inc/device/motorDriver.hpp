#pragma once

#include "common/etc.hpp"
#include "device/motor.hpp"
#include "config/mouse_config.hpp"
#include "common/pid.hpp"

enum class MotorDriverState {
    off,
    modeSelecting,
    setDuty,
    setVelocity 
};

class MotorDriver {
public:
    MotorDriver(Motor& left, Motor& right);

    void init();
    // 実測の並進速度・位置と角速度・角度（状態推定はPlanProfileの責務）を受け取り，制御出力を更新する
    void update(float current_velocity_x, float current_position_x, float current_omega, float current_angle);

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

    void switchToVelocityX();

    void setTargetAccelX(float accel_x) {
        target_accel_x_ = accel_x;
    }

    void setTargetVelocityX(float velocity_x) {
        target_velocity_x_ = velocity_x;
    }

    // 目標位置を直接設定する（位置の原点の取り直しはPlanProfile::resetTargetPositionX()）
    void setTargetPositionX(float position_x) {
        target_position_x_ = position_x;
    }

    // 追従性検証用ログ（target_velocity_x）で参照する
    float getTargetVelocityX() const {
        return target_velocity_x_;
    }

    float getTargetPositionX() const {
        return target_position_x_;
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

    // ---- 回転（角速度・角度）----
    void setTargetAlpha(float alpha) {
        target_alpha_ = alpha;
    }

    void setTargetOmega(float omega) {
        target_omega_ = omega;
    }

    // 目標角度を0にする（実測角度の原点はPlanProfile::resetCurrentAngle()で取り直す）
    void resetTargetAngle(void) {
        target_angle_ = 0.f;
    }

    float getTargetAlpha() const {
        return target_alpha_;
    }

    float getTargetOmega() const {
        return target_omega_;
    }

    float getTargetAngle() const {
        return target_angle_;
    }

    float getOmegaIntegralTerm() const {
        return pid_omega_.getIntegralTerm();
    }
    float getOmegaFeedforward() const {
        return pid_omega_.getLastFeedforward();
    }
    float getOmegaSaturated() const {
        return omega_pid_saturated_ ? 1.f : 0.f;
    }


    float getLeftDuty(void) const;
    float getRightDuty(void) const;

    MotorDriverState state = MotorDriverState::setDuty;
private:
    Motor& motorLeft_;
    Motor& motorRight_;
    float target_voltage_L_ = 0.f;
    float target_voltage_R_ = 0.f;

    float dutyFromVoltage(float voltage) const;

    float target_accel_x_ = 0.f;
    float target_velocity_x_ = 0.f;
    float target_position_x_ = 0.f;

    PIDController pid_velocity_x_;
    PIDController pid_position_x_;
    bool velocity_pid_saturated_ = false;
    bool position_pid_saturated_ = false;

    float target_alpha_ = 0.f;   // [dps/s]
    float target_omega_ = 0.f;   // [dps]
    float target_angle_ = 0.f;   // [deg]

    PIDController pid_omega_;
    PIDController pid_angle_;
    bool omega_pid_saturated_ = false;
};