#pragma once

#include "common/etc.hpp"
#include "device/motor.hpp"
#include "config/mouse_config.hpp"
#include "common/pid.hpp"

enum class MotorDriverState {
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

    // 並進の目標値（軌道生成はPlanProfileの責務。毎tick PlanProfile::update()から渡される）。
    // accel_xは加速度FFの入力としてだけ使う（積分はしない）
    void setTargetX(float position_x, float velocity_x, float accel_x) {
        target_position_x_ = position_x;
        target_velocity_x_ = velocity_x;
        target_accel_x_ = accel_x;
    }

    float getTargetAccelX() const {
        return target_accel_x_;
    }

    // 追従性検証用ログ（target_velocity_x）で参照する
    float getTargetVelocityX() const {
        return target_velocity_x_;
    }

    float getTargetPositionX() const {
        return target_position_x_;
    }

    // PI+FF診断用ログ：飽和状態を確認するため
    float getVelocityXSaturated() const {
        return velocity_pid_saturated_ ? 1.f : 0.f;
    }

    // ---- 回転（角速度・角度）----
    // 回転の目標値（並進と同じくPlanProfile::update()から毎tick渡される）。
    // alphaは角加速度FFの入力としてだけ使う（積分はしない）
    void setTargetRotation(float angle, float omega, float alpha) {
        target_angle_ = angle;
        target_omega_ = omega;
        target_alpha_ = alpha;
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

    MotorDriverState state = MotorDriverState::setDuty;
private:
    Motor& motorLeft_;
    Motor& motorRight_;

    float dutyFromVoltage(float voltage) const;

    // 目標値（PlanProfile::update()から毎tick渡される）
    float target_velocity_x_ = 0.f;   // [mm/s]
    float target_position_x_ = 0.f;   // [mm]
    float target_accel_x_ = 0.f;      // [mm/s^2] 加速度FFの入力

    PIDController pid_velocity_x_;
    bool velocity_pid_saturated_ = false;

    float target_alpha_ = 0.f;   // [dps/s] 角加速度FFの入力
    float target_omega_ = 0.f;   // [dps]
    float target_angle_ = 0.f;   // [deg]

    PIDController pid_omega_;
};