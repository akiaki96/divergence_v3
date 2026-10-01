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

    // ---- 回転（角度）----
    // 回転の目標値（並進と同じくPlanProfile::update()から毎tick渡される）。
    // 制御に使うのは目標角度だけ（目標角速度はログ用）
    void setTargetRotation(float angle, float omega) {
        target_angle_ = angle;
        target_omega_ = omega;
    }

    float getTargetOmega() const {
        return target_omega_;
    }

    float getTargetAngle() const {
        return target_angle_;
    }

    // 角度PIの出力（角速度の指令）[dps]
    float getOmegaCommand() const {
        return omega_cmd_;
    }

    float getAngleIntegralTerm() const {
        return pid_angle_.getIntegralTerm();
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

    float target_omega_ = 0.f;   // [dps]
    float target_angle_ = 0.f;   // [deg]

    PIDController pid_angle_;   // 角度PI：出力は角速度の指令 [dps]
    PIDController pid_omega_;   // 角速度PI：出力は左右の電圧差 R − L [V]
    float omega_cmd_ = 0.f;     // [dps] 角度PIの出力（ログ用に保持）
    float rotation_saturation_ = 0.f;   // 前tickの左右の電圧差の飽和（+1:上限, −1:下限, 0:なし）。角度PIの条件付き積分に使う
};