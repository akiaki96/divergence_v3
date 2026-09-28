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

    // 目標加速度を設定する。end_velocity_xを与えると，目標速度がそこに達した時点で割り込み側が
    // 目標速度=end_velocity_x，目標加速度=0に固定する（区間の終端速度。行き過ぎ・符号反転を防ぐ）。
    // 省略時は終端速度なし（加速度をかけ続ける）
    void setTargetAccelX(float accel_x, float end_velocity_x) {
        target_accel_x_ = 0.f;              // 旧加速度と新しい終端速度の組で割り込みが固定しないように先に止める
        end_velocity_x_ = end_velocity_x;
        target_accel_x_ = accel_x;
    }

    void setTargetAccelX(float accel_x) {
        setTargetAccelX(accel_x, (accel_x >= 0.f) ? NO_END_VELOCITY : -NO_END_VELOCITY);
    }

    float getTargetAccelX() const {
        return target_accel_x_;
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
    // 目標角加速度を設定する。end_omegaを与えると，目標角速度がそこに達した時点で割り込み側が
    // 目標角速度=end_omega，目標角加速度=0に固定する（区間の終端角速度。並進のsetTargetAccelXと同じ）。
    // 省略時は終端角速度なし（角加速度をかけ続ける）
    void setTargetAlpha(float alpha, float end_omega) {
        target_alpha_ = 0.f;              // 旧角加速度と新しい終端角速度の組で割り込みが固定しないように先に止める
        end_omega_ = end_omega;
        target_alpha_ = alpha;
    }

    void setTargetAlpha(float alpha) {
        setTargetAlpha(alpha, (alpha >= 0.f) ? NO_END_VELOCITY : -NO_END_VELOCITY);
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

    // 目標値は割り込み（update()）で積分し，メインコンテキスト（PlanProfileの待ちループ）でも読み書きするのでvolatile
    static constexpr float NO_END_VELOCITY = 1.0e9f;
    volatile float target_accel_x_ = 0.f;
    volatile float target_velocity_x_ = 0.f;
    volatile float target_position_x_ = 0.f;
    volatile float end_velocity_x_ = NO_END_VELOCITY;   // [mm/s] 目標速度の終端（setTargetAccelX参照）

    PIDController pid_velocity_x_;
    PIDController pid_position_x_;
    bool velocity_pid_saturated_ = false;
    bool position_pid_saturated_ = false;

    // 並進と同じく割り込みとメインコンテキストで共有するのでvolatile
    volatile float target_alpha_ = 0.f;   // [dps/s]
    volatile float target_omega_ = 0.f;   // [dps]
    volatile float target_angle_ = 0.f;   // [deg]
    volatile float end_omega_ = NO_END_VELOCITY;   // [dps] 目標角速度の終端（setTargetAlpha参照）

    PIDController pid_omega_;
    PIDController pid_angle_;
    bool omega_pid_saturated_ = false;
};