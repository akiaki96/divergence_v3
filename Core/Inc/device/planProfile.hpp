#pragma once

#include "device/motorDriver.hpp"

class PlanProfile {
public:
    explicit PlanProfile(MotorDriver& motorDriver);
    void init();
    void update();

    void stepVelocity(float target_velocity_x, float distance);
    void stepAccel(float target_accel_x, float distance);
    void vel2vel(float velocity1, float velocity2, float distance);

    // 目標加速度・目標速度を0にして止める（目標位置はその場で保持）。各プロファイルの後に呼ぶ
    void stop(void);

    float getCurrentVelocityX() const {
        return current_velocity_x_;
    }

    float getCurrentPositionX() const {
        return current_position_x_;
    }

    float getCurrentOmega() const {
        return current_omega_;
    }

    float getCurrentAngle() const {
        return current_angle_;
    }

    // 並進位置の原点を取り直す：エンコーダ（実測位置）とmotorDriverの目標位置を0にする
    void resetTargetPositionX(void);

    // 実測角度の原点を現在の姿勢に取り直す
    void resetCurrentAngle(void);

private:
    MotorDriver& motorDriver_;

    // 以下は制御周期の割り込み（update()）とメインコンテキスト（プロファイルの待ちループ等）の両方から
    // 読み書きするためvolatileにする（最適化でループ内の読み出しが省かれないように）

    // 実測の並進速度・位置（左右エンコーダの平均）。update()で毎tick更新し，MotorDriver::update()へ渡す
    volatile float current_velocity_x_ = 0.f;
    volatile float current_position_x_ = 0.f;

    // 実測の角速度・角度（ジャイロ）。同様にupdate()で更新してMotorDriver::update()へ渡す
    volatile float current_omega_ = 0.f;  // [dps]
    volatile float current_angle_ = 0.f;  // [deg] resetCurrentAngle()時の姿勢を0とする
    volatile float angle_origin_ = 0.f;   // [deg] resetCurrentAngle()時のimu.gyroAngleZ()
};