#pragma once

#include "device/motorDriver.hpp"

enum class PlanProfileState {
    off,
    velocityStep,
    accelStep,
};

class PlanProfile {
public:
    explicit PlanProfile(MotorDriver& motorDriver);
    void init();
    void update();

    void stepVelocity(float target_velocity_x, float distance);
    void stepAccel(float target_accel_x, float distance);
    void vel2vel(float target_velocity_x, float distance, float accel);

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

    // 実測角度の原点を現在の姿勢に取り直す
    void resetCurrentAngle(void);

private:
    PlanProfileState state = PlanProfileState::off;
    MotorDriver& motorDriver_;

    // 実測の並進速度・位置（左右エンコーダの平均）。update()で毎tick更新し，MotorDriver::update()へ渡す
    float current_velocity_x_ = 0.f;
    float current_position_x_ = 0.f;

    // 実測の角速度・角度（ジャイロ）。同様にupdate()で更新してMotorDriver::update()へ渡す
    float current_omega_ = 0.f;  // [dps]
    float current_angle_ = 0.f;  // [deg] resetCurrentAngle()時の姿勢を0とする
    float angle_origin_ = 0.f;   // [deg] resetCurrentAngle()時のimu.gyroAngleZ()
};