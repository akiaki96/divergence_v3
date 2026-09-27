#include "device/planProfile.hpp"
#include "device/device_instance.hpp"

PlanProfile::PlanProfile(MotorDriver& motorDriver)
    : motorDriver_(motorDriver)
{}

// 軌道追従を始める前にmotorDriverを初期状態にそろえる：
// モータを止め，並進・回転の目標値（加速度・速度・位置／角加速度・角速度・角度）を0にし，
// 位置と角度の原点を現在値に取り直す。PIDゲインはMotorDriver::init()で設定済み
void PlanProfile::init() {
    motorDriver_.state = MotorDriverState::setDuty;
    motorDriver_.setDuty(0.f, 0.f);

    motorDriver_.setTargetAccelX(0.f);
    motorDriver_.setTargetVelocityX(0.f);
    resetTargetPositionX();

    motorDriver_.setTargetAlpha(0.f);
    motorDriver_.setTargetOmega(0.f);
    motorDriver_.resetTargetAngle();
    resetCurrentAngle();
}

void PlanProfile::resetTargetPositionX(void) {
    encoderLeft.reset();
    encoderRight.reset();
    current_position_x_ = 0.f;
    motorDriver_.setTargetPositionX(0.f);
}

void PlanProfile::resetCurrentAngle(void) {
    angle_origin_ = imu.gyroAngleZ();
    current_angle_ = 0.f;
}

void PlanProfile::update() {
    current_velocity_x_ = (encoderLeft.velocity() + encoderRight.velocity()) / 2.f;
    current_position_x_ = (encoderLeft.distance() + encoderRight.distance()) / 2.f;

    current_omega_ = imu.gyroZ();
    current_angle_ = imu.gyroAngleZ() - angle_origin_;
}


void PlanProfile::stepVelocity(float target_velocity_x, float distance) {
    motorDriver_.switchToVelocityX();
    motorDriver_.setTargetVelocityX(target_velocity_x);
    motorDriver_.setTargetAccelX(0.f);
    resetTargetPositionX();

    while (fabsf(current_position_x_) < fabsf(distance)) {
        // wait
    }
}

void PlanProfile::stepAccel(float target_accel_x, float distance) {
    motorDriver_.switchToVelocityX();
    motorDriver_.setTargetAccelX(target_accel_x);
    resetTargetPositionX();

    while (fabsf(current_position_x_) < fabsf(distance)) {
        // wait
    }
}

void PlanProfile::vel2vel(float velocity1, float velocity2, float distance) {
    motorDriver_.switchToVelocityX();
    motorDriver_.setTargetVelocityX(velocity1);
    motorDriver_.setTargetAccelX((velocity2 * velocity2 - velocity1 * velocity1) / (2.f * distance));  // v2^2 = v1^2 + 2*a*d
    resetTargetPositionX();

    while (fabsf(current_position_x_) < fabsf(distance)) {
        // wait
    }

    motorDriver_.setTargetVelocityX(velocity2);
}

void PlanProfile::stop(void) {
    motorDriver_.setTargetAccelX(0.f);
    motorDriver_.setTargetVelocityX(0.f);
}
