#include "device/odometry.hpp"
#include "config/mouse_config.hpp"

Odometry::Odometry(Encoder& left, Encoder& right, Imu& imu)
    : left_(left), right_(right), imu_(imu)
{}

void Odometry::update() {
    velocity_x_ = (left_.velocity() + right_.velocity()) / 2.f;
    position_x_ = (left_.distance() + right_.distance()) / 2.f + position_shift_x_;

    omega_ = imu_.gyroZ();
    encoder_omega_ = (right_.velocity() - left_.velocity()) / 2.f / config::mouse::WHEEL_DIFF_PER_DPS;
    angle_ = imu_.gyroAngleZ() - angle_origin_;
}

void Odometry::reset() {
    left_.reset();
    right_.reset();
    position_shift_x_ = 0.f;
    position_x_ = 0.f;

    angle_origin_ = imu_.gyroAngleZ();
    angle_ = 0.f;
}
