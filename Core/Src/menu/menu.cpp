#include "menu/menu.hpp"

Menu::Menu() {
    buildTree();
}

void Menu::buildTree() {
    root_.setChildren(std::array{&run_, &device_});

    device_.setChildren(std::array{&imu_, &encoder_, &motor_, &fan_, &ir_, &battery_, &led_});

    ir_.setChildren(std::array{&ir_r_, &ir_fr_, &ir_fl_, &ir_l_});
    encoder_.setChildren(std::array{&encoder_r_, &encoder_l_});
    imu_.setChildren(std::array{&imu_acc_, &imu_gyro_});
    motor_.setChildren(std::array{&motor_r_, &motor_l_});
    
    root_.setParentRec();
}