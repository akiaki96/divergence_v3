#include "menu/menu.hpp"
#include "device/device_instance.hpp"

Menu::Menu() {
    buildTree();
    setFunction();
}

void Menu::buildTree() {
    root_.setChildren(std::array{&run_, &device_});

    device_.setChildren(std::array{&imu_, &encoder_, &motor_, &fan_, &ir_, &battery_, &led_});

    ir_.setChildren(std::array{&ir_r_, &ir_fr_, &ir_fl_, &ir_l_});
    encoder_.setChildren(std::array{&encoder_r_, &encoder_l_});
    imu_.setChildren(std::array{&imu_acc_, &imu_gyro_});
    motor_.setChildren(std::array{&motor_r_, &motor_l_});
    
    root_.setParentRec();
    root_.setParent(&root_);
}

void run_onselect() {
    for (uint8_t i = 0; i < 16; ++i) {
        ledBar16.set(1<<i);
        HAL_Delay(100);
    }
}

void Menu::setFunction() {
    device_.setOnSelected(run_onselect);
}
