#include "menu/menu.hpp"
#include "common/debug.hpp"
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


onselect(run, 
    ledManager.setall(true, true, false, false, false, false);
)

onselect(device,
    ledManager.setall(true, true, true, true, true, true);
)

onselect(imu,
    ledManager.setall(false, false, false, false, true, false);
)

onenter(imu_acc, 
    while (true) {
        LOG("x: %+.2f, y: %+.2f, z: %+.2f \r\n", imu.accelX(), imu.accelY(), imu.accelZ());
    }
)



void Menu::setFunction() {
    run_.setOnSelected(run_onselect);
    device_.setOnSelected(device_onselect);

    imu_.setOnSelected(imu_onselect);

    // ---------------------------

    imu_acc_.setOnEnter(imu_acc_onenter);
}