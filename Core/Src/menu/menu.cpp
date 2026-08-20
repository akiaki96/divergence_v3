#include "menu/menu.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "device/device_test.hpp"

Menu::Menu() {
    buildTree();
    setFunction();
}

void Menu::buildTree() {
    root_.setChildren(std::array{&run_, &device_, &log_test_});

    device_.setChildren(std::array{&imu_, &encoder_, &motor_, &fan_, &ir_, &battery_, &led_});
    log_test_.setChildren(std::array{&log_wait_, &log_dump_});

    ir_.setChildren(std::array{&ir_r_, &ir_fr_, &ir_fl_, &ir_l_});
    encoder_.setChildren(std::array{&encoder_r_, &encoder_l_});
    // imu_.setChildren(std::array{&imu_acc_, &imu_gyro_});
    imu_.setChildren(std::array{&imu_gyro_, &imu_acc_});
    motor_.setChildren(std::array{&motor_r_, &motor_l_});
    
    root_.setParentRec();
    root_.setParent(&root_);
}


onselect(run, 
    LOG("on run\r\n");
    ledManager.setall(true, true, false, false, false, false);
)

onselect(device,
    LOG("on device\r\n");
    ledManager.setall(true, true, true, true, true, true);
)

onselect(imu,
    LOG("on imu\r\n");
    ledManager.setall(false, false, false, false, true, false);
)

onenter(log_dump, 
    logger.dump();
)

void Menu::setFunction() {
    // log_dump_.setOnEnter(log_dump_onenter);

    run_.setOnSelected(run_onselect);
    device_.setOnSelected(device_onselect);

    imu_.setOnSelected(imu_onselect);

    // ---------------------------

    imu_acc_.setOnEnter(imu_acc_onenter);
    imu_gyro_.setOnEnter(imu_gyro_onenter);
    encoder_l_.setOnEnter(encoder_left_onenter);
    encoder_r_.setOnEnter(encoder_right_onenter);
}