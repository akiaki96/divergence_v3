#pragma once

#include "menu/menuNode.hpp"

class Menu {
public:
    Menu();

    MenuNode* root() {
        return &root_;
    };

    void tree() const {
        root_.tree();
    }

private:
    void buildTree();

private:
    MenuNode root_{"Root"};

    MenuNode run_{"Run"};
    MenuNode device_{"Device"};

    MenuNode imu_{"IMU"};
    MenuNode encoder_{"Encoder"};
    MenuNode motor_{"Motor"};
    MenuNode fan_{"Fan"};
    MenuNode ir_{"IR"};
    MenuNode battery_{"Battery"};
    MenuNode led_{"LED"};

    MenuNode ir_r_{"IR Right"};
    MenuNode ir_fr_{"IR Front Right"};
    MenuNode ir_fl_{"IR Front Left"};
    MenuNode ir_l_{"IR Left"};

    MenuNode encoder_l_{"Encoder Left"};
    MenuNode encoder_r_{"Encoder Right"};

    MenuNode imu_acc_{"IMU Accel"};
    MenuNode imu_gyro_{"IMU Gyros"};

    MenuNode motor_l_{"Motor Left"};
    MenuNode motor_r_{"Motor Right"};
};