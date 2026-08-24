#pragma once

#include "menu/menuNode.hpp"
#include "config/node_func_maker.hpp"

class Menu {
public:
    Menu();

    const MenuNode* root() {
        return &root_;
    };

    void tree() const {
        root_.tree();
    }

private:
    void buildTree();
    void setFunction();

private:
    MenuNode root_{"Root"};

    MenuNode log_test_{"LogTest"};
        MenuNode log_dump_{"LogDump"};
        MenuNode log_wait_{"LogWait"};

    MenuNode run_{"Run"};

    MenuNode device_{"Device"};
        MenuNode imu_{"IMU"};
            MenuNode imu_acc_{"IMU Accel"};
            MenuNode imu_gyro_{"IMU Gyros"};
        MenuNode encoder_{"Encoder"};
            MenuNode encoder_l_{"Encoder Left"};
            MenuNode encoder_r_{"Encoder Right"};
        MenuNode motor_{"Motor"};
            MenuNode motor_l_{"Motor Left"};
            MenuNode motor_r_{"Motor Right"};
                MenuNode right_050_{"Motor Right 0.5"};
            MenuNode motor_sysid_step_{"Motor sysid step"};
            MenuNode motor_sysid_lamp_{"Motor sysid Lamp"};
                MenuNode lamp_005sec_{"lamp 0.05/sec"};
                MenuNode lamp_010sec_{"lamp 0.10/sec"};
                MenuNode lamp_030sec_{"lamp 0.30/sec"};
                MenuNode lamp_050sec_{"lamp 0.50/sec"};
                MenuNode lamp_070sec_{"lamp 0.70/sec"};
        MenuNode fan_{"Fan"};
        MenuNode ir_{"IR"};
            MenuNode ir_r_{"IR Right"};
            MenuNode ir_fr_{"IR Front Right"};
            MenuNode ir_fl_{"IR Front Left"};
            MenuNode ir_l_{"IR Left"};
        MenuNode battery_{"Battery"};
        MenuNode led_{"LED"};

};
