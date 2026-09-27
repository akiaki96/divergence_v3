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

    MenuNode log_dump_{"LogDump"};

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
            MenuNode motor_velocity_x_{"Motor velocity X"};
                MenuNode velocity_step_300_{"v step +300"};
                MenuNode velocity_step_600_{"v step +600"};
                MenuNode velocity_step_900_{"v step +900"};
                MenuNode velocity_step_000_{"v step 0"};


        MenuNode fan_{"Fan"};
        MenuNode ir_{"IR"};
            MenuNode ir_r_{"IR Right"};
            MenuNode ir_fr_{"IR Front Right"};
            MenuNode ir_fl_{"IR Front Left"};
            MenuNode ir_l_{"IR Left"};
        MenuNode battery_{"Battery"};
        MenuNode led_{"LED"};

};
