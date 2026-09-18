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
                MenuNode step_010_{"step 0.10"};
                MenuNode step_015_{"step 0.15"};
                MenuNode step_020_{"step 0.20"};
                MenuNode step_log_{"step log"};
            MenuNode motor_sysid_lamp_{"Motor sysid Lamp"};
                MenuNode lamp_005sec_{"lamp 0.05/sec"};
                MenuNode lamp_010sec_{"lamp 0.10/sec"};
                MenuNode lamp_030sec_{"lamp 0.30/sec"};
                MenuNode lamp_050sec_{"lamp 0.50/sec"};
                MenuNode lamp_070sec_{"lamp 0.70/sec"};
            MenuNode motor_sysid_prbs_{"Motor sysid PRBS"};
                MenuNode prbs_0_{"prbs 0"};
                MenuNode prbs_1_{"prbs 1"};
                MenuNode prbs_2_{"prbs 2"};
                MenuNode prbs_3_{"prbs 3"};
                MenuNode prbs_4_{"prbs 4"};
                MenuNode prbs_5_{"prbs 5"};
                MenuNode prbs_6_{"prbs 6"};
                MenuNode prbs_7_{"prbs 7"};
                MenuNode prbs_8_{"prbs 8"};
                MenuNode prbs_9_{"prbs 9"};
            MenuNode motor_rot_step_v700_{"Motor rot step v700"};
                MenuNode rot_step_v700_pos_002_{"rot v700 +duty0.02"};
                MenuNode rot_step_v700_pos_004_{"rot v700 +duty0.04"};
                MenuNode rot_step_v700_pos_006_{"rot v700 +duty0.06"};
                MenuNode rot_step_v700_pos_010_{"rot v700 +duty0.10"};
                MenuNode rot_step_v700_pos_014_{"rot v700 +duty0.14"};
                MenuNode rot_step_v700_neg_002_{"rot v700 -duty0.02"};
                MenuNode rot_step_v700_neg_004_{"rot v700 -duty0.04"};
                MenuNode rot_step_v700_neg_006_{"rot v700 -duty0.06"};
                MenuNode rot_step_v700_neg_010_{"rot v700 -duty0.10"};
                MenuNode rot_step_v700_neg_014_{"rot v700 -duty0.14"};
            MenuNode motor_velocity_x_{"Motor velocity X"};
                MenuNode velocity_step_300_{"v step +300"};
                MenuNode velocity_step_600_{"v step +600"};
                MenuNode velocity_step_900_{"v step +900"};
                MenuNode velocity_step_neg600_{"v step -600"};


        MenuNode fan_{"Fan"};
        MenuNode ir_{"IR"};
            MenuNode ir_r_{"IR Right"};
            MenuNode ir_fr_{"IR Front Right"};
            MenuNode ir_fl_{"IR Front Left"};
            MenuNode ir_l_{"IR Left"};
        MenuNode battery_{"Battery"};
        MenuNode led_{"LED"};

};
