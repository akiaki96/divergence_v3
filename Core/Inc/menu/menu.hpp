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
            MenuNode motor_rot_step_v700_xl_{"Motor rot step v700 XL"};
                MenuNode rot_step_v700_pos_020_{"rot v700 +duty0.20"};
                MenuNode rot_step_v700_pos_028_{"rot v700 +duty0.28"};
                MenuNode rot_step_v700_neg_020_{"rot v700 -duty0.20"};
                MenuNode rot_step_v700_neg_028_{"rot v700 -duty0.28"};
                MenuNode rot_step_v700_pos_022_{"rot v700 +0.22 2s"};
                MenuNode rot_step_v700_pos_024_{"rot v700 +0.24 2s"};
                MenuNode rot_step_v700_pos_026_{"rot v700 +0.26 2s"};
                MenuNode rot_step_v700_neg_022_{"rot v700 -0.22 2s"};
                MenuNode rot_step_v700_neg_024_{"rot v700 -0.24 2s"};
                MenuNode rot_step_v700_neg_026_{"rot v700 -0.26 2s"};
            MenuNode motor_rot_prbs_v700_{"Motor rot PRBS v700"};
                MenuNode prbs_rot_0_{"rot prbs 0"};
                MenuNode prbs_rot_1_{"rot prbs 1"};
                MenuNode prbs_rot_2_{"rot prbs 2"};
                MenuNode prbs_rot_3_{"rot prbs 3"};
                MenuNode prbs_rot_4_{"rot prbs val0"};
                MenuNode prbs_rot_5_{"rot prbs val1"};
            MenuNode motor_omega_step_{"Motor omega ff"};
                MenuNode omega_f6_on_pos430_{"f6 on +430"};
                MenuNode omega_f6_off_pos430_{"f6 off +430"};
                MenuNode omega_f6_on_neg430_{"f6 on -430"};
                MenuNode omega_f6_off_neg430_{"f6 off -430"};
                MenuNode omega_f6_on_pos250_{"f6 on +250"};
                MenuNode omega_f6_off_pos250_{"f6 off +250"};
                MenuNode omega_f6_on_neg250_{"f6 on -250"};
                MenuNode omega_f6_off_neg250_{"f6 off -250"};
                MenuNode omega_f6_on_pos100_{"f6 on +100"};
                MenuNode omega_f6_on_neg100_{"f6 on -100"};
            MenuNode motor_velocity_x_{"Motor velocity X"};
                MenuNode velocity_step_300_{"v step +300"};
                MenuNode velocity_step_600_{"v step +600"};
                MenuNode velocity_step_900_{"v step +900"};
                MenuNode velocity_step_neg600_{"v step -600"};


        MenuNode fan_{"Fan"};
            MenuNode fan_run_025_{"fan vsag 0.25"};
            MenuNode fan_run_050_{"fan vsag 0.50"};
            MenuNode fan_run_075_{"fan vsag 0.75"};
            MenuNode fan_run_100_{"fan vsag 1.00"};
        MenuNode ir_{"IR"};
            MenuNode ir_r_{"IR Right"};
            MenuNode ir_fr_{"IR Front Right"};
            MenuNode ir_fl_{"IR Front Left"};
            MenuNode ir_l_{"IR Left"};
        MenuNode battery_{"Battery"};
        MenuNode led_{"LED"};

};
