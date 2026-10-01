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
            MenuNode plan_profile_{"Plan profile"};
                MenuNode plan_step_velocity_{"step velocity"};
                MenuNode plan_vel2vel_{"straight"};
                MenuNode plan_encoder_check_{"encoder check 50"};
                MenuNode plan_encoder_check_fan_{"encoder check 50 fan0.2"};
                MenuNode plan_fast_2000_{"fast 2000 fan0.2"};
            MenuNode plan_rotation_{"Plan rotation"};
                MenuNode plan_turn_pos430_{"turn +430"};
                MenuNode plan_turn_neg430_{"turn -430"};
            MenuNode rotation_{"Rotation"};
                MenuNode rot_angle_hold_{"angle hold"};
                MenuNode rot_pivot_pos90_{"pivot +90"};
                MenuNode rot_pivot_neg90_{"pivot -90"};
                MenuNode rot_pivot_pos180_{"pivot +180"};
                MenuNode rot_pivot_neg180_{"pivot -180"};


        MenuNode fan_{"Fan"};
            MenuNode fan_run_010_{"fan vsag 0.10"};
            MenuNode fan_run_020_{"fan vsag 0.20"};
            MenuNode fan_run_030_{"fan vsag 0.30"};
            MenuNode fan_run_040_{"fan vsag 0.40"};
            MenuNode fan_bringup_{"fan bringup"};
            MenuNode fan_hold_010_{"fan hold 0.10"};
            MenuNode fan_hold_020_{"fan hold 0.20"};
            MenuNode fan_hold_030_{"fan hold 0.30"};
            MenuNode fan_hold_040_{"fan hold 0.40"};
        MenuNode ir_{"IR"};
            MenuNode ir_r_{"IR Right"};
            MenuNode ir_fr_{"IR Front Right"};
            MenuNode ir_fl_{"IR Front Left"};
            MenuNode ir_l_{"IR Left"};
        MenuNode battery_{"Battery"};
        MenuNode led_{"LED"};

};
