#include "menu/menu.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "device/device_test.hpp"
#include "test/motor_id.hpp"
#include "test/plan_profile_test.hpp"
#include "test/fan_id.hpp"

Menu::Menu() {
    buildTree();
    setFunction();
}

void Menu::buildTree() {
    root_.setChildren(std::array{&run_, &device_, &log_dump_});

        device_.setChildren(std::array{&imu_, &encoder_, &motor_, &fan_, &ir_, &battery_, &led_});
            fan_.setChildren(std::array{&fan_run_010_, &fan_run_020_, &fan_run_030_, &fan_run_040_, &fan_bringup_, &fan_hold_010_, &fan_hold_020_, &fan_hold_030_, &fan_hold_040_});
            ir_.setChildren(std::array{&ir_r_, &ir_fr_, &ir_fl_, &ir_l_});
            encoder_.setChildren(std::array{&encoder_r_, &encoder_l_});
            imu_.setChildren(std::array{&imu_gyro_, &imu_acc_});
            motor_.setChildren(std::array{&motor_r_, &motor_l_, &motor_velocity_x_, &motor_omega_, &plan_profile_, &plan_rotation_});
                motor_r_.setChildren(std::array{&right_050_});
                motor_velocity_x_.setChildren(std::array{&velocity_step_300_, &velocity_step_600_, &velocity_step_900_, &velocity_step_000_});
                plan_profile_.setChildren(std::array{&plan_step_velocity_, &plan_step_accel_, &plan_vel2vel_, &plan_encoder_check_, &plan_encoder_check_fan_, &plan_fast_2000_});
                plan_rotation_.setChildren(std::array{&plan_turn_pos430_, &plan_turn_neg430_, &plan_turn_step_alpha_pos430_});
                motor_omega_.setChildren(std::array{&omega_ramp_pos430_, &omega_ramp_neg430_, &omega_ramp_pos250_, &omega_ramp_neg250_, &omega_ramp_pos100_, &omega_ramp_neg100_});

    
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
    log_dump_.setOnEnter(log_dump_onenter);

    run_.setOnSelected(run_onselect);
    device_.setOnSelected(device_onselect);

    imu_.setOnSelected(imu_onselect);

    battery_.setOnEnter(battery_onenter);

    // ---------------------------

    imu_acc_.setOnEnter(imu_acc_onenter);
    imu_gyro_.setOnEnter(imu_gyro_onenter);
    encoder_l_.setOnEnter(encoder_left_onenter);
    encoder_r_.setOnEnter(encoder_right_onenter);

    right_050_.setOnEnter(right_set050_onenter);

    fan_run_010_.setOnEnter(fan_run_010_onenter);
    fan_run_020_.setOnEnter(fan_run_020_onenter);
    fan_run_030_.setOnEnter(fan_run_030_onenter);
    fan_run_040_.setOnEnter(fan_run_040_onenter);
    fan_bringup_.setOnEnter(fan_bringup_onenter);
    fan_hold_010_.setOnEnter(fan_hold_010_onenter);
    fan_hold_020_.setOnEnter(fan_hold_020_onenter);
    fan_hold_030_.setOnEnter(fan_hold_030_onenter);
    fan_hold_040_.setOnEnter(fan_hold_040_onenter);

    velocity_step_300_.setOnEnter(velocity_step_300_onenter);
    velocity_step_600_.setOnEnter(velocity_step_600_onenter);
    velocity_step_900_.setOnEnter(velocity_step_900_onenter);
    velocity_step_000_.setOnEnter(velocity_step_000_onenter);

    plan_step_velocity_.setOnEnter(plan_step_velocity_onenter);
    plan_step_accel_.setOnEnter(plan_step_accel_onenter);
    plan_vel2vel_.setOnEnter(plan_vel2vel_onenter);
    plan_encoder_check_.setOnEnter(plan_encoder_check_onenter);
    plan_encoder_check_fan_.setOnEnter(plan_encoder_check_fan_onenter);
    plan_fast_2000_.setOnEnter(plan_fast_2000_onenter);
    plan_turn_pos430_.setOnEnter(plan_turn_pos430_onenter);
    plan_turn_neg430_.setOnEnter(plan_turn_neg430_onenter);
    plan_turn_step_alpha_pos430_.setOnEnter(plan_turn_step_alpha_pos430_onenter);

    omega_ramp_pos430_.setOnEnter(omega_ramp_pos430_onenter);
    omega_ramp_neg430_.setOnEnter(omega_ramp_neg430_onenter);
    omega_ramp_pos250_.setOnEnter(omega_ramp_pos250_onenter);
    omega_ramp_neg250_.setOnEnter(omega_ramp_neg250_onenter);
    omega_ramp_pos100_.setOnEnter(omega_ramp_pos100_onenter);
    omega_ramp_neg100_.setOnEnter(omega_ramp_neg100_onenter);
}