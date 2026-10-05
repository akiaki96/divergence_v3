#include "menu/menu.hpp"
#include "app/wall_edge_log.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "device/device_test.hpp"
#include "test/motor_id.hpp"
#include "test/plan_profile_test.hpp"
#include "test/fan_id.hpp"
#include "test/rotation_test.hpp"

Menu::Menu() {
    buildTree();
    setFunction();
}

void Menu::buildTree() {
    root_.setChildren(std::array{&run_, &device_, &log_dump_, &slalom_, &test_});

        run_.setChildren(std::array{&search_, &fast_, &maze_});
            search_.setChildren(pointersOf(search_presets_));
            fast_.setChildren(pointersOf(fast_presets_));
            maze_.setChildren(std::array{&maze_show_, &maze_clear_});

        device_.setChildren(std::array{&imu_, &encoder_, &motor_, &fan_, &ir_, &battery_, &led_});
            fan_.setChildren(std::array{&fan_run_010_, &fan_run_020_, &fan_run_030_, &fan_run_040_, &fan_bringup_, &fan_hold_010_, &fan_hold_020_, &fan_hold_030_, &fan_hold_040_});
            ir_.setChildren(std::array{&ir_wall_check_, &ir_front_check_, &ir_wall_edge_, &ir_r_, &ir_fr_, &ir_fl_, &ir_l_});
                ir_front_check_.setChildren(std::array{&front_check_wall_, &front_check_no_wall_, &front_check_show_, &front_check_reset_});
                ir_wall_edge_.setChildren(std::array{&wall_edge_calib_300_, &wall_edge_calib_500_, &wall_edge_calib_700_, &wall_edge_verify_500_, &wall_edge_inject_500_});
            encoder_.setChildren(std::array{&encoder_r_, &encoder_l_});
            imu_.setChildren(std::array{&imu_gyro_, &imu_acc_, &imu_gyro_fan_, &imu_acc_fan_});
            motor_.setChildren(std::array{&motor_r_, &motor_l_, &motor_velocity_x_, &plan_profile_, &plan_rotation_, &rotation_});
                motor_r_.setChildren(std::array{&right_050_});
                motor_velocity_x_.setChildren(std::array{&velocity_step_300_, &velocity_step_600_, &velocity_step_900_, &velocity_step_000_});
                plan_profile_.setChildren(std::array{&plan_step_velocity_, &plan_vel2vel_, &plan_encoder_check_, &plan_encoder_check_fan_, &plan_fast_2000_});
                plan_rotation_.setChildren(std::array{&plan_turn_pos430_, &plan_turn_neg430_});
                rotation_.setChildren(std::array{&rot_angle_hold_, &rot_pivot_pos90_, &rot_pivot_neg90_, &rot_pivot_pos180_, &rot_pivot_neg180_});

    // 種類・速度のノードは生成ヘッダから並べる（menu.hppのslalomTurnNodes() / slalomSpeedNodes()）
    slalom_.setChildren(std::array{&slalom_left_, &slalom_right_, &axle_check_});
        axle_check_.setChildren(std::array{&axle_check_n1_, &axle_check_n2_, &axle_check_n4_, &axle_check_n8_});
        linkSlalom(slalom_left_, slalom_left_turns_, slalom_left_speeds_);
        linkSlalom(slalom_right_, slalom_right_turns_, slalom_right_speeds_);

    // 試験用の探索のプリセットは生成ヘッダから並べる（menu.hppのtestSearchNodes()）
    test_.setChildren(std::array{&test_search_});
        test_search_.setChildren(pointersOf(test_search_presets_));

    
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

// 最後の走行のログをもう一度送る（受信に失敗したときの取り直し）。壁切れの記録があればそれも送る
onenter(log_dump, 
    logger.dump();
    wall_edge_log::dumpLast();
)

void Menu::setFunction() {
    log_dump_.setOnEnter(log_dump_onenter);

    run_.setOnSelected(run_onselect);
    maze_show_.setOnEnter(maze_show_onenter);
    maze_clear_.setOnEnter(maze_clear_onenter);
    device_.setOnSelected(device_onselect);

    imu_.setOnSelected(imu_onselect);

    battery_.setOnEnter(battery_onenter);
    ir_wall_check_.setOnEnter(wall_check_onenter);

    // ---------------------------

    imu_acc_.setOnEnter(imu_acc_onenter);
    imu_gyro_.setOnEnter(imu_gyro_onenter);
    imu_acc_fan_.setOnEnter(imu_acc_fan_onenter);
    imu_gyro_fan_.setOnEnter(imu_gyro_fan_onenter);
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
    plan_vel2vel_.setOnEnter(plan_vel2vel_onenter);
    plan_encoder_check_.setOnEnter(plan_encoder_check_onenter);
    plan_encoder_check_fan_.setOnEnter(plan_encoder_check_fan_onenter);
    plan_fast_2000_.setOnEnter(plan_fast_2000_onenter);
    plan_turn_pos430_.setOnEnter(plan_turn_pos430_onenter);
    plan_turn_neg430_.setOnEnter(plan_turn_neg430_onenter);
    rot_angle_hold_.setOnEnter(rot_angle_hold_onenter);
    rot_pivot_pos90_.setOnEnter(rot_pivot_pos90_onenter);
    rot_pivot_neg90_.setOnEnter(rot_pivot_neg90_onenter);
    rot_pivot_pos180_.setOnEnter(rot_pivot_pos180_onenter);
    rot_pivot_neg180_.setOnEnter(rot_pivot_neg180_onenter);
}