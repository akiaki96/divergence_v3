#include "menu/menu.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "device/device_test.hpp"
#include "test/motor_id.hpp"

Menu::Menu() {
    buildTree();
    setFunction();
}

void Menu::buildTree() {
    root_.setChildren(std::array{&run_, &device_, &log_dump_});

        device_.setChildren(std::array{&imu_, &encoder_, &motor_, &fan_, &ir_, &battery_, &led_});
            ir_.setChildren(std::array{&ir_r_, &ir_fr_, &ir_fl_, &ir_l_});
            encoder_.setChildren(std::array{&encoder_r_, &encoder_l_});
            imu_.setChildren(std::array{&imu_gyro_, &imu_acc_});
            motor_.setChildren(std::array{&motor_r_, &motor_l_, &motor_sysid_lamp_, &motor_sysid_step_, &motor_sysid_prbs_, &motor_rot_step_v700_, &motor_rot_step_v700_xl_, &motor_rot_prbs_v700_, &motor_omega_step_, &motor_velocity_x_});
                motor_r_.setChildren(std::array{&right_050_});
                motor_sysid_step_.setChildren(std::array{&step_010_, &step_015_, &step_020_, &step_log_});
                motor_sysid_lamp_.setChildren(std::array{&lamp_005sec_, &lamp_010sec_, &lamp_030sec_, &lamp_050sec_, &lamp_070sec_, &log_dump_});
                motor_sysid_prbs_.setChildren(std::array{&prbs_0_, &prbs_1_, &prbs_2_, &prbs_3_, &prbs_4_, &prbs_5_, &prbs_6_, &prbs_7_, &prbs_8_, &prbs_9_});
                motor_rot_step_v700_.setChildren(std::array{&rot_step_v700_pos_002_, &rot_step_v700_pos_004_, &rot_step_v700_pos_006_, &rot_step_v700_pos_010_, &rot_step_v700_pos_014_, &rot_step_v700_neg_002_, &rot_step_v700_neg_004_, &rot_step_v700_neg_006_, &rot_step_v700_neg_010_, &rot_step_v700_neg_014_});
                motor_rot_step_v700_xl_.setChildren(std::array{&rot_step_v700_pos_020_, &rot_step_v700_pos_028_, &rot_step_v700_neg_020_, &rot_step_v700_neg_028_, &rot_step_v700_pos_022_, &rot_step_v700_pos_024_, &rot_step_v700_pos_026_, &rot_step_v700_neg_022_, &rot_step_v700_neg_024_, &rot_step_v700_neg_026_});
                motor_rot_prbs_v700_.setChildren(std::array{&prbs_rot_0_, &prbs_rot_1_, &prbs_rot_2_, &prbs_rot_3_, &prbs_rot_4_, &prbs_rot_5_});
                motor_omega_step_.setChildren(std::array{&omega_step_pos200_, &omega_step_pos250_, &omega_step_pos400_, &omega_step_neg200_, &omega_step_neg400_,
                    &omega_step_pos100_, &omega_step_neg100_, &omega_step_neg250_, &omega_step_pos430_, &omega_step_neg430_});
                motor_velocity_x_.setChildren(std::array{&velocity_step_300_, &velocity_step_600_, &velocity_step_900_, &velocity_step_neg600_});
        log_test_.setChildren(std::array{&log_wait_, &log_dump_});

    
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
    log_wait_.setOnEnter(log_wait_onenter);

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
    lamp_005sec_.setOnEnter(lamp_005sec_onenter);
    lamp_010sec_.setOnEnter(lamp_010sec_onenter);
    lamp_030sec_.setOnEnter(lamp_030sec_onenter);
    lamp_050sec_.setOnEnter(lamp_050sec_onenter);
    lamp_070sec_.setOnEnter(lamp_070sec_onenter);

    step_010_.setOnEnter(step_010_onenter);
    step_015_.setOnEnter(step_015_onenter);
    step_020_.setOnEnter(step_020_onenter);
    step_log_.setOnEnter(log_dump_onenter);

    prbs_0_.setOnEnter(prbs_trans_t01_onenter);
    prbs_1_.setOnEnter(prbs_trans_t02_onenter);
    prbs_2_.setOnEnter(prbs_trans_t03_onenter);
    prbs_3_.setOnEnter(prbs_trans_t04_onenter);
    prbs_4_.setOnEnter(prbs_trans_t05_onenter);
    prbs_5_.setOnEnter(prbs_trans_t06_onenter);
    prbs_6_.setOnEnter(prbs_trans_t07_onenter);
    prbs_7_.setOnEnter(prbs_trans_t08_onenter);
    prbs_8_.setOnEnter(prbs_trans_val01_onenter);
    prbs_9_.setOnEnter(prbs_trans_val02_onenter);

    rot_step_v700_pos_002_.setOnEnter(rot_step_v700_pos_002_onenter);
    rot_step_v700_pos_004_.setOnEnter(rot_step_v700_pos_004_onenter);
    rot_step_v700_pos_006_.setOnEnter(rot_step_v700_pos_006_onenter);
    rot_step_v700_pos_010_.setOnEnter(rot_step_v700_pos_010_onenter);
    rot_step_v700_pos_014_.setOnEnter(rot_step_v700_pos_014_onenter);
    rot_step_v700_neg_002_.setOnEnter(rot_step_v700_neg_002_onenter);
    rot_step_v700_neg_004_.setOnEnter(rot_step_v700_neg_004_onenter);
    rot_step_v700_neg_006_.setOnEnter(rot_step_v700_neg_006_onenter);
    rot_step_v700_neg_010_.setOnEnter(rot_step_v700_neg_010_onenter);
    rot_step_v700_neg_014_.setOnEnter(rot_step_v700_neg_014_onenter);

    rot_step_v700_pos_020_.setOnEnter(rot_step_v700_pos_020_onenter);
    rot_step_v700_pos_028_.setOnEnter(rot_step_v700_pos_028_onenter);
    rot_step_v700_neg_020_.setOnEnter(rot_step_v700_neg_020_onenter);
    rot_step_v700_neg_028_.setOnEnter(rot_step_v700_neg_028_onenter);

    rot_step_v700_pos_022_.setOnEnter(rot_step_v700_pos_022_onenter);
    rot_step_v700_pos_024_.setOnEnter(rot_step_v700_pos_024_onenter);
    rot_step_v700_pos_026_.setOnEnter(rot_step_v700_pos_026_onenter);
    rot_step_v700_neg_022_.setOnEnter(rot_step_v700_neg_022_onenter);
    rot_step_v700_neg_024_.setOnEnter(rot_step_v700_neg_024_onenter);
    rot_step_v700_neg_026_.setOnEnter(rot_step_v700_neg_026_onenter);

    prbs_rot_0_.setOnEnter(prbs_rot_t01_onenter);
    prbs_rot_1_.setOnEnter(prbs_rot_t02_onenter);
    prbs_rot_2_.setOnEnter(prbs_rot_t03_onenter);
    prbs_rot_3_.setOnEnter(prbs_rot_t04_onenter);
    prbs_rot_4_.setOnEnter(prbs_rot_val01_onenter);
    prbs_rot_5_.setOnEnter(prbs_rot_val02_onenter);

    omega_step_pos200_.setOnEnter(omega_step_pos200_onenter);
    omega_step_pos250_.setOnEnter(omega_step_pos250_onenter);
    omega_step_pos400_.setOnEnter(omega_step_pos400_onenter);
    omega_step_neg200_.setOnEnter(omega_step_neg200_onenter);
    omega_step_neg400_.setOnEnter(omega_step_neg400_onenter);
    omega_step_pos100_.setOnEnter(omega_step_pos100_onenter);
    omega_step_neg100_.setOnEnter(omega_step_neg100_onenter);
    omega_step_neg250_.setOnEnter(omega_step_neg250_onenter);
    omega_step_pos430_.setOnEnter(omega_step_pos430_onenter);
    omega_step_neg430_.setOnEnter(omega_step_neg430_onenter);

    velocity_step_300_.setOnEnter(velocity_step_300_onenter);
    velocity_step_600_.setOnEnter(velocity_step_600_onenter);
    velocity_step_900_.setOnEnter(velocity_step_900_onenter);
    velocity_step_neg600_.setOnEnter(velocity_step_neg600_onenter);
}