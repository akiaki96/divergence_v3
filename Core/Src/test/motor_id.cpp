#include "test/motor_id.hpp"
#include "common/etc.hpp"
#include "common/prbs.hpp"
#include "config/mouse_config.hpp"

void id_init_log(void) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);

    logger.initLoggedVal();
    logger.add(
        "left_encoder_velocity",
        etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderLeft)
    );
    logger.add(
        "right_encoder_velocity",
        etl::delegate<float()>::create<Encoder, &Encoder::velocity>(encoderRight)
    );
    logger.add(
        "battery",
        etl::delegate<float()>::create<Battery, &Battery::voltage>(battery)
    );
    logger.add(
        "Left Duty",
        etl::delegate<float()>::create<Motor, &Motor::getDuty>(motorLeft)
    );
    logger.add(
        "Right Duty",
        etl::delegate<float()>::create<Motor, &Motor::getDuty>(motorRight)
    );
    logger.add(
        "gyro_z",
        etl::delegate<float()>::create<Imu, &Imu::gyroZ>(imu)
    );
    logger.add(
        "accel_x",
        etl::delegate<float()>::create<Imu, &Imu::accelX>(imu)
    );

    ledBar16.set(0xFFFF);
}

// 並進速度PI+FF制御の追従性検証用：id_init_log()の共通フィールドに加え，
// 目標速度target_velocity_xも記録する（実速度との比較でステップ応答・追従誤差を評価するため）
void id_init_log_velocity(void) {
    id_init_log();
    logger.add(
        "target_velocity_x",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetVelocityX>(motorDriver)
    );
    // 積分ワインドアップとfeedforward寄与を確認するための診断フィールド
    logger.add(
        "pid_integral_term",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXIntegralTerm>(motorDriver)
    );
    logger.add(
        "pid_feedforward",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXFeedforward>(motorDriver)
    );
    logger.add(
        "pid_saturated",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getVelocityXSaturated>(motorDriver)
    );
}

onenter(right_set050, 
    id_init_log();
    logger.dirName = "right_set0_50";

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setDuty(0.f, 0.5f);
    HAL_Delay(1000);
    motorDriver.setBreak();
    HAL_Delay(500);
    logger.stop();
    ledBar16.set(0xFFFF);
)

void lamp_tester(float lamp) {
    HAL_Delay(500);

    motorDriver.state = MotorDriverState::lampDuty;
    motorDriver.setLampGrad(0.f);

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setLampGrad(lamp);
    HAL_Delay(1000);
    motorDriver.setBreak();
    HAL_Delay(500);
    logger.stop();
    ledBar16.set(0xFFFF);
}

onenter(lamp_005sec, 
    id_init_log();
    logger.dirName = "lamp_0_05sec";
    lamp_tester(0.05);
)

onenter(lamp_010sec, 
    id_init_log();
    logger.dirName = "lamp_0_10sec";
    lamp_tester(0.1);
)

onenter(lamp_030sec, 
    id_init_log();
    logger.dirName = "lamp_0_30sec";
    lamp_tester(0.3);
)

onenter(lamp_050sec, 
    id_init_log();
    logger.dirName = "lamp_0_50sec";
    lamp_tester(0.5);
)

onenter(lamp_070sec, 
    id_init_log();
    logger.dirName = "lamp_0_70sec";
    lamp_tester(0.7);
)


void step_tester(float step) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setDuty(step, step);
    HAL_Delay(1000);
    motorDriver.setBreak();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();

    logger.dump();
    ledBar16.set(0x0000);
}

onenter(step_010,
    id_init_log();
    logger.setDirName("step_x");
    logger.setFileName("step_0_10");
    logger.includeTimestamp = false;
    step_tester(0.1f);
)

onenter(step_015,
    id_init_log();
    logger.setDirName("step_x");
    logger.setFileName("step_0_15");
    logger.includeTimestamp = false;
    step_tester(0.15f);
)

onenter(step_020,
    id_init_log();
    logger.setDirName("step_x");
    logger.setFileName("step_0_20");
    logger.includeTimestamp = false;
    step_tester(0.2f);
)

struct PRBSTransParams {
    uint16_t seed;
    float duty_min;
    float duty_max;
    float Tc_sec;
    float duration_sec;
};

PRBS g_prbs;

// duty_min/duty_max/Tc/durationはmouse_config.hpp(config::prbs_trans)の設計値を使用し，
// 試行ごとに異なるseedのみを差し替える
PRBSTransParams prbs_trans_params(uint16_t seed) {
    return {seed,
            config::prbs_trans::DUTY_MIN,
            config::prbs_trans::DUTY_MAX,
            config::prbs_trans::TC_SEC,
            config::prbs_trans::DURATION_SEC};
}

void prbs_trans_tester(const PRBSTransParams& p) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    uint32_t ticks_per_clock = static_cast<uint32_t>(p.Tc_sec * 1000.f + 0.5f);
    uint32_t total_ticks     = static_cast<uint32_t>(p.duration_sec * 1000.f + 0.5f);
    g_prbs.configure(p.seed, p.duty_min, p.duty_max, ticks_per_clock, total_ticks);

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);

    motorDriver.setPRBS(&g_prbs);

    HAL_Delay(static_cast<uint32_t>(p.duration_sec * 1000.f) + 100);

    motorDriver.setBreak();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(prbs_trans_t01,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t01");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T01));
)

onenter(prbs_trans_t02,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t02");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T02));
)

onenter(prbs_trans_t03,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t03");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T03));
)

onenter(prbs_trans_t04,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t04");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T04));
)

onenter(prbs_trans_t05,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t05");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T05));
)

onenter(prbs_trans_t06,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t06");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T06));
)

onenter(prbs_trans_t07,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t07");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T07));
)

onenter(prbs_trans_t08,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_t08");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_T08));
)

onenter(prbs_trans_val01,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_val01");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_VAL01));
)

onenter(prbs_trans_val02,
    id_init_log();
    logger.setDirName("prbs_trans_x");
    logger.setFileName("prbs_trans_val02");
    logger.setIncludeTimestamp(false);
    prbs_trans_tester(prbs_trans_params(config::prbs_trans::SEED_VAL02));
)


// 回転方向のstep応答事前同定用ログ：velocity診断フィールドに加え，
// 左右duty差(duty_diff)も記録する（system_identification_flow.md §2 [2]）
void id_init_log_rot_v700(void) {
    id_init_log_velocity();
    logger.add(
        "duty_diff",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getDutyDiff>(motorDriver)
    );
}

// 並進速度を閉ループでconfig::rot_step_v700::TRANSLATION_VELOCITY_MM_Sに固定したまま，
// 左右duty差をステップ印加して回転方向の応答を励振する
void rot_step_v700_tester(float duty_diff) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用

    // フェーズ1：並進速度を閉ループで立ち上げる（回転励振なし）
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(config::rot_step_v700::TRANSLATION_VELOCITY_MM_S);
    HAL_Delay(config::rot_step_v700::ACCEL_MS);

    // フェーズ2：並進速度700mm/sを維持しつつ左右duty差をステップ印加
    motorDriver.setDutyDiff(duty_diff);
    HAL_Delay(config::rot_step_v700::TEST_MS);

    motorDriver.setBreak();
    motorDriver.setDutyDiff(0.f);   // 次回の試行に持ち越さない
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

// duty0が厳密な直進(v_L=v_R)にならない機体バイアスがあるため，
// 正負両方向のduty差を試して非対称性を切り分ける
onenter(rot_step_v700_pos_002,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos002");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_1);
)

onenter(rot_step_v700_pos_004,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos004");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_2);
)

onenter(rot_step_v700_pos_006,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos006");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_3);
)

onenter(rot_step_v700_pos_010,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos010");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_4);
)

onenter(rot_step_v700_pos_014,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos014");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_5);
)

onenter(rot_step_v700_neg_002,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg002");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_1);
)

onenter(rot_step_v700_neg_004,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg004");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_2);
)

onenter(rot_step_v700_neg_006,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg006");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_3);
)

onenter(rot_step_v700_neg_010,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg010");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_4);
)

onenter(rot_step_v700_neg_014,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg014");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_5);
)

// 実運用目標（700mm/s時，角加速度目安2500deg/s^2・最高角速度目安430deg/s）に対し，
// duty_diff<=0.14までの実測ではヨーレートが最大171dps程度までしか届いておらず，
// 運用域に向けた特性把握のため追加した大振幅水準（片輪はさらに深く負転する）
onenter(rot_step_v700_pos_020,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos020");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_6);
)

onenter(rot_step_v700_pos_028,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_pos028");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_7);
)

onenter(rot_step_v700_neg_020,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg020");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_6);
)

onenter(rot_step_v700_neg_028,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty_neg028");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(-config::rot_step_v700::DUTY_DIFF_7);
)

// 回転方向PRBS本同定（data_analysis2/prbs_rot_design.m）。
// 並進速度700mm/sを閉ループで維持しつつ，duty_diffをPRBS（±0.06, Tc=4ms）で励振する。
PRBS g_prbs_rot;

void prbs_rot_tester(uint16_t seed) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用

    // フェーズ1：並進速度700mm/sを閉ループで立ち上げる（回転励振なし）
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(config::prbs_rot::TRANSLATION_VELOCITY_MM_S);
    HAL_Delay(config::prbs_rot::ACCEL_MS);

    // フェーズ2：並進速度700mm/sを維持しつつduty_diffをPRBS励振
    uint32_t ticks_per_clock = static_cast<uint32_t>(config::prbs_rot::TC_SEC * 1000.f + 0.5f);
    uint32_t total_ticks     = static_cast<uint32_t>(config::prbs_rot::DURATION_SEC * 1000.f + 0.5f);
    g_prbs_rot.configure(seed, -config::prbs_rot::DUTY_DIFF_AMP, config::prbs_rot::DUTY_DIFF_AMP,
                          ticks_per_clock, total_ticks);
    motorDriver.setPRBSDutyDiff(&g_prbs_rot);

    HAL_Delay(static_cast<uint32_t>(config::prbs_rot::DURATION_SEC * 1000.f) + 100);

    motorDriver.setBreak();
    motorDriver.setPRBSDutyDiff(nullptr);
    motorDriver.setDutyDiff(0.f);   // 次回の試行に持ち越さない
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(prbs_rot_t01,
    id_init_log_rot_v700();
    logger.setDirName("prbs_rot_v700_x");
    logger.setFileName("prbs_rot_t01");
    logger.setIncludeTimestamp(false);
    prbs_rot_tester(config::prbs_rot::SEED_T01);
)

onenter(prbs_rot_t02,
    id_init_log_rot_v700();
    logger.setDirName("prbs_rot_v700_x");
    logger.setFileName("prbs_rot_t02");
    logger.setIncludeTimestamp(false);
    prbs_rot_tester(config::prbs_rot::SEED_T02);
)

onenter(prbs_rot_t03,
    id_init_log_rot_v700();
    logger.setDirName("prbs_rot_v700_x");
    logger.setFileName("prbs_rot_t03");
    logger.setIncludeTimestamp(false);
    prbs_rot_tester(config::prbs_rot::SEED_T03);
)

onenter(prbs_rot_t04,
    id_init_log_rot_v700();
    logger.setDirName("prbs_rot_v700_x");
    logger.setFileName("prbs_rot_t04");
    logger.setIncludeTimestamp(false);
    prbs_rot_tester(config::prbs_rot::SEED_T04);
)

onenter(prbs_rot_val01,
    id_init_log_rot_v700();
    logger.setDirName("prbs_rot_v700_x");
    logger.setFileName("prbs_rot_val01");
    logger.setIncludeTimestamp(false);
    prbs_rot_tester(config::prbs_rot::SEED_VAL01);
)

onenter(prbs_rot_val02,
    id_init_log_rot_v700();
    logger.setDirName("prbs_rot_v700_x");
    logger.setFileName("prbs_rot_val02");
    logger.setIncludeTimestamp(false);
    prbs_rot_tester(config::prbs_rot::SEED_VAL02);
)

// 回転角速度PI制御（2自由度ではなく純粋PI, config::pid_omega）の追従性検証用ログ：
// rot_v700診断フィールドに加え，目標角速度・積分項・飽和状態も記録する
void id_init_log_omega(void) {
    id_init_log_rot_v700();
    logger.add(
        "target_omega",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getTargetOmega>(motorDriver)
    );
    logger.add(
        "omega_integral_term",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getOmegaIntegralTerm>(motorDriver)
    );
    logger.add(
        "omega_saturated",
        etl::delegate<float()>::create<MotorDriver, &MotorDriver::getOmegaSaturated>(motorDriver)
    );
}

// 並進速度700mm/sを閉ループで維持しつつ，目標角速度target_omegaへステップ指令する。
// FFなしの純粋PI（config::pid_omega）が実際にgyro_zへ追従できるか検証する
void rot_omega_step_tester(float target_omega, uint32_t duration_ms) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用

    // フェーズ1：並進速度700mm/sを閉ループで立ち上げる（回転励振なし）
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(config::rot_step_v700::TRANSLATION_VELOCITY_MM_S);
    HAL_Delay(config::rot_step_v700::ACCEL_MS);

    // フェーズ2：並進速度700mm/sを維持しつつ角速度PIを有効化しステップ指令
    motorDriver.enableOmegaControl();
    motorDriver.setTargetOmega(target_omega);
    HAL_Delay(duration_ms);

    motorDriver.setBreak();
    motorDriver.disableOmegaControl();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(omega_step_pos200,
    id_init_log_omega();
    logger.setDirName("omega_step_v700_x");
    logger.setFileName("omega_step_pos200");
    logger.setIncludeTimestamp(false);
    rot_omega_step_tester(200.f, 800);
)

onenter(omega_step_pos400,
    id_init_log_omega();
    logger.setDirName("omega_step_v700_x");
    logger.setFileName("omega_step_pos400");
    logger.setIncludeTimestamp(false);
    rot_omega_step_tester(400.f, 800);
)

onenter(omega_step_neg200,
    id_init_log_omega();
    logger.setDirName("omega_step_v700_x");
    logger.setFileName("omega_step_neg200");
    logger.setIncludeTimestamp(false);
    rot_omega_step_tester(-200.f, 800);
)

onenter(omega_step_neg400,
    id_init_log_omega();
    logger.setDirName("omega_step_v700_x");
    logger.setFileName("omega_step_neg400");
    logger.setIncludeTimestamp(false);
    rot_omega_step_tester(-400.f, 800);
)

// 並進速度PI+FF制御の追従性検証（velocity_x_ff, config::pid_velocity_x）。
// target_velocity_xへステップ指令し，実速度(left/right_encoder_velocity平均)の追従を
// ログから確認する。duration_msは閉ループ時定数λ=0.1s基準で整定後も十分保持できる長さとする。
void velocity_step_tester(float target_velocity_x, uint32_t duration_ms) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(target_velocity_x);
    HAL_Delay(duration_ms);
    motorDriver.setBreak();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(velocity_step_300,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_300");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(300.f, 2000);
)

onenter(velocity_step_600,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_600");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(600.f, 2000);
)

onenter(velocity_step_900,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_900");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(900.f, 2000);
)

onenter(velocity_step_neg600,
    id_init_log_velocity();
    logger.setDirName("velocity_step_x");
    logger.setFileName("velocity_step_neg600");
    logger.setIncludeTimestamp(false);
    velocity_step_tester(-600.f, 2000);
)