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

void step_rot(const float step) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);              // 静止区間
    motorDriver.setDuty(step, -step);   // 左右逆相：回転励振
    HAL_Delay(1500);             // 回転方向は並進より短時間で飽和しやすいので短め
    motorDriver.setBreak();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(rot_step_010,
    constexpr float step = 0.1f;
    id_init_log();
    logger.dirName = "step_rot_0_10";
    step_rot(step);
)
onenter(rot_step_015,
    constexpr float step = 0.15f;
    id_init_log();
    logger.dirName = "step_rot_0_15";
    step_rot(step);
)

onenter(rot_step_020,
    constexpr float step = 0.2f;
    id_init_log();
    logger.dirName = "step_rot_0_20";
    step_rot(step);
)

onenter(rot_step_022,
    constexpr float step = 0.22f;
    id_init_log();
    logger.dirName = "step_rot_0_22";
    step_rot(step);
)

onenter(rot_step_025,
    constexpr float step = 0.25f;
    id_init_log();
    logger.dirName = "step_rot_0_25";
    step_rot(step);
)

onenter(rot_step_030,
    constexpr float step = 0.3f;
    id_init_log();
    logger.dirName = "step_rot_0_30";
    step_rot(step);
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

onenter(rot_step_v700_002,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty002");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_1);
)

onenter(rot_step_v700_004,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty004");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_2);
)

onenter(rot_step_v700_006,
    id_init_log_rot_v700();
    logger.setDirName("rot_step_v700_x");
    logger.setFileName("rot_step_v700_duty006");
    logger.setIncludeTimestamp(false);
    rot_step_v700_tester(config::rot_step_v700::DUTY_DIFF_3);
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