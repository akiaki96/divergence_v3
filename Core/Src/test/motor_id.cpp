#include "test/motor_id.hpp"
#include "common/etc.hpp"
#include "common/prbs.hpp"

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


onenter(step_01, 
    constexpr float step = 0.1f;
    id_init_log();
    logger.dirName = "step_0_1";
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setDuty(step, step);
    HAL_Delay(2000);
    motorDriver.setBreak();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();

    logger.dump();
    ledBar16.set(0x0000);
)

onenter(step_02, 
    constexpr float step = 0.2f;
    id_init_log();
    logger.dirName = "step_0_2";
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);
    motorDriver.setDuty(step, step);
    HAL_Delay(2000);
    motorDriver.setBreak();
    HAL_Delay(50);
    logger.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();

    logger.dump();
    ledBar16.set(0x0000);
)

struct PRBSTransParams {
    uint16_t seed;
    float duty_min;
    float duty_max;
    float Tc_sec;
    float duration_sec;
};

PRBS g_prbs;

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
    logger.dirName = "prbs_trans_t01";
    prbs_trans_tester({0x1A2B, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_t02,
    id_init_log();
    logger.dirName = "prbs_trans_t02";
    prbs_trans_tester({0x3C4D, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_t03,
    id_init_log();
    logger.dirName = "prbs_trans_t03";
    prbs_trans_tester({0x5E6F, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_t04,
    id_init_log();
    logger.dirName = "prbs_trans_t04";
    prbs_trans_tester({0x7890, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_t05,
    id_init_log();
    logger.dirName = "prbs_trans_t05";
    prbs_trans_tester({0xABCD, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_t06,
    id_init_log();
    logger.dirName = "prbs_trans_t06";
    prbs_trans_tester({0xEF01, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_val01,
    id_init_log();
    logger.dirName = "prbs_trans_val01";
    prbs_trans_tester({0x2468, 0.08f, 0.16f, 0.15f, 3.0f});
)

onenter(prbs_trans_val02,
    id_init_log();
    logger.dirName = "prbs_trans_val02";
    prbs_trans_tester({0x1357, 0.08f, 0.16f, 0.15f, 3.0f});
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