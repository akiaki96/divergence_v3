#include "test/fan_id.hpp"
#include "common/etc.hpp"

// ファン試験のタイムライン [ms]（logger容量: 3フィールド×8000サンプル=8s に収まる）
namespace {
constexpr uint32_t FAN_PRE_OFF_MS  = 1000;   // ファンOFF区間（電圧のベースライン）
constexpr uint32_t FAN_ON_MS       = 3000;   // ファンON区間（スピンアップ後の定常電圧を見る）
constexpr uint32_t FAN_POST_OFF_MS = 2000;   // ファンOFF区間（電圧の回復・ドリフト確認）

// TIM6(1kHz)とTIM3(64kHz)は同一クロックで比が64:1ちょうどのため，そのままだとバッテリADCが
// 毎回PWMの同じ位相を拾い，PWMリプル分の偏りがブート毎に固定で乗る。PWM周期を1000と互いに素な
// 値にして位相を巡回させ，ON区間の平均でリプルを消す（試験後に元へ戻す）
constexpr uint32_t FAN_PWM_PERIOD_TEST = 997;
constexpr uint32_t FAN_PWM_PERIOD_NORMAL = 1000;
}

// ログは最小限（Global_time, battery, fan_duty）。1kHzサンプルでON区間の電圧降下を記録する
static void fan_init_log(void) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);   // 車輪モーターの電流をベースラインに含めないため停止させておく
    fan.stop();

    logger.initLoggedVal();
    logger.add(
        "battery",
        etl::delegate<float()>::create<Battery, &Battery::voltage>(battery)
    );
    logger.add(
        "fan_duty",
        etl::delegate<float()>::create<Fan, &Fan::getDuty>(fan)
    );

    ledBar16.set(0xFFFF);
}

// in: duty(ファンのduty 0〜1) / out: なし（実行後にdumpする）
static void fan_tester(float duty) {
    fan.setPwmPeriod(FAN_PWM_PERIOD_TEST);
    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(FAN_PRE_OFF_MS);
    fan.setDuty(duty);
    HAL_Delay(FAN_ON_MS);
    fan.stop();
    HAL_Delay(FAN_POST_OFF_MS);
    logger.stop();
    fan.setPwmPeriod(FAN_PWM_PERIOD_NORMAL);
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    logger.dump();
    ledBar16.set(0x0000);
}

onenter(fan_run_025,
    fan_init_log();
    logger.setDirName("fan_vsag_m1");
    logger.setFileName("fan_025");
    logger.setIncludeTimestamp(false);
    fan_tester(0.25f);
)

onenter(fan_run_050,
    fan_init_log();
    logger.setDirName("fan_vsag_m1");
    logger.setFileName("fan_050");
    logger.setIncludeTimestamp(false);
    fan_tester(0.50f);
)

onenter(fan_run_075,
    fan_init_log();
    logger.setDirName("fan_vsag_m1");
    logger.setFileName("fan_075");
    logger.setIncludeTimestamp(false);
    fan_tester(0.75f);
)

onenter(fan_run_100,
    fan_init_log();
    logger.setDirName("fan_vsag_m1");
    logger.setFileName("fan_100");
    logger.setIncludeTimestamp(false);
    fan_tester(1.00f);
)
