#include "test/fan_id.hpp"
#include "common/etc.hpp"
#include "common/debug.hpp"
#include "tim.h"

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

// ファンが回らないときの切り分け用。Fanクラス・ロガー・PWM周期変更を使わず，divergence_v2/Core/Src/fan.c と
// 同じ HAL_TIM_PWM_Start + CCR直書き(400/1000 = 40%)で3秒回し，TIM3とPB4のレジスタをシリアルに出す
static void fan_dump_regs(const char* tag) {
    LOG("[%s] TIM3 CR1=0x%04lx CCER=0x%04lx CCMR1=0x%04lx ARR=%lu CCR1=%lu CNT=%lu\r\n", tag,
        TIM3->CR1, TIM3->CCER, TIM3->CCMR1, TIM3->ARR, TIM3->CCR1, TIM3->CNT);
    LOG("[%s] PB4 MODER=%lu (2=AF) AFRL=%lu (2=AF2/TIM3) OTYPER=%lu OSPEEDR=%lu PUPDR=%lu IDR=%lu\r\n", tag,
        (GPIOB->MODER >> 8) & 3, (GPIOB->AFR[0] >> 16) & 0xF, (GPIOB->OTYPER >> 4) & 1,
        (GPIOB->OSPEEDR >> 8) & 3, (GPIOB->PUPDR >> 8) & 3, (GPIOB->IDR >> 4) & 1);
}

onenter(fan_bringup,
    ledBar16.set(0x00FF);
    fan_dump_regs("before");
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 400);
    HAL_Delay(1000);
    fan_dump_regs("running");
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    fan_dump_regs("after");
    // 通常運用の状態(PWM走行・duty0)へ戻す
    fan.init();
    ledBar16.set(0xFFFF);
)
