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
    logger.add<&Battery::voltage>("battery", battery);
    logger.add<&Fan::getDuty>("fan_duty", fan);
    logger.setDuration(FAN_PRE_OFF_MS + FAN_ON_MS + FAN_POST_OFF_MS + 500);

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

onenter(fan_run_010,
    fan_init_log();
    logger.setDirName("fan_vsag_m2");
    logger.setFileName("fan_010");
    logger.setIncludeTimestamp(false);
    fan_tester(0.10f);
)

onenter(fan_run_020,
    fan_init_log();
    logger.setDirName("fan_vsag_m2");
    logger.setFileName("fan_020");
    logger.setIncludeTimestamp(false);
    fan_tester(0.20f);
)

onenter(fan_run_030,
    fan_init_log();
    logger.setDirName("fan_vsag_m2");
    logger.setFileName("fan_030");
    logger.setIncludeTimestamp(false);
    fan_tester(0.30f);
)

onenter(fan_run_040,
    fan_init_log();
    logger.setDirName("fan_vsag_m2");
    logger.setFileName("fan_040");
    logger.setIncludeTimestamp(false);
    fan_tester(0.40f);
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

// ---- 連続運転モード -------------------------------------------------------------------
// ファンを一定dutyで回し続ける。止めるには機体を大きく傾ける（haltByAccZと同じ条件: 加速度Zが
// 基準から1G以上下がる）。電池が下がった場合も自動で止める（LiPo 2Sの過放電防止）。
namespace {
constexpr float FAN_HOLD_MIN_START_V = 6.8f;   // [V] これ未満なら開始しない（USB給電のみ・電池未接続・低電圧の検出も兼ねる）
constexpr float FAN_HOLD_CUTOFF_V    = 6.4f;   // [V] 運転中にこれを下回ったら停止（ファン負荷時の電圧で判定するので保守側）
constexpr uint32_t FAN_HOLD_POLL_MS  = 50;
constexpr uint32_t FAN_HOLD_LOG_MS   = 1000;   // 状態をシリアルに出す周期
}

// in: duty(0〜1) / out: なし（傾けるか電池低下で戻る）
static void fan_hold(float duty) {
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);   // 車輪は止めておく

    float v0 = battery.voltage();
    if (v0 < FAN_HOLD_MIN_START_V) {
        LOG("fan hold not started: battery %.2f V < %.2f V (LiPo not connected / low?)\r\n", v0, FAN_HOLD_MIN_START_V);
        for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s
            ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
            HAL_Delay(500);
        }
        ledBar16.set(0x0000);
        return;
    }

    LOG("fan hold: duty %.2f, battery %.2f V. Tilt the robot to stop.\r\n", duty, v0);
    ledBar16.set(0x00FF);
    fan.setDuty(duty);

    const char* reason = "tilt";
    uint32_t elapsed_ms = 0;
    while (imu.accelZ() + config::imu::G > config::mode_selector::ACC_THRESH) {
        float v = battery.voltage();
        if (v < FAN_HOLD_CUTOFF_V) {
            reason = "battery low";
            break;
        }
        if (elapsed_ms % FAN_HOLD_LOG_MS == 0) {
            LOG("fan hold: t=%lu s duty=%.2f battery=%.2f V\r\n", elapsed_ms / 1000, fan.getDuty(), v);
        }
        HAL_Delay(FAN_HOLD_POLL_MS);
        elapsed_ms += FAN_HOLD_POLL_MS;
    }

    fan.stop();
    LOG("fan hold stopped (%s) after %lu ms, battery %.2f V\r\n", reason, elapsed_ms, battery.voltage());
    ledBar16.set(0xFFFF);
    HAL_Delay(500);
    ledBar16.set(0x0000);
}

onenter(fan_hold_010, fan_hold(0.10f);)
onenter(fan_hold_020, fan_hold(0.20f);)
onenter(fan_hold_030, fan_hold(0.30f);)
onenter(fan_hold_040, fan_hold(0.40f);)
