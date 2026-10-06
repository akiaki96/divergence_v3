#include "test/closed_loop_test.hpp"
#include "device/device_instance.hpp"
#include "common/etc.hpp"
#include "common/debug.hpp"

namespace {
}

void runClosedLoopTest(const ClosedLoopTest& test) {
    test.init_log();
    logger.setDirName(test.dir);
    logger.setFileName(test.file);
    logger.setIncludeTimestamp(false);

    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    fan.stop();

    if (test.min_battery_v > 0.f) {
        float v0 = battery.voltage();
        if (v0 < test.min_battery_v) {
            LOG("test not started: battery %.2f V < %.2f V\r\n", v0, test.min_battery_v);
            for (int i = 0; i < 6; ++i) {   // 開始できない合図: LEDバー左右交互点滅 約3s
                ledBar16.set((i % 2 == 0) ? 0x00FF : 0xFF00);
                HAL_Delay(500);
            }
            ledBar16.set(0x0000);
            return;
        }
    }

    // IMU校正はファンを回す前に行う（振動がジャイロのオフセット推定に乗らないように）
    imu.calibrate();
    HAL_Delay(1100);

    if (test.fan_duty > 0.f) {
        fan.setDuty(test.fan_duty);
        HAL_Delay(config::fan::SPINUP_MS);
    }

    // 原点の取り直し：実測（エンコーダ・角度）と目標値を同時に0にそろえる
    odometry.reset();
    planProfile.reset();

    ledBar16.set(0x0000);
    logger.start();
    HAL_Delay(100);   // 静止区間：オフセット推定用
    motorDriver.switchToVelocityX();   // PIを初期化して閉ループへ切り替える（走行開始時に1回だけ）

    test.profile();   // 区間を積むだけで戻る
    planProfile.waitUntilIdle();   // 積んだ区間がすべて終わるまで待つ

    HAL_Delay(test.settle_ms);
    planProfile.stop();
    logger.stop();
    motorDriver.setBreak();
    fan.stop();
    HAL_Delay(500);
    ledBar16.set(0xFFFF);
    haltByAccZ();
    // 区間が積めなかった・落ちたことは，持ち上げた後に UART へ出す
    if (planProfile.rejectedCount() > 0 || planProfile.droppedCount() > 0) {
        LOG("plan profile: %lu segments rejected, %lu dropped\r\n",
            static_cast<unsigned long>(planProfile.rejectedCount()),
            static_cast<unsigned long>(planProfile.droppedCount()));
    }
    logger.dump();
    ledBar16.set(0x0000);
}
