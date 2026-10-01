#pragma once

#include <cstdint>

// 閉ループ走行試験（PlanProfileで目標軌道を与える試験）の共通手順。
// init_log() → ログ保存先の設定 →（電池電圧の確認）→ IMU校正 →（ファンのスピンアップ）→ odometry/planProfileの原点取り直し
// → ログ開始 → 静止100ms → 閉ループへ切替 → profile() → settle_ms待ち → planProfile.stop()
// → ログ停止 → ブレーキ・ファン停止 → 加速度Zで待ち → ログダンプ
struct ClosedLoopTest {
    const char* dir;
    const char* file;
    void (*init_log)();   // ログ項目の登録（logger.initLoggedVal()から）
    void (*profile)();    // 走行の中身（planProfileの区間を並べる）
    float fan_duty = 0.f;        // >0ならIMU校正の後にファンを回して走る
    float min_battery_v = 0.f;   // [V] これ未満なら走らない（0で確認しない）
    uint32_t settle_ms = 500;    // profile()の後，停止指令までの待ち（目標から遅れた分を位置Pで追いつく時間）
};

void runClosedLoopTest(const ClosedLoopTest& test);
