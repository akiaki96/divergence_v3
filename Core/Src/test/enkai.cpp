#include "test/enkai.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "common/etc.hpp"
#include "config/node_func_maker.hpp"
#include "config/mouse_config.hpp"

// 宴会芸：位置・角度のカスケードPIで開始時の位置・向きを保持する。手で押したり回したりしても
// 元の位置・向きへ戻る（外側の位置/角度PI -> 内側の速度/角速度PI）。
// 両前IRセンサーを手で覆い続けると終了する。安全のため最大時間で打ち切る
constexpr uint32_t kEnkaiExitHoldMs = 500;     // 両前IRを覆い続けて終了とみなす時間 [ms]
constexpr uint32_t kEnkaiTimeoutMs  = 60000;   // 最大動作時間 [ms]

onenter(enkai,
    LOG("on enkai\r\n");
    motorDriver.state = MotorDriverState::setDuty;
    motorDriver.setDuty(0.f, 0.f);
    imu.calibrate();
    HAL_Delay(1100);

    // 内側：並進速度PI・角速度PI，外側：位置PI・角度PI。現在位置・角度を原点として目標0で保持する
    motorDriver.switchToVelocityX();
    motorDriver.setTargetVelocityX(0.f);   // 目標速度0（目標位置は積分されても動かない）。前の試験の値を残さない
    motorDriver.enablePositionControl();
    motorDriver.enableAngleControl();
    motorDriver.setTargetPosition(0.f);
    motorDriver.setTargetAngle(0.f);
    ledBar16.set(0xFFFF);

    uint32_t start = HAL_GetTick();
    uint32_t covered_since = 0;
    bool covered = false;
    while (HAL_GetTick() - start < kEnkaiTimeoutMs) {
        bool both_covered = irFL.filtered_ > config::mode_selector::IR_THRESH
                         && irFR.filtered_ > config::mode_selector::IR_THRESH;
        if (both_covered) {
            if (!covered) {
                covered = true;
                covered_since = HAL_GetTick();
            } else if (HAL_GetTick() - covered_since >= kEnkaiExitHoldMs) {
                break;
            }
        } else {
            covered = false;
        }
        HAL_Delay(10);
    }

    motorDriver.setBreak();
    motorDriver.disablePositionControl();
    motorDriver.disableOmegaControl();   // 角度PIも一緒に止まる
    ledBar16.set(0x0000);
    HAL_Delay(50);
)
