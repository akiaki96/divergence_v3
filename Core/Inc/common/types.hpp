#pragma once

enum class Direction {
    Normal,
    Reversed
};

enum class Enable {
    Enable,
    Disable
};

// 1軸（並進 or 回転）の目標値。PlanProfileが毎tick生成してMotorDriver::update()へ渡す
struct AxisReference {
    float pos;   // 目標位置 [mm] / 目標角度 [deg]
    float vel;   // 目標速度 [mm/s] / 目標角速度 [dps]（各ループの外側のFF）
    float acc;   // 目標加速度 [mm/s^2] / 目標角加速度 [dps/s]（並進は加速度FFに使う。回転は使わない）
};

// 1軸の実測値。Odometryが毎tick計算してMotorDriver::update()へ渡す
struct AxisMeasurement {
    float pos;   // 位置 [mm]（エンコーダ平均） / 角度 [deg]（ジャイロの積分）
    float vel;   // 速度 [mm/s]（エンコーダ平均） / 角速度 [dps]（ジャイロ）
};
