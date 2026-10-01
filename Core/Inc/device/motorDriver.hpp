#pragma once

#include "common/etc.hpp"
#include "device/motor.hpp"
#include "config/mouse_config.hpp"
#include "common/pid.hpp"

enum class MotorDriverState {
    setDuty,       // 開ループ：setDuty()で与えたdutyのまま（update()は何もしない）
    setVelocity    // 閉ループ：update()で目標値に追従する
};

// 1軸（並進 or 回転）の目標値。PlanProfileが毎tick生成してMotorDriver::update()へ渡す
struct AxisReference {
    float pos;   // 目標位置 [mm] / 目標角度 [deg]
    float vel;   // 目標速度 [mm/s] / 目標角速度 [dps]（各ループの外側のFF）
    float acc;   // 目標加速度 [mm/s^2] / 目標角加速度 [dps/s]（並進は加速度FFに使う。回転は使わない）
};

// 1軸の実測値。PlanProfileが毎tick計算してMotorDriver::update()へ渡す
struct AxisMeasurement {
    float pos;   // 位置 [mm]（エンコーダ平均） / 角度 [deg]（ジャイロの積分）
    float vel;   // 速度 [mm/s]（エンコーダ平均） / 角速度 [dps]（ジャイロ）
};

class MotorDriver {
public:
    MotorDriver(Motor& left, Motor& right);

    // 並進・回転の目標値と実測値（軌道生成・状態推定はPlanProfileの責務）を受け取り，制御出力を更新する
    void update(const AxisReference& trans_ref, const AxisMeasurement& trans,
                const AxisReference& rot_ref, const AxisMeasurement& rot);

    void enable();
    void disable();

    void setDuty(float left, float right) {
        motorLeft_.setDuty(left);
        motorRight_.setDuty(right);
    }

    void setBreak() {
        state = MotorDriverState::setDuty;
        motorLeft_.setBreak();
        motorRight_.setBreak();
    }

    void switchToVelocityX();

    // PI+FF診断用ログ：飽和状態を確認するため
    float getVelocityXSaturated() const {
        return (pid_velocity_x_.saturation() != 0.f) ? 1.f : 0.f;
    }

    // 角度PIの出力（角速度の指令）[dps]
    float getOmegaCommand() const {
        return omega_cmd_;
    }

    float getAngleIntegralTerm() const {
        return pid_angle_.getIntegralTerm();
    }

    float getOmegaIntegralTerm() const {
        return pid_omega_.getIntegralTerm();
    }

    MotorDriverState state = MotorDriverState::setDuty;
private:
    Motor& motorLeft_;
    Motor& motorRight_;

    float dutyFromVoltage(float voltage) const;

    // 並進：速度PI（出力は左右共通の電圧 [V]）
    PIController pid_velocity_x_{
        config::pid_velocity_x::kp, config::pid_velocity_x::ki, config::pid_velocity_x::BACK_CALC_TT
    };

    // 回転：角度PI（出力は角速度の指令 [dps]）→ 角速度PI（出力は左右の電圧差 R − L [V]）
    PIController pid_angle_{
        config::pid_rotation::ANGLE_KP, config::pid_rotation::ANGLE_KI, config::pid_rotation::ANGLE_BACK_CALC_TT
    };
    PIController pid_omega_{
        config::pid_rotation::OMEGA_KP, config::pid_rotation::OMEGA_KI, config::pid_rotation::OMEGA_TI
    };
    float omega_cmd_ = 0.f;     // [dps] 角度PIの出力（ログ用に保持）
};
