#pragma once

#include "common/etc.hpp"
#include "device/motor.hpp"
#include "config/mouse_config.hpp"
#include "common/axis_controller.hpp"
#include "common/types.hpp"

enum class MotorDriverState {
    setDuty,       // 開ループ：setDuty()で与えたdutyのまま（update()は何もしない）
    setVelocity    // 閉ループ：update()で目標値に追従する
};

class MotorDriver {
public:
    MotorDriver(Motor& left, Motor& right);

    // 並進・回転の目標値（PlanProfile）と実測値（Odometry）を受け取り，制御出力を更新する。
    // angle_ki_scaleは回転の角度PIのKiに掛ける倍率（PlanProfile::angleKiScale()，止まって向きを合わせる間だけ大きい）
    void update(const AxisReference& trans_ref, const AxisMeasurement& trans,
                const AxisReference& rot_ref, const AxisMeasurement& rot, float angle_ki_scale = 1.f);

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

    // ---- ログ用 ----
    // 並進の速度PIの飽和（1:飽和, 0:なし）
    float getVelocityXSaturated() const {
        return (trans_.saturation() != 0.f) ? 1.f : 0.f;
    }

    // 角度PIの出力（角速度の指令）[dps]
    float getOmegaCommand() const {
        return rot_.command();
    }

    float getAngleIntegralTerm() const {
        return rot_.outerIntegralTerm();
    }

    float getOmegaIntegralTerm() const {
        return rot_.innerIntegralTerm();
    }

    MotorDriverState state = MotorDriverState::setDuty;
private:
    Motor& motorLeft_;
    Motor& motorRight_;

    float dutyFromVoltage(float voltage) const;

    // 並進：位置P（出力は速度指令 [mm/s]）→ 速度PI（出力は左右共通の電圧 [V]）
    AxisController trans_{
        PIController{config::pid_position_x::kp, 0.f, 1.f},
        config::pid_position_x::VELOCITY_CMD_LIMIT,
        PIController{config::pid_velocity_x::kp, config::pid_velocity_x::ki, config::pid_velocity_x::BACK_CALC_TT}
    };

    // 回転：角度PI（出力は角速度の指令 [dps]）→ 角速度PI（出力は左右の電圧差 R − L [V]）
    AxisController rot_{
        PIController{config::pid_rotation::ANGLE_KP, config::pid_rotation::ANGLE_KI, config::pid_rotation::ANGLE_BACK_CALC_TT},
        config::pid_rotation::OMEGA_CMD_LIMIT,
        PIController{config::pid_rotation::OMEGA_KP, config::pid_rotation::OMEGA_KI, config::pid_rotation::OMEGA_TI}
    };
};
