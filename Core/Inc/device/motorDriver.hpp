#pragma once

#include "common/etc.hpp"
#include "device/motor.hpp"
#include "common/prbs.hpp"
#include "config/mouse_config.hpp"
#include "common/pid.hpp"

enum MotorDriverState {
    off,
    modeSelecting,
    setDuty,
    lampDuty,
    prbsDuty,
    setVoltage,    // 追加：電圧指令（毎tickでVbatt補償）
    prbsVoltage,   // 追加：電圧PRBS（毎tickでVbatt補償）
    setVelocity 
};

class MotorDriver {
public:
    MotorDriver(Motor& left, Motor& right);

    void init();
    void update();

    void enable();
    void disable();

    void setDuty(float left, float right) {
        motorLeft_.setDuty(left);
        motorRight_.setDuty(right);
    }

    void setVoltage(float voltage_L, float voltage_R) {
        target_voltage_L_ = voltage_L;
        target_voltage_R_ = voltage_R;
        state = MotorDriverState::setVoltage;
    }

    void setBreak() {
        state = MotorDriverState::setDuty;
        motorLeft_.setBreak();
        motorRight_.setBreak();
        // 前試行のapplied_duty_diff_が次試行冒頭のログにリークするのを防ぐ
        // （data_analysis2/rot_step_v700_report.mdで確認された不具合）
        applied_duty_diff_ = 0.f;
    }

    void setPRBS(PRBS* prbs) {
        prbs_ = prbs;
        state = MotorDriverState::prbsDuty;
    }

    bool isPRBSFinished() const {
        return (state != MotorDriverState::prbsDuty);
    }

    void switchToVelocityX();

    void setTargetVelocityX(float velocity_x) {
        velocity_x_ = velocity_x;
    }

    // 並進速度閉ループ(setVelocity)の上に重畳する左右duty差（R-L）。
    // 回転方向のstep/PRBS同定用：並進を一定速度に保ったまま回転を励振する
    void setDutyDiff(float duty_diff) {
        duty_diff_ = duty_diff;
    }

    // duty_diffをPRBSで駆動する（回転PRBS同定用）。非nullの間はsetDutyDiff()の値より優先される。
    // 並進側のsetPRBS()と異なりstateは変更しない（setVelocityの閉ループを維持したまま励振する）
    void setPRBSDutyDiff(PRBS* prbs) {
        prbs_rot_diff_ = prbs;
    }

    bool isPRBSDutyDiffFinished() const {
        return (prbs_rot_diff_ == nullptr) || prbs_rot_diff_->isFinished();
    }

    // 回転角速度の閉ループ制御（PI，FFなし）。setVelocityX()と併用することで
    // 並進速度・角速度を同時に制御できる。setDutyDiff()/setPRBSDutyDiff()より優先度は低い
    // （それらは同定用の明示的な励振指令のため）
    // 有効化時，指令ランプの起点を現在のgyro値に合わせる（バンプレス）
    void enableOmegaControl();
    void disableOmegaControl() {
        omega_control_enabled_ = false;
        target_omega_ = 0.f;
        omega_ref_ = 0.f;
    }
    // 目標角速度（最終値）を設定する。PIに渡る指令値omega_ref_は，この値へ最大角加速度
    // （setOmegaAccelLimit(), 既定config::pid_omega::OMEGA_ACCEL_MAX）でランプする。
    // 積分時間Ti（Ki, back-calculation時定数）はランプ中の指令値omega_ref_でスケジュールする
    void setTargetOmega(float omega_dps) {
        target_omega_ = omega_dps;
    }
    // 指令のレート制限 [dps/s]。試験でステップ/ランプを切り替えるための実行時上書き
    // （1.0e9f程度でステップ指令と同等）
    void setOmegaAccelLimit(float accel_dps2) {
        omega_accel_max_ = accel_dps2;
    }

    float getTargetOmega() const {
        return target_omega_;
    }
    float getOmegaRef() const {
        return omega_ref_;
    }
    float getOmegaIntegralTerm() const {
        return pid_omega_.getIntegralTerm();
    }
    float getOmegaSaturated() const {
        return omega_saturated_ ? 1.f : 0.f;
    }

    // 追従性検証用ログ（target_velocity_x）で参照する
    float getTargetVelocityX() const {
        return velocity_x_;
    }
    // 直近tickで実際に印加されたduty差を返す（PRBS駆動中はその出力値，
    // 静的setDutyDiff()時はその値）。ログ用にupdate()内で毎tick更新される
    float getDutyDiff() const {
        return applied_duty_diff_;
    }

    // PI+FF診断用ログ：積分ワインドアップ・feedforward寄与・飽和状態を確認するため
    float getVelocityXIntegralTerm() const {
        return pid_velocity_x_.getIntegralTerm();
    }
    float getVelocityXFeedforward() const {
        return pid_velocity_x_.getLastFeedforward();
    }
    float getVelocityXSaturated() const {
        return velocity_pid_saturated_ ? 1.f : 0.f;
    }


    float getLeftDuty(void) const;
    float getRightDuty(void) const;
    void setLampGrad(float lamp);

    MotorDriverState state = MotorDriverState::setDuty;
private:
    Motor& motorLeft_;
    Motor& motorRight_;
    float target_voltage_L_ = 0.f;
    float target_voltage_R_ = 0.f;
    PRBS* prbs_ = nullptr;

    float lamp_grad_ = 0.f;

    float dutyFromVoltage(float voltage) const;

    float velocity_x_ = 0.f;
    float duty_diff_ = 0.f;
    float applied_duty_diff_ = 0.f;   // ログ用：直近tickで実際に印加されたduty差
    PRBS* prbs_rot_diff_ = nullptr;
    PIDController pid_velocity_x_;
    bool velocity_pid_saturated_ = false;

    bool omega_control_enabled_ = false;
    float target_omega_ = 0.f;            // 最終目標
    float omega_ref_ = 0.f;               // レート制限後の指令（PIに渡る値）
    float omega_accel_max_ = config::pid_omega::OMEGA_ACCEL_MAX;
    PIDController pid_omega_;
    bool omega_saturated_ = false;
};