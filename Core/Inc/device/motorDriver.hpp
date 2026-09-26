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
        // （data_analysis2/03_rot_identification/rot_step_v700_report.mdで確認された不具合）
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
        angle_control_enabled_ = false;   // 角度PIは角速度PIの外側ループなので一緒に止める
        omega_control_enabled_ = false;
        target_omega_ = 0.f;
        omega_ref_ = 0.f;
        omega_ff_ = 0.f;
        pid_omega_.setExternalFF(0.f);
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

    // 2自由度FF（静的FF＋加速度FF，config::pid_omega::FF_*）の有効/無効。試験ごとにFF ON/OFFを
    // 同一セッションで比較するための実行時上書き（既定 config::pid_omega::OMEGA_FF_ENABLED）
    void setOmegaFFEnabled(bool enabled) {
        omega_ff_enabled_ = enabled;
    }
    // F6：バッテリ補償（出力に V_REF/V_batt を掛ける）と電圧基準の出力上限の有効/無効。
    // OFFで従来動作（換算なし・上限0.28）。試験でON/OFFを同一セッションで比較するための実行時上書き
    // （既定 config::pid_omega::OMEGA_BATT_COMP_ENABLED）
    void setOmegaBattCompEnabled(bool enabled) {
        omega_batt_comp_enabled_ = enabled;
    }
    // 加速度FF係数の倍率（試験でFF係数の大きさを変えて比較するための実行時上書き。既定1.0）
    void setOmegaAccelFFScale(float scale) {
        omega_accel_ff_scale_ = scale;
    }

    // 位置・角度のカスケードPI（外側ループ，目標速度FFつき。config::pid_position / config::pid_angle）。
    // 目標位置・目標角度は，setTargetVelocityX()の目標速度・setTargetOmega()の（レート制限後の）目標角速度を
    // 毎tick積分して進める。内側へは「目標速度 + 外側PIの補正」を渡す：
    //   v_cmd = v_ref + PI(x_ref − x),  ω_cmd = ω_ref + PI(θ_ref − θ)
    // setVelocity状態（switchToVelocityX()後）でのみ動作する。有効化時の現在位置/角度を原点とし，
    // 目標位置・角度0から始める（有効化直後は偏差0でバンプレス）
    void enablePositionControl();
    void disablePositionControl() {
        position_control_enabled_ = false;
    }
    // 目標位置 [mm]（原点からの相対）を直接書き換える。通常は目標速度の積分で自動的に進むので不要
    void setTargetPosition(float position_mm) {
        target_position_ = position_mm;
    }
    // 内側の角速度PIが無効なら有効化する
    void enableAngleControl();
    void disableAngleControl() {
        angle_control_enabled_ = false;
    }
    // 目標角度 [deg]（原点からの相対）を直接書き換える。通常は目標角速度の積分で自動的に進むので不要
    void setTargetAngle(float angle_deg) {
        target_angle_ = angle_deg;
    }
    // 原点（有効化時）からの位置 [mm]・角度 [deg]。ログ用
    float getPosition() const;
    float getAngle() const;
    float getTargetPosition() const {
        return target_position_;
    }
    float getTargetAngle() const {
        return target_angle_;
    }
    // 内側PIに実際に渡した指令（目標速度 + 外側PIの補正）。ログ用
    float getVelocityXCommand() const {
        return velocity_x_cmd_;
    }
    float getOmegaCommand() const {
        return omega_cmd_;
    }

    float getTargetOmega() const {
        return target_omega_;
    }
    // 直近tickでPIに加えたFFの合計 [duty]（静的＋加速度）。ログ用
    float getOmegaFF() const {
        return omega_ff_;
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
        return velocity_x_ff_;
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

    float velocity_x_ = 0.f;              // 目標並進速度 v_ref [mm/s]（setTargetVelocityX()）
    float velocity_x_cmd_ = 0.f;          // 内側PIに渡した指令 v_cmd（ログ用）
    float velocity_x_ff_ = 0.f;           // 直近tickの並進FF [V]（ログ用）
    float duty_diff_ = 0.f;
    float applied_duty_diff_ = 0.f;   // ログ用：直近tickで実際に印加されたduty差
    PRBS* prbs_rot_diff_ = nullptr;
    PIDController pid_velocity_x_;
    bool velocity_pid_saturated_ = false;

    bool omega_control_enabled_ = false;
    float target_omega_ = 0.f;            // 最終目標
    float omega_ref_ = 0.f;               // レート制限後の目標角速度 ω_ref（FF・Tiスケジュールの基準）
    float omega_cmd_ = 0.f;               // 内側PIに渡した指令 ω_cmd（角度PI無効時は omega_ref_ と同じ。ログ用）
    bool omega_ff_enabled_ = config::pid_omega::OMEGA_FF_ENABLED;
    float omega_accel_ff_scale_ = 1.f;
    bool omega_batt_comp_enabled_ = config::pid_omega::OMEGA_BATT_COMP_ENABLED;
    float omega_ff_ = 0.f;                // 直近tickのFF合計（ログ用）
    float omega_accel_max_ = config::pid_omega::OMEGA_ACCEL_MAX;
    PIDController pid_omega_;
    bool omega_saturated_ = false;

    bool position_control_enabled_ = false;
    float position_origin_ = 0.f;         // 有効化時の位置 [mm]
    float target_position_ = 0.f;         // 原点からの目標位置 [mm]
    PIDController pid_position_;

    bool angle_control_enabled_ = false;
    float angle_origin_ = 0.f;            // 有効化時の角度 [deg]
    float target_angle_ = 0.f;            // 原点からの目標角度 [deg]
    PIDController pid_angle_;
};