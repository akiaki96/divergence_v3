#pragma once

#include "device/motorDriver.hpp"

class PlanProfile {
public:
    explicit PlanProfile(MotorDriver& motorDriver);
    void init();
    void update();

    // 走行開始時に1回だけ呼ぶ（PIの初期化と閉ループへの切替）。区間の間ではリセットしない
    void start(void);

    // 目標速度をステップで変える（加速度0）。区間の終わりを待たない
    void setTargetVelocityX(float velocity_x);

    // 各区間は目標値（目標位置・目標速度）で終わりを判定し，前の区間の目標値から連続につなぐ
    void stepVelocity(float target_velocity_x, float distance);
    void stepAccel(float target_accel_x, float distance);
    void vel2vel(float velocity2, float distance);   // 初速は今の目標速度

    // 回転。angle[deg]は符号つき（正で左旋回＝ω正）。並進と同じく目標角度で終わりを判定し，
    // 前の区間の目標値から連続につなぐ。並進の目標速度はそのまま保たれるので，走行中の旋回にも使える
    void stepOmega(float target_omega, float angle);
    void stepAlpha(float target_alpha, float angle);
    void omega2omega(float omega2, float angle);   // 初速は今の目標角速度

    // 並進・回転の目標加速度・目標速度を0にして止める（目標位置・目標角度はその場で保持）。各プロファイルの後に呼ぶ
    void stop(void);

    float getCurrentVelocityX() const {
        return current_velocity_x_;
    }

    float getCurrentPositionX() const {
        return current_position_x_;
    }

    float getCurrentOmega() const {
        return current_omega_;
    }

    float getCurrentAngle() const {
        return current_angle_;
    }

    // 並進位置の原点を取り直す：エンコーダ（実測位置）と目標位置を0にする
    void resetTargetPositionX(void);

    // 角度の原点を現在の姿勢に取り直す：実測角度と目標角度を0にする
    void resetTargetAngle(void);

private:
    bool isSegmentDone(float x_end) const;
    bool isRotationDone(float angle_start, float angle) const;

    // 目標（角）加速度と終端（角）速度の設定。省略時は終端なし（かけ続ける）
    void setAccelX(float accel_x, float end_velocity_x);
    void setAccelX(float accel_x);
    void setAlpha(float alpha, float end_omega);
    void setAlpha(float alpha);

    MotorDriver& motorDriver_;

    // 以下は制御周期の割り込み（update()）とメインコンテキスト（プロファイルの待ちループ等）の両方から
    // 読み書きするためvolatileにする（最適化でループ内の読み出しが省かれないように）

    // 実測の並進速度・位置（左右エンコーダの平均）。update()で毎tick更新し，MotorDriver::update()へ渡す
    volatile float current_velocity_x_ = 0.f;
    volatile float current_position_x_ = 0.f;

    // 実測の角速度・角度（ジャイロ）。同様にupdate()で更新してMotorDriver::update()へ渡す
    volatile float current_omega_ = 0.f;  // [dps]
    volatile float current_angle_ = 0.f;  // [deg] resetTargetAngle()時の姿勢を0とする
    volatile float angle_origin_ = 0.f;   // [deg] resetTargetAngle()時のimu.gyroAngleZ()

    // 目標値（軌道）。update()で積分してmotorDriverへ渡す
    static constexpr float NO_END_VELOCITY = 1.0e9f;
    volatile float target_accel_x_ = 0.f;      // [mm/s^2]
    volatile float target_velocity_x_ = 0.f;   // [mm/s]
    volatile float target_position_x_ = 0.f;   // [mm]
    volatile float end_velocity_x_ = NO_END_VELOCITY;   // [mm/s] 目標速度の終端（setAccelX参照）
    volatile float target_alpha_ = 0.f;        // [dps/s]
    volatile float target_omega_ = 0.f;        // [dps]
    volatile float target_angle_ = 0.f;        // [deg]
    volatile float end_omega_ = NO_END_VELOCITY;        // [dps] 目標角速度の終端（setAlpha参照）
};