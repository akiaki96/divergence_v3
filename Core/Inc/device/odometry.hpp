#pragma once

#include <etl/queue_spsc_atomic.h>
#include "common/types.hpp"
#include "device/encoder.hpp"
#include "device/imu.hpp"

// 状態推定：エンコーダとジャイロから並進・回転の実測値を毎tick計算する。
//   並進：左右エンコーダの平均（位置・速度）
//   回転：ジャイロ（角速度と，reset()時の姿勢を0とする角度）
// 制御周期の割り込みでエンコーダ・IMUの更新後にupdate()を呼び，translation()/rotation()をMotorDriverへ渡す
class Odometry {
public:
    Odometry(Encoder& left, Encoder& right, Imu& imu);

    void update();

    // 原点を取り直す：エンコーダ（並進位置）を0にし，今の姿勢を角度0とする。壁切れの補正も0に戻す
    void reset();

    // 並進の実測位置に delta [mm] を足す（壁切れの補正，common/wall_edge.hpp）。足した分は reset() まで残る。
    // 割り込み（update() の後）から呼ぶ
    void shiftPositionX(float delta) {
        position_shift_x_ = position_shift_x_ + delta;
        position_x_ = position_x_ + delta;
    }

    // メインコンテキストから実測位置の補正を頼む（前壁の距離による補正，app/search.cpp）。
    // 次の update()（割り込み）で shiftPositionX() と同じように足す。キューが一杯なら false
    bool requestShiftX(float delta) {
        return shift_requests_.push(delta);
    }

    AxisMeasurement translation() const {
        return {position_x_, velocity_x_};
    }
    AxisMeasurement rotation() const {
        return {angle_, omega_};
    }

    // ---- ログ用 ----
    float positionX() const {
        return position_x_;
    }

    float velocityX() const {
        return velocity_x_;
    }

    float angle() const {
        return angle_;
    }

    float omega() const {
        return omega_;
    }

    // 左右のエンコーダの速度差から求めた角速度 [dps]（ジャイロと比べて滑りを見る）
    float encoderOmega() const {
        return encoder_omega_;
    }

private:
    Encoder& left_;
    Encoder& right_;
    Imu& imu_;

    // 割り込み（update()）とメインコンテキスト（ログ以外の読み出し・reset()）の両方から触るためvolatileにする
    volatile float position_x_ = 0.f;     // [mm] エンコーダの平均 + position_shift_x_
    volatile float position_shift_x_ = 0.f;   // [mm] 壁切れ・前壁の補正の合計
    etl::queue_spsc_atomic<float, 4> shift_requests_;   // メイン → 割り込み（requestShiftX）
    volatile float velocity_x_ = 0.f;     // [mm/s]
    volatile float angle_ = 0.f;          // [deg] reset()時の姿勢を0とする
    volatile float omega_ = 0.f;          // [dps]
    volatile float encoder_omega_ = 0.f;  // [dps]
    volatile float angle_origin_ = 0.f;   // [deg] reset()時のimu.gyroAngleZ()
};
