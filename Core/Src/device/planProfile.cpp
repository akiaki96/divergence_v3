#include "device/planProfile.hpp"
#include "device/device_instance.hpp"

PlanProfile::PlanProfile(MotorDriver& motorDriver)
    : motorDriver_(motorDriver)
{}

// 軌道追従を始める前にmotorDriverを初期状態にそろえる：
// モータを止め，並進・回転の目標値（加速度・速度・位置／角加速度・角速度・角度）を0にし，
// 位置と角度の原点を現在値に取り直す。PIDゲインはMotorDriver::init()で設定済み
void PlanProfile::init() {
    motorDriver_.state = MotorDriverState::setDuty;
    motorDriver_.setDuty(0.f, 0.f);

    motorDriver_.setTargetAccelX(0.f);
    motorDriver_.setTargetVelocityX(0.f);
    resetTargetPositionX();

    motorDriver_.setTargetAlpha(0.f);
    motorDriver_.setTargetOmega(0.f);
    motorDriver_.resetTargetAngle();
    resetCurrentAngle();
}

void PlanProfile::resetTargetPositionX(void) {
    encoderLeft.reset();
    encoderRight.reset();
    current_position_x_ = 0.f;
    motorDriver_.setTargetPositionX(0.f);
}

void PlanProfile::resetCurrentAngle(void) {
    angle_origin_ = imu.gyroAngleZ();
    current_angle_ = 0.f;
}

void PlanProfile::update() {
    current_velocity_x_ = (encoderLeft.velocity() + encoderRight.velocity()) / 2.f;
    current_position_x_ = (encoderLeft.distance() + encoderRight.distance()) / 2.f;

    current_omega_ = imu.gyroZ();
    current_angle_ = imu.gyroAngleZ() - angle_origin_;
}


// 走行開始時に1回だけ呼ぶ：速度・位置・角度のPIを初期化して閉ループへ切り替える。
// 以降の区間（stepVelocity/stepAccel/vel2vel）はPIもエンコーダ（位置の原点）もリセットせず，
// 目標値を前の区間の終わりから連続につなぐ（リセットは実測速度の乱れ・出力の段差になる）
void PlanProfile::start(void) {
    motorDriver_.switchToVelocityX();
}

// 区間の終わりは目標値で判定する（実測は制御ループが追いかけるだけで，軌道の進行には関わらない）：
// 目標位置が終点に達した，または目標速度が0で止まった（減速で終端速度0に固定された）
bool PlanProfile::isSegmentDone(float x_end) const {
    return motorDriver_.getTargetPositionX() >= x_end ||
           (motorDriver_.getTargetAccelX() == 0.f && motorDriver_.getTargetVelocityX() <= 0.f);
}

// 目標速度をtarget_velocity_xへステップしてdistance[mm]進む（前進のみ）。
// target_velocity_x<=0なら速度を設定してすぐ戻る（停止にはstop()を使う）
void PlanProfile::stepVelocity(float target_velocity_x, float distance) {
    float x_end = motorDriver_.getTargetPositionX() + distance;
    motorDriver_.setTargetAccelX(0.f);
    motorDriver_.setTargetVelocityX(target_velocity_x);

    while (!isSegmentDone(x_end)) {
        // wait
    }
}

// 目標加速度target_accel_x[mm/s^2]でdistance[mm]進み，終わったら加速度を0にする（速度はそのまま）。
// 減速（負の加速度）は目標速度0で止める（負にはしない）
void PlanProfile::stepAccel(float target_accel_x, float distance) {
    float x_end = motorDriver_.getTargetPositionX() + distance;
    if (target_accel_x < 0.f) {
        motorDriver_.setTargetAccelX(target_accel_x, 0.f);
    } else {
        motorDriver_.setTargetAccelX(target_accel_x);
    }

    while (!isSegmentDone(x_end)) {
        // wait
    }
    motorDriver_.setTargetAccelX(0.f);
}

// 今の目標速度からvelocity2へ，distance[mm]で等加速度に変化させる（v2^2 = v1^2 + 2*a*d）。
// 初速は引数ではなく今の目標速度を使うので，前の区間から速度が連続につながる。
// 目標速度がvelocity2に達したら割り込み側で固定され（a=0），velocity2=0ならそこで止まって戻る
void PlanProfile::vel2vel(float velocity2, float distance) {
    if (distance <= 0.f) return;
    float velocity1 = motorDriver_.getTargetVelocityX();
    float x_end = motorDriver_.getTargetPositionX() + distance;
    float accel = (velocity2 * velocity2 - velocity1 * velocity1) / (2.f * distance);
    motorDriver_.setTargetAccelX(accel, velocity2);

    // 終端速度に達して（a=0）から，目標位置が終点に達する（離散化で数tickずれる）か0で止まるまで待つ
    while (!(motorDriver_.getTargetAccelX() == 0.f && isSegmentDone(x_end))) {
        // wait
    }
}
void PlanProfile::stop(void) {
    motorDriver_.setTargetAccelX(0.f);
    motorDriver_.setTargetVelocityX(0.f);
}
