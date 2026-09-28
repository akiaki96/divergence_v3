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

    setAccelX(0.f);
    target_velocity_x_ = 0.f;
    resetTargetPositionX();

    setAlpha(0.f);
    target_omega_ = 0.f;
    resetTargetAngle();
}

void PlanProfile::resetTargetPositionX(void) {
    encoderLeft.reset();
    encoderRight.reset();
    current_position_x_ = 0.f;
    target_position_x_ = 0.f;
}

void PlanProfile::resetTargetAngle(void) {
    angle_origin_ = imu.gyroAngleZ();
    current_angle_ = 0.f;
    target_angle_ = 0.f;
}

// 制御周期の割り込みから毎tick呼ぶ（MotorDriver::update()の前）：
// 1. 実測値（並進速度・位置，角速度・角度）を計算する
// 2. 閉ループ中は目標値（軌道）を進める：加速度→速度→位置，角加速度→角速度→角度と積分し，
//    終端速度・終端角速度に達したらそこで固定して加速度を切る（減速で0を越えて負になる・行き過ぎるのを防ぐ）
// 3. 目標値をmotorDriverへ渡す（motorDriverは受け取った目標値に追従するだけ）
void PlanProfile::update() {
    current_velocity_x_ = (encoderLeft.velocity() + encoderRight.velocity()) / 2.f;
    current_position_x_ = (encoderLeft.distance() + encoderRight.distance()) / 2.f;

    current_omega_ = imu.gyroZ();
    current_angle_ = imu.gyroAngleZ() - angle_origin_;

    if (motorDriver_.state == MotorDriverState::setVelocity) {
        target_velocity_x_ += target_accel_x_ * config::control::DT_S;
        if ((target_accel_x_ > 0.f && target_velocity_x_ >= end_velocity_x_) ||
            (target_accel_x_ < 0.f && target_velocity_x_ <= end_velocity_x_)) {
            target_velocity_x_ = end_velocity_x_;
            target_accel_x_ = 0.f;
        }
        target_position_x_ += target_velocity_x_ * config::control::DT_S;

        target_omega_ += target_alpha_ * config::control::DT_S;
        if ((target_alpha_ > 0.f && target_omega_ >= end_omega_) ||
            (target_alpha_ < 0.f && target_omega_ <= end_omega_)) {
            target_omega_ = end_omega_;
            target_alpha_ = 0.f;
        }
        target_angle_ += target_omega_ * config::control::DT_S;
    }

    motorDriver_.setTargetX(target_position_x_, target_velocity_x_);
    motorDriver_.setTargetRotation(target_angle_, target_omega_, target_alpha_);
}

// 目標加速度を設定する。end_velocity_xに達したらupdate()で目標速度をそこに固定し，加速度を0にする
void PlanProfile::setAccelX(float accel_x, float end_velocity_x) {
    target_accel_x_ = 0.f;              // 旧加速度と新しい終端速度の組で割り込みが固定しないように先に止める
    end_velocity_x_ = end_velocity_x;
    target_accel_x_ = accel_x;
}

void PlanProfile::setAccelX(float accel_x) {
    setAccelX(accel_x, (accel_x >= 0.f) ? NO_END_VELOCITY : -NO_END_VELOCITY);
}

// 目標角加速度を設定する。end_omegaに達したらupdate()で目標角速度をそこに固定し，角加速度を0にする
void PlanProfile::setAlpha(float alpha, float end_omega) {
    target_alpha_ = 0.f;
    end_omega_ = end_omega;
    target_alpha_ = alpha;
}

void PlanProfile::setAlpha(float alpha) {
    setAlpha(alpha, (alpha >= 0.f) ? NO_END_VELOCITY : -NO_END_VELOCITY);
}

// 走行開始時に1回だけ呼ぶ：速度・位置・角度のPIを初期化して閉ループへ切り替える。
// 以降の区間はPIもエンコーダ（位置の原点）もリセットせず，目標値を前の区間の終わりから連続につなぐ
// （リセットは実測速度の乱れ・出力の段差になる）
void PlanProfile::start(void) {
    motorDriver_.switchToVelocityX();
}

// 目標速度をステップで変える（加速度0）。区間の終わりを待たない（試験で時間指定の走行に使う）
void PlanProfile::setTargetVelocityX(float velocity_x) {
    setAccelX(0.f);
    target_velocity_x_ = velocity_x;
}

// 区間の終わりは目標値で判定する（実測は制御ループが追いかけるだけで，軌道の進行には関わらない）：
// 目標位置が終点に達した，または目標速度が0で止まった（減速で終端速度0に固定された）
bool PlanProfile::isSegmentDone(float x_end) const {
    return target_position_x_ >= x_end ||
           (target_accel_x_ == 0.f && target_velocity_x_ <= 0.f);
}

// 目標速度をtarget_velocity_xへステップしてdistance[mm]進む（前進のみ）。
// target_velocity_x<=0なら速度を設定してすぐ戻る（停止にはstop()を使う）
void PlanProfile::stepVelocity(float target_velocity_x, float distance) {
    float x_end = target_position_x_ + distance;
    setTargetVelocityX(target_velocity_x);

    while (!isSegmentDone(x_end)) {
        // wait
    }
}

// 目標加速度target_accel_x[mm/s^2]でdistance[mm]進み，終わったら加速度を0にする（速度はそのまま）。
// 減速（負の加速度）は目標速度0で止める（負にはしない）
void PlanProfile::stepAccel(float target_accel_x, float distance) {
    float x_end = target_position_x_ + distance;
    if (target_accel_x < 0.f) {
        setAccelX(target_accel_x, 0.f);
    } else {
        setAccelX(target_accel_x);
    }

    while (!isSegmentDone(x_end)) {
        // wait
    }
    setAccelX(0.f);
}

// 今の目標速度からvelocity2へ，distance[mm]で等加速度に変化させる（v2^2 = v1^2 + 2*a*d）。
// 初速は引数ではなく今の目標速度を使うので，前の区間から速度が連続につながる。
// 目標速度がvelocity2に達したらupdate()で固定され（a=0），velocity2=0ならそこで止まって戻る
void PlanProfile::vel2vel(float velocity2, float distance) {
    if (distance <= 0.f) return;
    float velocity1 = target_velocity_x_;
    float x_end = target_position_x_ + distance;
    float accel = (velocity2 * velocity2 - velocity1 * velocity1) / (2.f * distance);
    setAccelX(accel, velocity2);

    // 終端速度に達して（a=0）から，目標位置が終点に達する（離散化で数tickずれる）か0で止まるまで待つ
    while (!(target_accel_x_ == 0.f && isSegmentDone(x_end))) {
        // wait
    }
}

// 回転区間の終わり（目標値で判定）：目標角度が angle_start からangle（符号つき）だけ進んだ，
// または目標角速度が回転方向に対して0で止まった（減速で終端角速度0に固定された）
bool PlanProfile::isRotationDone(float angle_start, float angle) const {
    float dir = (angle >= 0.f) ? 1.f : -1.f;
    float progress = (target_angle_ - angle_start) * dir;
    return progress >= angle * dir ||
           (target_alpha_ == 0.f && target_omega_ * dir <= 0.f);
}

// 目標角速度をtarget_omegaへステップしてangle[deg]回る。target_omegaが回転方向と逆または0ならすぐ戻る
void PlanProfile::stepOmega(float target_omega, float angle) {
    if (angle == 0.f) return;
    float angle_start = target_angle_;
    setAlpha(0.f);
    target_omega_ = target_omega;

    while (!isRotationDone(angle_start, angle)) {
        // wait
    }
}

// 目標角加速度target_alphaでangle[deg]回り，終わったら角加速度を0にする（角速度はそのまま）。
// 回転方向と逆向きの角加速度（減速）は目標角速度0で止める（逆回転にはしない）
void PlanProfile::stepAlpha(float target_alpha, float angle) {
    if (angle == 0.f) return;
    float angle_start = target_angle_;
    if (target_alpha * angle < 0.f) {
        setAlpha(target_alpha, 0.f);
    } else {
        setAlpha(target_alpha);
    }

    while (!isRotationDone(angle_start, angle)) {
        // wait
    }
    setAlpha(0.f);
}

// 今の目標角速度からomega2へ，angle[deg]で等角加速度に変化させる（ω2^2 = ω1^2 + 2*α*θ，符号つきで成り立つ）。
// ω1・ω2は回転方向と同じ向き（または0）を想定。目標角速度がomega2に達したらupdate()で固定され，
// omega2=0ならそこで止まって戻る
void PlanProfile::omega2omega(float omega2, float angle) {
    if (angle == 0.f) return;
    float omega1 = target_omega_;
    float angle_start = target_angle_;
    float alpha = (omega2 * omega2 - omega1 * omega1) / (2.f * angle);
    setAlpha(alpha, omega2);

    // 終端角速度に達して（α=0）から，目標角度が終点に達するか0で止まるまで待つ
    while (!(target_alpha_ == 0.f && isRotationDone(angle_start, angle))) {
        // wait
    }
}

void PlanProfile::stop(void) {
    setAccelX(0.f);
    target_velocity_x_ = 0.f;
    setAlpha(0.f);
    target_omega_ = 0.f;
}
