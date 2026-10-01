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

    setFreeVelocity(trans_, 0.f);
    resetTargetPositionX();

    setFreeVelocity(rot_, 0.f);
    resetTargetAngle();
}

void PlanProfile::resetTargetPositionX(void) {
    encoderLeft.reset();
    encoderRight.reset();
    current_position_x_ = 0.f;
    trans_.pos = 0.f;
}

void PlanProfile::resetTargetAngle(void) {
    angle_origin_ = imu.gyroAngleZ();
    current_angle_ = 0.f;
    rot_.pos = 0.f;
}

// 制御周期の割り込みから毎tick呼ぶ（MotorDriver::update()の前）：
// 1. 実測値（並進速度・位置，角速度・角度）を計算する
// 2. 閉ループ中は目標軌道を1tick進める（advance()）
// 目標値・実測値はtransReference()などで取り出してmotorDriverへ渡す（motorDriverは受け取った目標値に追従するだけ）
void PlanProfile::update() {
    current_velocity_x_ = (encoderLeft.velocity() + encoderRight.velocity()) / 2.f;
    current_position_x_ = (encoderLeft.distance() + encoderRight.distance()) / 2.f;

    current_omega_ = imu.gyroZ();
    encoder_omega_ = (encoderRight.velocity() - encoderLeft.velocity()) / 2.f / config::pid_rotation::WHEEL_DIFF_PER_DPS;
    current_angle_ = imu.gyroAngleZ() - angle_origin_;

    if (motorDriver_.state == MotorDriverState::setVelocity) {
        advance(trans_, config::control::DT_S);
        advance(rot_, config::control::DT_S);
    }
}

// 目標軌道を1tick進める（割り込み側）。開始要求があれば今の目標位置をx0として区間を始め，
// 区間中は解析式で位置・速度を計算し，t >= T で終点（x0 + d, v_end）にちょうどそろえて終える
void PlanProfile::advance(Axis& ax, float dt) {
    if (ax.pending) {
        ax.x0 = ax.pos;
        ax.v0 = ax.req_v0;
        ax.a = ax.req_a;
        ax.T = ax.req_T;
        ax.x_end = ax.pos + ax.req_d;
        ax.v_end = ax.req_v_end;
        ax.t = 0.f;
        ax.active = true;
        ax.pending = false;
    }

    if (ax.active) {
        ax.t += dt;
        if (ax.t >= ax.T) {
            ax.pos = ax.x_end;
            ax.vel = ax.v_end;
            ax.acc = 0.f;
            ax.active = false;
        } else {
            ax.pos = ax.x0 + ax.v0 * ax.t + 0.5f * ax.a * ax.t * ax.t;
            ax.vel = ax.v0 + ax.a * ax.t;
            ax.acc = ax.a;
        }
    } else {
        ax.pos += ax.vel * dt;
        ax.acc = 0.f;
    }
}

// 区間の開始を割り込み側へ要求し，終わるまで待つ（メインコンテキスト）
void PlanProfile::runSegment(Axis& ax, float v0, float a, float T, float d, float v_end) {
    ax.req_v0 = v0;
    ax.req_a = a;
    ax.req_T = T;
    ax.req_d = d;
    ax.req_v_end = v_end;
    ax.pending = true;

    while (ax.pending || ax.active) {
        // wait
    }
}

// 区間を打ち切り，目標速度velで等速に進める（区間の終わりを待たない）
void PlanProfile::setFreeVelocity(Axis& ax, float vel) {
    ax.pending = false;
    ax.active = false;
    ax.vel = vel;
    ax.acc = 0.f;
}

// 目標速度をvelへステップしてd進む（所要時間 T = d / vel）。velがdと逆向きか0なら等速にしてすぐ戻る
void PlanProfile::segmentStepVelocity(Axis& ax, float vel, float d) {
    if (vel * d <= 0.f) {
        setFreeVelocity(ax, vel);
        return;
    }
    runSegment(ax, vel, 0.f, d / vel, d, vel);
}

// 今の目標速度からvel2へ，等加速度でちょうどd進む（T = 2d / (v0 + v2), a = (v2 − v0) / T）。
// v0・v2がdと同じ向き（または一方が0）であること
void PlanProfile::segmentVel2Vel(Axis& ax, float vel2, float d) {
    float v0 = ax.vel;
    if ((v0 + vel2) * d <= 0.f) return;
    float T = 2.f * d / (v0 + vel2);
    runSegment(ax, v0, (vel2 - v0) / T, T, d, vel2);
}

// 走行開始時に1回だけ呼ぶ：速度・位置・角度のPIを初期化して閉ループへ切り替える。
// 以降の区間はPIもエンコーダ（位置の原点）もリセットせず，目標値を前の区間の終わりから連続につなぐ
// （リセットは実測速度の乱れ・出力の段差になる）
void PlanProfile::start(void) {
    motorDriver_.switchToVelocityX();
}

// 目標速度をステップで変える（加速度0）。区間の終わりを待たない（試験で時間指定の走行に使う）
void PlanProfile::setTargetVelocityX(float velocity_x) {
    setFreeVelocity(trans_, velocity_x);
}

// 各区間は目標位置・目標角度がちょうど指定の距離・角度だけ進んだところで終わり，次の区間はそこから始まる
void PlanProfile::stepVelocity(float target_velocity_x, float distance) {
    segmentStepVelocity(trans_, target_velocity_x, distance);
}

void PlanProfile::vel2vel(float velocity2, float distance) {
    segmentVel2Vel(trans_, velocity2, distance);
}

// 回転。angle[deg]は符号つき（正で左旋回）
void PlanProfile::stepOmega(float target_omega, float angle) {
    segmentStepVelocity(rot_, target_omega, angle);
}

void PlanProfile::omega2omega(float omega2, float angle) {
    segmentVel2Vel(rot_, omega2, angle);
}

void PlanProfile::stop(void) {
    setFreeVelocity(trans_, 0.f);
    setFreeVelocity(rot_, 0.f);
}
