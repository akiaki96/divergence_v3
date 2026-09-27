#include "device/planProfile.hpp"
#include "device/device_instance.hpp"

PlanProfile::PlanProfile(MotorDriver& motorDriver)
    : motorDriver_(motorDriver)
{}

// 軌道追従を始める前にmotorDriverを初期状態にそろえる：
// モータを止め，並進・回転の目標値（加速度・速度・位置／角加速度・角速度・角度）を0にし，
// 位置と角度の原点を現在値に取り直す。PIDゲインはMotorDriver::init()で設定済み
void PlanProfile::init() {
    state = PlanProfileState::off;

    motorDriver_.state = MotorDriverState::setDuty;
    motorDriver_.setDuty(0.f, 0.f);

    motorDriver_.setTargetAccelX(0.f);
    motorDriver_.setTargetVelocityX(0.f);
    motorDriver_.resetTargetPositionX();

    motorDriver_.setTargetAlpha(0.f);
    motorDriver_.setTargetOmega(0.f);
    motorDriver_.resetTargetAngle();
}