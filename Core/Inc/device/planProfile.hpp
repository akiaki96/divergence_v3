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

    // 各区間は目標位置がちょうどdistanceだけ進んだところで終わり（区間ごとの差分がちょうどdistance），
    // 次の区間はその位置と速度から連続に始まる
    void stepVelocity(float target_velocity_x, float distance);
    void stepAccel(float target_accel_x, float distance);
    void vel2vel(float velocity2, float distance);   // 初速は今の目標速度

    // 回転。angle[deg]は符号つき（正で左旋回＝ω正）。並進と同じく目標角度がちょうどangleだけ進んだところで終わり，
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
    // 1軸（並進 or 回転）の目標軌道。区間は時間の関数として解析的に生成する：
    //   x(t) = x0 + v0·t + a·t²/2,  v(t) = v0 + a·t   （0 <= t < T）
    // t >= T になったtickで x = x0 + d，v = v_end ちょうどにそろえて区間を終える（区間ごとの差分がちょうどd）。
    // 区間外（free）は v 一定で x += v·dt。区間の開始（x0の確定）は割り込み側で行う（pending → active）
    struct Axis {
        volatile float pos = 0.f;   // 目標位置 [mm] / 目標角度 [deg]
        volatile float vel = 0.f;   // 目標速度 [mm/s] / 目標角速度 [dps]
        volatile float acc = 0.f;   // 目標加速度 [mm/s^2] / 目標角加速度 [dps/s]（FF用）
        volatile bool pending = false;   // メイン→割り込み：区間の開始要求
        volatile bool active = false;    // 区間の実行中
        // 開始要求の内容（pendingをtrueにする前に書く）
        float req_v0 = 0.f, req_a = 0.f, req_T = 0.f, req_d = 0.f, req_v_end = 0.f;
        // 実行中の区間（割り込み側だけが使う）
        float t = 0.f, x0 = 0.f, v0 = 0.f, a = 0.f, T = 0.f, x_end = 0.f, v_end = 0.f;
    };

    static void advance(Axis& ax, float dt);
    static void runSegment(Axis& ax, float v0, float a, float T, float d, float v_end);
    static void setFreeVelocity(Axis& ax, float vel);
    // 区間の種類（並進・回転共通。dは符号つきの距離・角度）
    static void segmentStepVelocity(Axis& ax, float vel, float d);
    static void segmentStepAccel(Axis& ax, float acc, float d);
    static void segmentVel2Vel(Axis& ax, float vel2, float d);

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

    // 目標軌道。update()で進めてmotorDriverへ渡す
    Axis trans_;   // 並進 [mm]
    Axis rot_;     // 回転 [deg]
};
