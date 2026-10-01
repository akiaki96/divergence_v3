#pragma once

#include "common/types.hpp"

// 目標軌道の生成：並進・回転の目標位置・速度・加速度を毎tick進める。
// 区間を始める関数（straight / turn）はメインコンテキストから呼び，区間の終わりまで待って戻る。
// 制御周期の割り込みで閉ループ中にupdate()を呼び，transReference()/rotReference()をMotorDriverへ渡す
class PlanProfile {
public:
    // 目標値を0に戻す（並進・回転とも速度0・位置0）。Odometry::reset()と同時に呼んで原点をそろえる
    void reset();

    // 目標軌道を1tick進める（割り込み側，閉ループ中だけ呼ぶ）
    void update();

    // 並進：今の目標速度からv_endへ等加速度で，目標位置がちょうどdistance[mm]進んだところで戻る。
    // v_endが今の目標速度と同じなら等速。次の区間はその位置と速度から連続に始まる
    void straight(float v_end, float distance);

    // 回転：今の目標角速度からomega_endへ等角加速度で，目標角度がちょうどangle[deg]進んだところで戻る。
    // angleは符号つき（正で左旋回＝ω正）。並進の目標速度はそのまま保たれるので，走行中の旋回にも使える
    void turn(float omega_end, float angle);

    // 並進の目標速度をステップで変える（加速度0）。区間の終わりを待たない。
    // 続けてstraight(v, d)を呼べば「vへステップしてd進む」になる
    void setVelocityX(float velocity_x);

    // 並進・回転の目標加速度・目標速度を0にして止める（目標位置・目標角度はその場で保持）。各プロファイルの後に呼ぶ
    void stop();

    AxisReference transReference() const {
        return {trans_.pos, trans_.vel, trans_.acc};
    }
    AxisReference rotReference() const {
        return {rot_.pos, rot_.vel, rot_.acc};
    }

    // ---- ログ用 ----
    float getTargetPositionX() const {
        return trans_.pos;
    }

    float getTargetVelocityX() const {
        return trans_.vel;
    }

    float getTargetAccelX() const {
        return trans_.acc;
    }

    float getTargetAngle() const {
        return rot_.pos;
    }

    float getTargetOmega() const {
        return rot_.vel;
    }

private:
    // 1軸（並進 or 回転）の目標軌道。区間は時間の関数として解析的に生成する：
    //   x(t) = x0 + v0·t + a·t²/2,  v(t) = v0 + a·t   （0 <= t < T）
    // t >= T になったtickで x = x0 + d，v = v_end ちょうどにそろえて区間を終える（区間ごとの差分がちょうどd）。
    // 区間外（free）は v 一定で x += v·dt。区間の開始（x0の確定）は割り込み側で行う（pending → active）。
    // 割り込みとメインコンテキストの両方から読み書きするのでvolatileにする
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
    // 今の目標速度からv_endへ等加速度でちょうどd進む区間（並進・回転共通。dは符号つきの距離・角度）
    static void segment(Axis& ax, float v_end, float d);

    Axis trans_;   // 並進 [mm]
    Axis rot_;     // 回転 [deg]
};
