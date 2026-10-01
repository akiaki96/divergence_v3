#pragma once

#include <atomic>
#include <cstdint>
#include <etl/queue_spsc_atomic.h>
#include "common/types.hpp"
#include "config/mouse_config.hpp"

// 区間を積んだときの検査結果
enum class SegmentResult {
    ok,
    zeroDistance,        // 距離・角度が0（またはNaN）
    directionMismatch,   // 初速・終速が距離の向きと逆，または両方0（進めない・途中で反転する）
    accelLimit,          // 加速度が上限（config::profile_limit）を超える
    tooShort,            // 所要時間が1tick未満
    queueFull,           // 区間キューに空きがない
};

// 初速v0から終速v_endへ等加速度でd進む区間を検査する（d・速度は符号つき，加速度の上限は大きさ）。
// 積むとき（PlanProfile::straight/turn）と，固定の試験プロファイルのstatic_assertで共有する
constexpr SegmentResult validateSegment(float v0, float v_end, float d, float max_accel, float max_decel) {
    if (!(d == d) || d == 0.f) return SegmentResult::zeroDistance;
    if (!(v0 == v0) || !(v_end == v_end)) return SegmentResult::directionMismatch;
    float s = (d > 0.f) ? 1.f : -1.f;
    // 初速・終速とも進む向き（または0）で，両方0ではないこと：0割り・逆向き・途中での反転を防ぐ
    if (v0 * s < 0.f || v_end * s < 0.f || (v0 == 0.f && v_end == 0.f)) return SegmentResult::directionMismatch;
    float T = 2.f * d / (v0 + v_end);
    if (T < config::control::DT_S) return SegmentResult::tooShort;
    // 進む向きの加速度（正で増速）：v_end² = v0² + 2·a·|d|
    float a = (v_end * v_end - v0 * v0) / (2.f * d * s);
    if (a > max_accel || -a > max_decel) return SegmentResult::accelLimit;
    return SegmentResult::ok;
}

// 目標軌道の生成：並進・回転の目標位置・速度・加速度を毎tick進める。
//
// メインコンテキストは区間を区間キューに積むだけで，待たずに戻る（経路の計算などを並行して行える）。
// 制御周期の割り込み（update()）が，実行中の区間が終わると次の区間をキューから取り出して始める。
// 区間は積んだ順に1つずつ実行し，実行していない方の軸は今の目標速度のまま進む。
// 区間の初速・始点はISRが取り出した時点の目標値から決めるので，区間どうしは連続につながり，
// 各区間の差分はちょうど指定の距離・角度になる。
//
// 終わるまで待ちたいとき（試験など）だけ waitUntilIdle() を呼ぶ
class PlanProfile {
public:
    static constexpr std::size_t QUEUE_SIZE = 16;

    // 目標値を0に戻し（並進・回転とも速度0・位置0），区間キューを空にする。Odometry::reset()と同時に呼んで原点をそろえる。
    // update()が呼ばれていない間（開ループ中）に呼ぶこと
    void reset();

    // 目標軌道を1tick進める（割り込み側，閉ループ中だけ呼ぶ）
    void update();

    // 並進：その時点の目標速度からv_endへ等加速度で，目標位置がちょうどdistance[mm]進む区間を積む。
    // v_endが初速と同じなら等速。初速は直前に積んだ区間の終速（計画上の速度）として検査する
    SegmentResult straight(float v_end, float distance);

    // 回転：その時点の目標角速度からomega_endへ等角加速度で，目標角度がちょうどangle[deg]進む区間を積む。
    // angleは符号つき（正で左旋回＝ω正）。並進の目標速度はそのまま保たれるので，走行中の旋回にも使える
    SegmentResult turn(float omega_end, float angle);

    // 並進の目標速度をステップで変える（加速度0）。キューの順番どおりに，前の区間が終わったところで反映される。
    // 続けてstraight(v, d)を積めば「vへステップしてd進む」になる
    SegmentResult setVelocityX(float velocity_x);

    // 実行中の区間と積んである区間をすべて取り消し，並進・回転の目標速度・加速度を0にする
    // （目標位置・目標角度はその場で保持）。次のtickで反映され，待たずに戻る。
    // stop()の後に積んだ区間は取り消されない
    void stop();

    // 積んだ区間がすべて終わり，取り消しの反映も済んでいるか
    bool isIdle() const;

    // isIdle()になるまで待つ（閉ループ中＝update()が呼ばれている間に呼ぶこと）
    void waitUntilIdle() const;

    // 積むときに検査で弾いた区間の数（メイン側）と，ISRが取り出したときに弾いた区間の数
    uint32_t rejectedCount() const {
        return rejected_count_;
    }
    uint32_t droppedCount() const {
        return dropped_count_;
    }

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
    // 1軸の目標値（割り込み側が書き，メイン側はログ・待ちで読む）
    struct Axis {
        volatile float pos = 0.f;   // 目標位置 [mm] / 目標角度 [deg]
        volatile float vel = 0.f;   // 目標速度 [mm/s] / 目標角速度 [dps]
        volatile float acc = 0.f;   // 目標加速度 [mm/s^2] / 目標角加速度 [dps/s]（FF用）
    };

    enum class AxisId : uint8_t { translation, rotation };
    enum class EntryKind : uint8_t {
        segment,       // 等加速度でdistance進む区間（終わりまで次へ進まない）
        setVelocity,   // 目標速度のステップ（取り出したtickで反映して次へ進む）
        // 将来：イベント（壁切れ等）で終わる区間。until（判定）とd_max（打ち切り距離）を持たせる
    };

    // 区間キューの要素
    struct Entry {
        EntryKind kind;
        AxisId axis;
        float v_end;        // 終速 [mm/s] / [dps]（setVelocityでは設定する速度）
        float distance;     // 距離 [mm] / 角度 [deg]（setVelocityでは未使用）
        uint32_t generation;   // 積んだときのstop()の世代。今の世代と違えば取り消し済みとして捨てる
    };

    // 実行中の区間（割り込み側だけが書く）。x(t) = x0 + v0·t + a·t²/2,  v(t) = v0 + a·t （0 <= t < T）。
    // t >= T になったtickで x = x_end，v = v_end ちょうどにそろえて終える
    struct Segment {
        Axis* axis = nullptr;
        float t = 0.f, x0 = 0.f, v0 = 0.f, a = 0.f, T = 0.f, x_end = 0.f, v_end = 0.f;
    };

    Axis& axisOf(AxisId id) {
        return (id == AxisId::translation) ? trans_ : rot_;
    }

    SegmentResult push(const Entry& entry);
    SegmentResult pushSegment(AxisId axis, float v_end, float distance, float max_accel, float max_decel);
    void applyStop();
    void startNextEntry();
    void advance(float dt);

    Axis trans_;   // 並進 [mm]
    Axis rot_;     // 回転 [deg]

    etl::queue_spsc_atomic<Entry, QUEUE_SIZE> queue_;
    Segment segment_;
    volatile bool active_ = false;   // 区間の実行中

    // stop()の世代：メイン側がstop()で進め，割り込み側が反映したらapplied_generation_をそろえる
    std::atomic<uint32_t> generation_{0};
    volatile uint32_t applied_generation_ = 0;

    // 計画上の速度（直前に積んだ区間の終速）。積むときの検査の初速に使う（メイン側だけ）
    float planned_vel_[2] = {0.f, 0.f};

    uint32_t rejected_count_ = 0;            // メイン側だけ
    volatile uint32_t dropped_count_ = 0;    // 割り込み側だけが書く
};
