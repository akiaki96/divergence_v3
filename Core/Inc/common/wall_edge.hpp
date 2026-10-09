#pragma once

#include <cstddef>
#include <cstdint>
#include <etl/queue_spsc_atomic.h>

// 壁切れによる並進の距離の補正。
//
// 直進中に横のセンサー（左・右）の値が config::wall_edge::THRESH_OFF_* を下回った（横壁が切れた）位置を
// 1tickの間で線形補間して求め，メイン側が expect() で教えた区画境界 b と対応づける。
// 検出したときの車軸の位置は b + OFFSET_* + LAG_S·v のはずなので，実測との差を補正量として返す。
// ISR は補正量を Odometry::shiftPositionX() で実測位置に足す（目標軌道はそのまま。差は位置Pで追いつく）。
//
// 区画境界は経路に沿った距離（PlanProfile の目標位置・Odometry の実測と同じ座標）で与える。
// 補正しないとき（start(false)）も壁切れは記録するので，calib の試験と探索のログに使える。
//
// メイン側：reset() → expect() …（走りながら積む）→ start() … stop() → event() で読む
// ISR：active() のときだけ update() を毎tick呼ぶ
//
// デバイスに依存しない（ホストの単体試験 tools/host_test/test_wall_edge.cpp でも使う）
class WallEdge {
public:
    enum Side : uint8_t { left, right, SIDE_COUNT };

    // 1回の壁切れの記録
    struct Event {
        float x;          // [mm] 壁切れの位置（補正前の実測，tick間を補間）
        float boundary;   // [mm] 対応づけた区画境界。見つからなければ NaN
        float shift;      // [mm] 実測に足した補正（補正しないとき・対応がないときは 0）
        float velocity;   // [mm/s] そのときの目標速度
        uint8_t side;     // Side
    };

    // 記録の上限（超えた分は記録しないが補正は続ける）。5×7 の探索で 55 件まで出たので，16×16 の長い経路でも
    // 足りるように 128（1件 20 B，2.5 KB）
    static constexpr std::size_t MAX_EVENTS = 128;
    static constexpr std::size_t QUEUE_SIZE = 8;     // expect() のキュー
    static constexpr std::size_t MAX_PENDING = 4;    // ISR が同時に待つ区画境界

    // 止めて記録・待ちの境界を消す。ISR が update() を呼んでいない間か，active() でない間に呼ぶ
    void reset();

    // 検出を始める。correct が false なら記録だけ。対応づけの幅 [mm]：予想より前は window_mm，後ろは window_late_mm
    // （既定 config::wall_edge::WINDOW_MM / WINDOW_LATE_MM。幅を1つだけ渡すと前後とも同じ）
    void start(bool correct, float window_mm, float window_late_mm);
    void start(bool correct, float window_mm);
    void start(bool correct);
    void stop() {
        active_ = false;
    }

    bool active() const {
        return active_;
    }

    // 対応づけの幅 [mm]（start() で決めた値）。window() は予想より前（補正が正），windowLate() は後ろ（補正が負）
    float window() const {
        return window_;
    }
    float windowLate() const {
        return window_late_;
    }

    // メイン側：経路に沿った距離 boundary_mm に区画境界がある（次に通る順に積む）。キューが一杯なら false
    bool expect(float boundary_mm);

    // ISR（毎tick）。value_* は横のセンサーの値，x は今の実測の並進位置，v_target / omega_target は今の目標。
    // 実測位置に足す補正量 [mm] を返す（補正しないときは 0）
    float update(int16_t value_left, int16_t value_right, float x, float v_target, float omega_target);

    // 検出したときの車軸の位置の予想（境界 b，目標速度 v）
    static float expectedX(Side side, float boundary, float v);

    uint32_t eventCount() const {
        return event_count_;
    }
    const Event& event(uint32_t i) const {
        return events_[i];
    }

    // ---- ログ用 ----
    float totalShift() const {
        return total_shift_;
    }

private:
    struct SideState {
        bool valid = false;     // 直進に入ってから1回でも値を見たか
        bool on = false;        // 壁あり（ヒステリシスの状態）
        float on_since = 0.f;   // [mm] 壁ありになった位置
        int16_t prev_value = 0;
        float prev_x = 0.f;
    };

    void drainQueue();
    void expire(float x, float v);
    float onFallingEdge(Side side, float x_edge, float v);
    void record(const Event& e);

    volatile bool active_ = false;
    bool correct_ = false;
    float window_ = 0.f;
    float window_late_ = 0.f;

    etl::queue_spsc_atomic<float, QUEUE_SIZE> queue_;   // メイン → ISR
    float pending_[MAX_PENDING] = {};                    // ISR だけ
    std::size_t pending_count_ = 0;

    SideState side_[SIDE_COUNT];

    Event events_[MAX_EVENTS] = {};
    volatile uint32_t event_count_ = 0;
    volatile float total_shift_ = 0.f;
};
