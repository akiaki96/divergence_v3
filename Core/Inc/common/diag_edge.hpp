#pragma once

#include <cstdint>
#include <limits>

// 斜めの直線で，横のセンサー（irL / irR）が見る壁・柱の切れ目からの距離を数える。
// naophis「斜めの姿勢制御をするには」（2018-12-08）の方法：切れ目からの距離に対するセンサー値の表を作り，
// 走るときはその距離での表の値を目標にする。ここは表を作るためのデータ収集と，制御で使う「切れ目からの距離」。
//
// 左右それぞれ，値がヒステリシス（config::diag::THRESH_ON_* / THRESH_OFF_*）で壁あり→なしになった位置を
// tick 間で線形補間して覚え，since() で今の位置との差を返す（まだ切れ目がなければ NaN）。
// 位置は経路に沿った距離（Odometry::positionX() と同じ座標）。
//
// メイン側：reset() → start() … stop()。ISR：active() のときだけ update() を毎tick呼ぶ。
// デバイスに依存しない（ホストの単体試験 tools/host_test/test_diag_edge.cpp でも使う）
class DiagEdge {
public:
    enum Side : uint8_t { left, right, SIDE_COUNT };

    void reset();
    void start();
    void stop() {
        active_ = false;
    }
    bool active() const {
        return active_;
    }

    // ISR（毎tick）。value_* は横のセンサーの値，x は今の実測の並進位置
    void update(int16_t value_left, int16_t value_right, float x);

    // 最後の切れ目から今までの距離 [mm]（まだなければ NaN）
    float since(Side side) const;
    // 最後の切れ目の位置 [mm]（まだなければ NaN）
    float lastEdge(Side side) const {
        return side_[side].edge_x;
    }
    uint32_t edgeCount(Side side) const {
        return side_[side].count;
    }

    // ---- ログ用 ----
    float sinceLeft() const {
        return since(left);
    }
    float sinceRight() const {
        return since(right);
    }

private:
    struct SideState {
        bool valid = false;     // start() の後に1回でも値を見たか
        bool on = false;        // 壁あり（ヒステリシスの状態）
        float on_since = 0.f;   // [mm] 壁ありになった位置
        int16_t prev_value = 0;
        float prev_x = 0.f;
        float edge_x = std::numeric_limits<float>::quiet_NaN();   // [mm] 最後の切れ目
        uint32_t count = 0;
    };

    volatile bool active_ = false;
    float x_ = 0.f;   // [mm] 最後に update() で見た位置
    SideState side_[SIDE_COUNT];
};
