#pragma once

#include <cstdint>
#include "common/diag_edge.hpp"
#include "common/types.hpp"
#include "config/mouse_config.hpp"

// 斜めの直線の向きの補正（naophis「斜めの姿勢制御をするには」の表を使う方法。考え方と定数は config::diag_control，
// 手順は tools/DIAGONAL.md）。
//
// 横のセンサー（irL / irR）が最後の柱の切れ目（DiagEdge）から since 進んだところでの値を，表
// （config/diag_table.hpp：中心線上の基準値と横ずれへの感度）と比べて横のずれ y [mm]（左が正）にする。
// 両側が読めれば平均する。表は柱の両側に壁がある並べ方で取ったので，壁が抜けると値が下がる：大きく離れて
// 読めた側（config::diag_control::MAX_AWAY_MM）は壁の抜けとみなして使わない。y をならして向きの目標を「入ったときの向き + 積分 − KP·y」にし，
// 足した向きは WallControl と同じく回転の目標の角度に足し続ける（旋回の後も残る）。
//
// 斜めの直線は経路に沿った位置の範囲 [x0, x1)（Odometry::positionX() と同じ座標）で教える。範囲に入るたびに
// 積分とならしを始め直し，範囲の外・旋回中（目標の角速度が 0 でない）・遅いときは何もしない。
//
// メイン側：reset() → setRanges() → start(correction) … stop()。ISR：毎tick apply()。
// correction が false なら横のずれを出すだけで向きは変えない（ログで確かめる用）。
// デバイスに依存しない（ホストの単体試験 tools/host_test/test_diag_control.cpp でも使う）
class DiagControl {
public:
    struct Range {
        float x0;   // [mm] 斜めの直線の始まり（出口が斜めのターンの出口の基準点）
        float x1;   // [mm] 終わり（次のターンの入口の基準点）
    };
    static constexpr uint8_t MAX_RANGES = config::diag_control::MAX_RANGES;

    // 補正を0に戻して止め，範囲を消す（PlanProfile::reset() と同時に）
    void reset();
    // 範囲を位置の小さい順に教える（止めている間に）。入りきらなければ入った数を返す
    uint8_t setRanges(const Range* ranges, uint8_t n);
    void start(bool correction) {
        correction_ = correction;
        enabled_ = true;
    }
    void stop() {
        enabled_ = false;
    }
    // 試験用：次に入る斜めの直線の始まりで，回転の目標を deg だけずらしたままにする（制御には見せない：
    // 入口の向きのずれ θ0 と同じ。補正がなければ横へ θ0·距離 だけずれていく）
    void injectAngle(float deg) {
        inject_deg_ = deg;
    }

    // ISR（毎tick）。止めている間は rot をそのまま返す。rot は回転の目標，omega_target / v_target は今の目標（直進中かの判定），x は実測の並進位置，
    // value_* は横のセンサーの値，edge は切れ目の検出（ISR で先に update() したもの）。補正を足した回転の目標を返す
    AxisReference apply(const AxisReference& rot, float omega_target, float v_target, float x, int16_t value_left,
                        int16_t value_right, const DiagEdge& edge);

    // 表から片側の寄り [mm]（その側へ寄ったら正，表の外・感度 0 なら NaN）。寄っていなければ負もありうる
    static float sideOffset(DiagEdge::Side side, float since, int16_t value);

    // ---- ログ用 ----
    float lateral() const {   // [mm] ならした横のずれ（左が正）
        return lateral_;
    }
    float omega() const {   // [dps] 今のtickの補正
        return omega_;
    }
    float offset() const {   // [deg] 補正の積分（向きに足している分。注入したずれは入らない）
        return offset_deg_;
    }
    float integral() const {   // [deg] 今の斜めの直線での積分
        return integral_deg_;
    }
    float measured() const {   // 今のtickに表を引けた側（1：左，2：右，3：両方，0：なし）
        return static_cast<float>(measured_);
    }
    uint32_t activeTicks() const {
        return active_ticks_;
    }
    uint32_t measuredTicks() const {
        return measured_ticks_;
    }

private:
    volatile bool enabled_ = false;
    bool correction_ = false;
    Range ranges_[MAX_RANGES] = {};
    uint8_t range_count_ = 0;
    uint8_t range_index_ = 0;    // 今の（次の）範囲
    bool in_range_ = false;      // 今の範囲に入って始め直したか
    float inject_deg_ = 0.f;
    float bias_deg_ = 0.f;       // [deg] 注入したずれ（回転の目標に足すが，制御の向きの目標には入れない）

    float anchor_deg_ = 0.f;     // [deg] 範囲に入ったときの offset_deg_
    float integral_deg_ = 0.f;
    float lateral_ = 0.f;
    float unmeasured_mm_ = 0.f;  // [mm] 最後に読めてから走った距離
    float omega_ = 0.f;
    float offset_deg_ = 0.f;
    uint8_t measured_ = 0;
    uint32_t active_ticks_ = 0;
    uint32_t measured_ticks_ = 0;
};
