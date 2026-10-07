#pragma once

#include <cstddef>
#include <cstdint>
#include "app/run_preset.hpp"
#include "etl/vector.h"
#include "global.h"   // uint8_vector と ACT_*（external/micromouse_simulator/solver/core）

// 最短走行の経路（time_based_dijkstra が返す ACT_* の列）を、実機で走る手順に直す。HAL に依存しない
// （ホストでもコンパイルして確かめられる）。
//
// ACT の列の幾何（シミュレータの action.json・time_based_dijkstra と同じ約束）:
//   - ターンは「基準点 → 基準点」の動作。大回り90°・180°・入出45°・入出135°の縦横側の基準点は区画中央、
//     小回り90°（ACT_TURN_*_MOVE）の基準点は区画の辺、斜め側の基準点は区画の辺の中点
//     （大回り90° は中央から中央へ (180,180)、小回り90° は辺から辺へ (90,90)、入45° は中央から辺の中点へ (90,180)）。
//     実機のスラロームの入口・出口（slalom::Anchor の center / edge / diagonal）が同じ点を指す前提
//   - ACT_MOVE_nSEC は n × 半区画、ACT_MOVE_nSEC_DIA は n × 半区画×√2。ソルバーは前後のターンの基準点の間の
//     長さを出す（小回りの前の半区画なども含めてある）ので、そのまま積めばよい
//   - 経路は (0,0) の区画中央・北向きから始まり、ゴール区画の中央で終わる。最後がターンならターンの速度のまま
//     中央に着くので、そこから減速して止まる
namespace fast_plan {

// 直線かターンの1つ
struct Step {
    const slalom::Param* turn;   // ターンのパラメータ。nullptr なら直線
    slalom::TurnDir dir;         // ターンの向き
    bool diagonal;               // 直線が斜めか（最高速度が max_speed_dia になる）
    TurnKind kind;               // ターンの種類（RunPreset::turns の添字。速度を落とすときに候補を引く）。直線では使わない
    float distance;              // [mm] 直線の長さ
};
static_assert(sizeof(Step) == sizeof(void*) + 8, "Step is stored MAX_STEPS times; keep it packed");

inline constexpr std::size_t MAX_STEPS = 128;
using Steps = etl::vector<Step, MAX_STEPS>;

enum class Error : uint8_t {
    none,
    noMotion,        // 動作が1つもない（経路がない）
    unknownAction,   // 最短走行では使わない動作がある
    turnMissing,     // プリセットにないターン（斜めを使わないプリセットに斜めのターンなど）
    tooManySteps,    // 手順が MAX_STEPS を超える
    speedUnfit,      // いちばん遅い候補まで落としても，あいだの直線で速度を変えきれない
};
const char* errorName(Error e);

// ACT の列を手順にする。start_offset [mm] は置いた位置（車軸）から (0,0) の区画中央まで。
// ゼロの長さの直線は除き、続く直線はまとめる。ターンはまず各種類のいちばん速い候補にし，fitSpeeds で合わせる
Error build(const uint8_vector& actions, const RunPreset& p, float start_offset, Steps* out);

// ターンの速度を合わせる。隣り合うターン（または始めの停止・終わりの停止）のあいだの直線で，機体の上限
// （config::profile_limit::MAX_ACCEL_X・MAX_DECEL_X。validate() と同じ）で速度を変えきれないところは，速い方のターンを同じ種類の下の候補（RunPreset::turns）に落とす。変わらなくなるまで繰り返す
// （落とすだけなので必ず終わる）。いちばん遅い候補でも合わなければ Error::speedUnfit
Error fitSpeeds(Steps* steps, const RunPreset& p);

// ソルバー（time_based_dijkstra）の時間のコストを，このプリセットの速度と各ターンの経路長にそろえる（solver_options）。
// 直線のコストは turn_speed で入って出る台形の近似，ターンはその種類のいちばん速い候補の経路長 / 速度。
// 使わないターン（候補なし）はコストを DISABLED_TURN_MS にして選ばせない。ゴールは変えない。値が不正なら false
inline constexpr uint16_t DISABLED_TURN_MS = 30000;
bool applySolverCosts(const RunPreset& p);

// PlanProfile に積む1つの区間
struct Segment {
    const slalom::Param* turn;   // ターン（slalom::push で積む）。nullptr なら並進の区間
    slalom::TurnDir dir;
    float v_end;                 // [mm/s] 並進の終速
    float distance;              // [mm]
};
inline constexpr uint8_t MAX_SEGMENTS_PER_STEP = 3;

// 手順 i を区間にして out に入れ、数を返す。直線は始めの速度（最初は 0、ほかはターンの速度）から
// 加速度 accel で最高速度まで上げ、減速度 decel で終わりの速度（最後は 0、ほかはターンの速度）まで下げる台形。
// 短くて最高速度まで届かなければ三角、加速・減速の区間がごく短くなるなら1区間で始めから終わりの速度へ
uint8_t segments(const Steps& steps, std::size_t i, const RunPreset& p, Segment out[MAX_SEGMENTS_PER_STEP]);

// すべての区間を積む前に検査する（PlanProfile と同じ validateSegment / slalom::validate、ターンに入る速度）。
// 通らなければ、その手順の番号を *bad_step に入れて結果を返す
SegmentResult validate(const Steps& steps, const RunPreset& p, std::size_t* bad_step);

// 推定の走行時間 [s]（区間の速度から計算。ログとホストでの確認用）
float estimatedTime(const Steps& steps, const RunPreset& p);

// 壁切れの補正（common/wall_edge.hpp）に教える区画境界。入口が区画中央のターン（大回り90°・180°・入45°・入135°）の
// 手前の縦横の直線にある境界（入口 − 半区画，さらに1区画ずつ手前）を，ターンごとに最大 per_turn 個。
// 壁切れを検出する予想位置（窓の幅も含めて）が直線の中に収まる境界だけ（ターンの直後の短い直線では検出できない）。
// 位置は経路に沿った距離で，置いた位置が 0（Odometry::positionX と同じ座標）。小さい順に out に入れ，数を返す
std::size_t edgeBoundaries(const Steps& steps, const RunPreset& p, int per_turn, float* out, std::size_t max);

} // namespace fast_plan
